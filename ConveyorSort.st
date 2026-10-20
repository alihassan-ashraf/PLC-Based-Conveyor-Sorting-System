(*
================================================================================
  PROGRAM   : ConveyorSort
  PROJECT   : DecodeLabs Robotics & Automation Internship -- Project 4
  AUTHOR    : Naveed Ali
  LANGUAGE  : IEC 61131-3 Structured Text
  TARGET    : Siemens S7-1200/1500 (SCL) / CODESYS / TwinCAT 3
================================================================================

  FUNCTION
  --------
  Sorts boxes on a powered conveyor. A photo-eye detects each box arriving; a
  height sensor classifies it; tall boxes are diverted by a pneumatic pusher
  1.5 s later, when the box has travelled the 0.75 m from sensor to pusher.
  Five accepted boxes complete a batch. An E-Stop halts everything and latches
  a fault that requires a deliberate operator acknowledge.

  SAFETY NOTE -- READ BEFORE MODIFYING
  ------------------------------------
  This program does NOT stop the machine. It cannot, and must not be relied on
  to. The E-Stop button breaks the coil circuit of a hardware safety relay,
  which drops motor power directly (Stop Category 0 per IEC 60204-1). %I0.2 is
  a MONITORING contact from that relay -- it tells the PLC what the hardware
  already did. The software layer exists for diagnostics, latching and orderly
  restart, never for the stop itself. A PLC with a corrupted scan cycle must
  still result in a stopped machine.

  DEVIATION FROM THE SUPPLIED SPECIFICATION
  -----------------------------------------
  The project brief specifies i_Stop_PB as normally-closed (NC) but writes the
  start transition as:

        T_1->2 = i_Start_PB AND NOT(i_Stop_PB) AND ...

  Those two statements contradict each other. On an NC button the circuit is
  CLOSED when the button is at rest, so the PLC input reads TRUE when the
  machine is healthy and FALSE when the button is pressed OR the wire is cut.
  Taking NOT() of that would mean the conveyor can only be started while
  somebody is holding the stop button down -- and, far worse, a severed stop
  wire would read as "safe to run".

  This implementation keeps the NC wiring, which is the part that is
  safety-relevant and non-negotiable, and normalises it into an explicit
  internal request flag:

        b_Stop_Request := NOT i_Stop_PB_NC;   // open circuit = stop demanded

  The transition then reads NOT(b_Stop_Request), preserving the brief's logical
  shape while making a broken wire fail toward a stop instead of away from one.

================================================================================
*)

PROGRAM ConveyorSort
VAR_INPUT
    (* ---------------- Physical inputs, 24 VDC discrete ---------------- *)
    i_Start_PB      AT %I0.0 : BOOL;  (* Start push button, NO contact        *)
    i_Prox_Sensor   AT %I0.1 : BOOL;  (* Box-present photo-eye                *)
    i_EStop_Mon     AT %I0.2 : BOOL;  (* Safety-relay monitor, TRUE = healthy *)
    i_Stop_PB_NC    AT %I0.3 : BOOL;  (* Stop push button, NC: TRUE = healthy *)
    i_Tall_Sensor   AT %I0.4 : BOOL;  (* High-level height sensor             *)
END_VAR

VAR_OUTPUT
    (* ---------------- Physical outputs ---------------- *)
    q_Conv_Motor    AT %Q1.0 : BOOL;  (* Belt drive contactor         *)
    q_Reject_Push   AT %Q1.1 : BOOL;  (* Diverter solenoid valve      *)
    q_Warn_Light    AT %Q1.2 : BOOL;  (* Amber warning beacon         *)
END_VAR

VAR
    (* ---------------- State machine ---------------- *)
    (* Mutually exclusive by construction: eState holds exactly one value, so
       two states can never drive the same output in one scan. A set of
       independent BOOLs for Idle/Running/Sorting/Fault would allow exactly
       that race, which is the bug this structure exists to prevent. *)
    eState          : INT := STATE_IDLE;
    ePrevState      : INT;

    (* ---------------- Latches and flags ---------------- *)
    b_EStop_Fault   : BOOL := FALSE;  (* Latched; cleared only by operator  *)
    b_Batch_Complete: BOOL := FALSE;
    b_Batch_Prev    : BOOL := FALSE;  (* Counter state before this scan     *)
    b_Box_Valid     : BOOL := FALSE;  (* Box edge, gated by RUNNING state   *)
    b_Restart_Blocked: BOOL := FALSE; (* Sort zone occupied -> no restart   *)
    b_Stop_Request  : BOOL;           (* Normalised from the NC input       *)
    b_Tall_Latched  : BOOL := FALSE;  (* Height sampled at detection time   *)

    (* ---------------- Edge detection ---------------- *)
    (* A box blocks the photo-eye for ~400 ms. At a 10 ms scan that is 40
       consecutive TRUE scans. Counting the LEVEL would log 40 boxes. The
       rising edge converts "is blocked" into "has just become blocked",
       which is one pulse per physical box regardless of box length or
       belt speed. *)
    fbBoxEdge       : R_TRIG;
    fbStartEdge     : R_TRIG;
    fbEStopEdge     : F_TRIG;         (* Falling edge: relay went unhealthy *)

    (* ---------------- Timers ---------------- *)
    fbTransitTimer  : TON;            (* Sensor -> pusher flight time       *)
    fbPulseTimer    : TON;            (* Pusher extend duration             *)
    fbDebounceStart : TON;            (* Contact-bounce filter on Start     *)
    fbBeaconFlash   : TON;            (* Fault beacon flash rate            *)

    (* ---------------- Counters ---------------- *)
    fbBatchCounter  : CTU;            (* Accumulates discrete box events    *)
    fbRejectCounter : CTU;            (* Diagnostics: how many diverted     *)

    (* ---------------- Diagnostics ---------------- *)
    nTotalBoxes     : INT := 0;
    nTallBoxes      : INT := 0;
    nBatchesDone    : INT := 0;
    b_BeaconState   : BOOL := FALSE;
END_VAR

VAR CONSTANT
    STATE_FAULT     : INT  := 0;
    STATE_IDLE      : INT  := 1;
    STATE_RUNNING   : INT  := 2;
    STATE_SORTING   : INT  := 3;

    (* Transit time = distance / belt speed = 0.75 m / 0.5 m/s = 1.5 s.
       This is a physical constant of the machine, not a tuning knob: change
       the belt speed or move the pusher and this value must be recomputed. *)
    T_TRANSIT       : TIME := T#1500MS;
    T_PUSH_EXTEND   : TIME := T#400MS;
    T_DEBOUNCE      : TIME := T#500MS;
    T_BEACON_FLASH  : TIME := T#500MS;

    BATCH_SIZE      : INT  := 5;
END_VAR


(* ============================================================================
   RUNG 1 -- INPUT CONDITIONING
   Normalise the NC stop button and debounce the mechanical start button.
   Nothing downstream touches the raw %I tags for these two signals.
   ========================================================================= *)

b_Stop_Request := NOT i_Stop_PB_NC;

fbDebounceStart(IN := i_Start_PB, PT := T_DEBOUNCE);
fbStartEdge(CLK := fbDebounceStart.Q);


(* ============================================================================
   RUNG 2 -- SAFETY MONITOR (highest priority, evaluated before all logic)
   The latch is deliberately sticky. Releasing a twisted E-Stop mushroom must
   NOT restart the conveyor on its own: an operator may have released it from
   outside the cell while a colleague is still inside clearing a jam.
   ========================================================================= *)

fbEStopEdge(CLK := i_EStop_Mon);

IF fbEStopEdge.Q THEN
    b_EStop_Fault := TRUE;          (* relay dropped -> latch the fault *)
END_IF;

IF NOT i_EStop_Mon THEN
    eState := STATE_FAULT;          (* unconditional, pre-empts every state *)
END_IF;


(* ============================================================================
   RUNG 3 -- BOX DETECTION
   One pulse per physical box, and the height classification is sampled on
   that same pulse. Sampling height later would read whatever happens to be
   under the sensor at that moment, which may already be the next box.

   CRITICAL: the edge detector is clocked on the RAW sensor, unconditionally.
   Writing

        fbBoxEdge(CLK := i_Prox_Sensor AND (eState = STATE_RUNNING));

   looks equivalent and is not. Leaving STATE_RUNNING drops CLK low; coming
   back raises it again while the SAME box is still blocking the photo-eye,
   manufacturing a second rising edge from one carton. In simulation that
   turned a single box into five counted boxes in 0.1 s.

   An edge detector must only ever see the physical signal. State gating
   belongs on the CONSUMPTION of the pulse (b_Box_Valid), never on its clock.
   ========================================================================= *)

fbBoxEdge(CLK := i_Prox_Sensor);

b_Box_Valid := fbBoxEdge.Q AND (eState = STATE_RUNNING);

IF b_Box_Valid THEN
    b_Tall_Latched := i_Tall_Sensor;
    nTotalBoxes := nTotalBoxes + 1;
    IF b_Tall_Latched THEN
        nTallBoxes := nTallBoxes + 1;
    END_IF;
END_IF;


(* ============================================================================
   RUNG 4 -- BATCH COUNTER
   CTU, not TON: this accumulates discrete events, it does not measure a
   duration. Driven by the edge pulse so each box increments it exactly once.

   b_Batch_Prev snapshots the counter BEFORE this scan's increment.

   The brief gates the sorting transition on NOT(b_Batch_Complete). Taken
   literally that is an off-by-one: this rung is evaluated earlier in the
   same scan as the box pulse feeding it, so on the 5th box b_Batch_Complete
   is already TRUE by the time RUNG 5 executes, and the final box of every
   batch is counted but never sorted. Verified in simulation: a batch of five
   tall boxes diverted only four and passed one tall carton to discharge.

   Testing the previous scan's value fixes it without changing the brief's
   logical intent -- "do not begin sorting a box once the batch is closed".
   ========================================================================= *)

b_Batch_Prev := fbBatchCounter.Q;

fbBatchCounter(CU := b_Box_Valid,
               RESET := (eState = STATE_IDLE),
               PV := BATCH_SIZE);

b_Batch_Complete := fbBatchCounter.Q;


(* ============================================================================
   RUNG 5 -- STATE MACHINE
   ========================================================================= *)

ePrevState := eState;

CASE eState OF

    (* -------------------------------------------------------------- *)
    STATE_IDLE:
        (* RESTART INTERLOCK -- added, not in the supplied brief.

           Failure found in simulation: when the line halts mid-cycle a box
           can come to rest ON the photo-eye. At restart the sensor is
           ALREADY TRUE, so no rising edge is ever generated and that box
           runs the full length of the line uncounted and unsorted. In the
           E-Stop test five boxes reached the end but only four were
           detected -- had the uncounted one been tall, it would have
           reached discharge undiverted.

           Demanding a clear sensor zone before restart makes that
           impossible rather than merely unlikely. Seeding the edge detector
           from the sensor level instead would risk double-counting a box
           that was already counted before the stop.                      *)
        b_Restart_Blocked := i_Prox_Sensor;

        (* T_1->2 : start pressed, no stop demand, relay healthy,
                    no latched software fault, sort zone clear.           *)
        IF fbStartEdge.Q
           AND NOT b_Stop_Request
           AND i_EStop_Mon
           AND NOT b_EStop_Fault
           AND NOT b_Restart_Blocked
        THEN
            eState := STATE_RUNNING;
        END_IF;

    (* -------------------------------------------------------------- *)
    STATE_RUNNING:
        (* Branch order matters: a detected box is ALWAYS sorted before the
           batch is allowed to close, or the last box of each batch slips
           through unsorted. *)
        IF b_Stop_Request THEN
            eState := STATE_IDLE;

        (* T_2->3 : a box has just been detected and the batch was open
           at the start of this scan. *)
        ELSIF b_Box_Valid AND NOT b_EStop_Fault AND NOT b_Batch_Prev THEN
            eState := STATE_SORTING;

        ELSIF b_Batch_Complete THEN
            nBatchesDone := nBatchesDone + 1;
            eState := STATE_IDLE;
        END_IF;

    (* -------------------------------------------------------------- *)
    STATE_SORTING:
        (* The belt keeps running through this state -- the box has to
           physically travel to the pusher. Sorting is a timing window
           overlaid on motion, not a pause. *)
        IF b_Stop_Request THEN
            eState := STATE_IDLE;
        ELSIF fbTransitTimer.Q AND fbPulseTimer.Q THEN
            eState := STATE_RUNNING;      (* cycle complete, rearm *)
        ELSIF NOT b_Tall_Latched THEN
            eState := STATE_RUNNING;      (* short box, nothing to do *)
        END_IF;

    (* -------------------------------------------------------------- *)
    STATE_FAULT:
        (* Recovery needs BOTH: the physical relay healthy again AND a
           deliberate operator acknowledge. Either alone is not enough. *)
        IF i_EStop_Mon AND fbStartEdge.Q THEN
            b_EStop_Fault := FALSE;
            eState := STATE_IDLE;
        END_IF;

END_CASE;


(* ============================================================================
   RUNG 6 -- TIMERS
   TON measures elapsed physical duration and self-resets when its input goes
   FALSE, which is exactly the behaviour wanted here: leaving STATE_SORTING
   rearms both timers for the next box with no explicit reset.
   ========================================================================= *)

fbTransitTimer(IN := (eState = STATE_SORTING) AND b_Tall_Latched,
               PT := T_TRANSIT);

fbPulseTimer(IN := fbTransitTimer.Q, PT := T_PUSH_EXTEND);

fbRejectCounter(CU := fbTransitTimer.Q,
                RESET := (eState = STATE_IDLE),
                PV := 32767);

fbBeaconFlash(IN := NOT fbBeaconFlash.Q, PT := T_BEACON_FLASH);
IF fbBeaconFlash.Q THEN
    b_BeaconState := NOT b_BeaconState;
END_IF;


(* ============================================================================
   RUNG 7 -- OUTPUTS
   Every physical output is written exactly once, here, at the end of the
   scan. Scattering coil writes through the state machine is what produces
   output chatter: two branches assign the same coil in one scan and the last
   one silently wins.

   Each expression is also gated on the hardware monitor independently, so a
   state-machine bug alone cannot energise a motor while the relay is open.
   ========================================================================= *)

q_Conv_Motor := ((eState = STATE_RUNNING) OR (eState = STATE_SORTING))
                AND i_EStop_Mon
                AND NOT b_EStop_Fault
                AND NOT b_Stop_Request;

(* Pusher fires only after the full transit time, only for a latched tall
   box, and only while the extend pulse is still open. *)
q_Reject_Push := (eState = STATE_SORTING)
                 AND b_Tall_Latched
                 AND fbTransitTimer.Q
                 AND NOT fbPulseTimer.Q
                 AND i_EStop_Mon;

(* Beacon: solid while sorting, flashing on fault, off otherwise. *)
q_Warn_Light := ((eState = STATE_FAULT) AND b_BeaconState)
                OR (eState = STATE_SORTING);

END_PROGRAM
