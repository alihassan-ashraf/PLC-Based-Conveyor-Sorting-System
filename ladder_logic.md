# Ladder Logic Diagram — Conveyor Sorting System

Ladder equivalent of `src/ConveyorSort.st`. Both represent the same control
program; ST is the maintained source, this is the LD view a controls
technician would read on the HMI or in the panel.

Symbols used throughout:

```
--| |--     normally-open contact      (examine if closed / TRUE)
--|/|--     normally-closed contact    (examine if open   / FALSE)
--( )--     output coil
--(S)--     set / latch coil
--(R)--     reset / unlatch coil
--|P|--     positive (rising) edge, one-shot
--|N|--     negative (falling) edge, one-shot
[TON]       on-delay timer
[CTU]       up-counter
```

---

## I/O at a glance

| Tag | Address | Type | Sense | Device |
|---|---|---|---|---|
| `i_Start_PB` | `%I0.0` | BOOL | NO | Start push button |
| `i_Prox_Sensor` | `%I0.1` | BOOL | NO | Box-present photo-eye |
| `i_EStop_Mon` | `%I0.2` | BOOL | **NC, TRUE = healthy** | Safety relay monitor contact |
| `i_Stop_PB_NC` | `%I0.3` | BOOL | **NC, TRUE = healthy** | Stop push button |
| `i_Tall_Sensor` | `%I0.4` | BOOL | NO | High-level height sensor |
| `q_Conv_Motor` | `%Q1.0` | BOOL | — | Belt drive contactor |
| `q_Reject_Push` | `%Q1.1` | BOOL | — | Diverter solenoid valve |
| `q_Warn_Light` | `%Q1.2` | BOOL | — | Amber warning beacon |

---

## RUNG 1 — Input conditioning

The NC stop button is normalised into a positive-logic request flag, and the
mechanical start button is filtered for contact bounce.

```
    i_Stop_PB_NC                                      b_Stop_Request
 ------|/|---------------------------------------------( )--------
       %I0.3        (open circuit = stop demanded)

    i_Start_PB            [ TON  fbDebounceStart ]    b_Start_Clean
 ------| |-------------------[ PT := T#500MS   ]--------( )--------
       %I0.0                 [ IN      Q       ]

    b_Start_Clean                                     b_Start_Pulse
 ------|P|---------------------------------------------( )--------
```

**Why `--|/|--` on the stop button.** On an NC contact the circuit is closed at
rest, so `%I0.3` reads **TRUE when the machine is healthy**. A pressed button
*or a severed wire* both read FALSE. Taking the complement turns "circuit
open" into "stop demanded", which means a broken wire fails toward a stop.

The project brief writes this transition as `NOT(i_Stop_PB)` while also
specifying the button as NC — those contradict each other, and taken literally
the line would only run while somebody held the stop button down. See
[`README.md`](../README.md#deviations-from-the-supplied-brief).

**Why the 500 ms debounce.** A mechanical pushbutton's contacts bounce for
5–20 ms. At a 10 ms scan the PLC samples straight through that bounce and sees
several presses. The TON requires the signal to be *continuously* TRUE for
500 ms before passing it on.

---

## RUNG 2 — Safety monitor (evaluated before all process logic)

```
    i_EStop_Mon                                       b_EStop_Fault
 ------|N|---------------------------------------------(S)--------
       %I0.2    (relay dropped out -> latch the fault)

    i_EStop_Mon                                       eState := FAULT
 ------|/|---------------------------------------------[MOV 0]----
       %I0.2    (unconditional, pre-empts every state)
```

**This rung does not stop the machine.** The E-Stop button breaks the coil
circuit of a hardware safety relay, which drops motor power directly —
Stop Category 0 per IEC 60204-1. `%I0.2` is a *monitoring* contact telling the
PLC what the hardware has already done.

The latch is deliberately sticky: a released mushroom head must not restart
the conveyor on its own, because it may have been released from outside the
cell while somebody is still inside clearing a jam.

---

## RUNG 3 — Box detection and height latch

```
    i_Prox_Sensor                                     b_Box_Edge
 ------| |-------|P|------------------------------------( )--------
       %I0.1

    b_Box_Edge    eState = RUNNING                    b_Box_Valid
 ------| |-----------| |-------------------------------( )--------

    b_Box_Valid   i_Tall_Sensor                       b_Tall_Latched
 ------| |-----------| |-------------------------------(S)--------
                     %I0.4

    b_Box_Valid   i_Tall_Sensor                       b_Tall_Latched
 ------| |-----------|/|-------------------------------(R)--------
                     %I0.4
```

**The one-shot is the whole rung.** A 0.30 m box at 0.5 m/s blocks the
photo-eye for ~600 ms. At a 10 ms scan that is 60 consecutive TRUE scans.
Evaluating the *level* would log 60 boxes from one carton. `--|P|--` converts
"is blocked" into "has just become blocked" — one pulse per physical box,
regardless of box length or belt speed.

**The edge detector is clocked on the raw sensor, never on `sensor AND
state`.** Gating the clock looks equivalent and is not: leaving `RUNNING`
drops the clock low, and re-entering raises it again while the same box is
still on the eye, manufacturing a second edge from one carton. State gating
belongs on `b_Box_Valid` — the *consumption* of the pulse — not on its clock.
This was a real bug found in testing; see
[`test_edge_detector_is_not_gated_on_state`](../src/test_plc_logic.py).

**Height is sampled on the same pulse that detects the box.** Reading it later
returns whatever happens to be under the sensor at that moment, which may
already be the next box.

---

## RUNG 4 — Batch counter

```
                                                      b_Batch_Prev
 ------| b_Batch_Complete |----------------------------( )--------
          (snapshot taken BEFORE this scan's count)

    b_Box_Valid          [ CTU  fbBatchCounter ]      b_Batch_Complete
 ------| |-------------------[ CU        Q     ]--------( )--------
                             [ PV := 5         ]
    eState = IDLE            [ RESET           ]
 ------| |-------------------[                 ]
```

**CTU, not TON.** A counter accumulates discrete *events*; a timer measures
*duration*. They are not interchangeable: a TON self-resets the instant its
input drops, so it can never count boxes, and a CTU has no concept of elapsed
time and holds its value indefinitely until explicitly reset.

**`b_Batch_Prev` exists to fix an off-by-one in the brief.** The brief gates
the sorting transition on `NOT(b_Batch_Complete)`. This rung is evaluated
earlier in the same scan as the box pulse feeding it, so on the 5th box
`b_Batch_Complete` is already TRUE when the state machine runs — and the last
box of every batch is counted but never sorted. Verified: a batch of five tall
boxes diverted only four and passed one tall carton to discharge. Testing the
*previous* scan's value fixes it without changing the brief's intent.

---

## RUNG 5 — State machine

States are encoded in a single `INT`, so they are **mutually exclusive by
construction**. Independent BOOLs for Idle/Running/Sorting/Fault would permit
two states to drive the same coil in one scan — the exact output race this
structure exists to prevent.

```
 IDLE (1) -> RUNNING (2)

   b_Start_Pulse  b_Stop_Request  i_EStop_Mon  b_EStop_Fault  i_Prox_Sensor
 ------| |------------|/|------------| |-----------|/|------------|/|-----[MOV 2]--
                                     %I0.2                       %I0.1
                                                            (restart interlock)

 RUNNING (2) -> SORTING (3)

   b_Box_Valid    b_EStop_Fault   b_Batch_Prev
 ------| |------------|/|-------------|/|-----------------------------[MOV 3]--

 SORTING (3) -> RUNNING (2)      (cycle complete, or nothing to divert)

   fbTransitTimer.Q   fbPulseTimer.Q
 ---------| |--------------| |----------------------------------------[MOV 2]--
                +
   b_Tall_Latched
 ---------|/|---------------------------------------------------------[MOV 2]--

 RUNNING (2) -> IDLE (1)         (stop demanded, or batch closed)

   b_Stop_Request
 ---------| |---------------------------------------------------------[MOV 1]--
                +
   b_Batch_Complete
 ---------| |---------------------------------------------------------[MOV 1]--

 FAULT (0) -> IDLE (1)           (BOTH conditions required)

   i_EStop_Mon      b_Start_Pulse
 ---------| |------------| |-----------------------------[MOV 1]--+--(R) b_EStop_Fault
          %I0.2
```

**Branch order inside RUNNING matters.** A detected box is always routed to
`SORTING` *before* the batch is allowed to close, or the last box of each
batch slips through unsorted.

**Idle cannot reach Sorting directly.** Sorting assumes the belt is already
moving and that a transit time is meaningful; entering it from a standstill
would fire the pusher at a box that never arrives. The brief calls this out
explicitly, and
[`test_cannot_jump_idle_to_sorting`](../src/test_plc_logic.py) enforces it.

**The restart interlock (`--|/|-- %I0.1`) is an addition, not in the brief.**
When the line halts mid-cycle a box can come to rest *on* the photo-eye. At
restart the sensor is already TRUE, no rising edge is ever produced, and that
box runs the full line uncounted and unsorted. In the E-Stop scenario five
boxes reached the end but only four were detected — had the uncounted one been
tall, it would have reached discharge undiverted. Requiring a clear sensor
zone makes the failure impossible rather than merely unlikely.

---

## RUNG 6 — Timers

```
   eState = SORTING   b_Tall_Latched    [ TON fbTransitTimer ]
 ---------| |--------------| |--------------[ PT := T#1500MS ]--
                                            [ IN         Q   ]

   fbTransitTimer.Q                       [ TON fbPulseTimer ]
 ---------| |-------------------------------[ PT := T#400MS  ]--
                                            [ IN         Q   ]
```

**Where 1500 ms comes from:**

```
        distance from photo-eye to pusher     0.75 m
   t =  --------------------------------  =  --------  =  1.5 s
                   belt speed                0.5 m/s
```

This is a **physical constant of the machine, not a tuning knob**. Change the
belt speed or move the pusher and it must be recomputed, or the paddle swings
through empty air behind a box that has already passed.

A TON self-resets when its input falls, so leaving `SORTING` rearms both
timers for the next box with no explicit reset — exactly the behaviour wanted
here, and exactly why a TON could never serve as the batch counter.

---

## RUNG 7 — Outputs

Every physical output is written **exactly once**, here, at the end of the
scan. Scattering coil writes through the state machine is what produces output
chatter: two branches assign the same coil in one scan and the last one
silently wins.

```
   eState=RUNNING    i_EStop_Mon   b_EStop_Fault  b_Stop_Request   q_Conv_Motor
 --------| |-------+-----| |-----------|/|-------------|/|-----------( )--------
   eState=SORTING  |     %I0.2                                       %Q1.0
 --------| |-------+

   eState=SORTING  b_Tall_Latched  Transit.Q   Pulse.Q   i_EStop_Mon  q_Reject_Push
 --------| |-----------| |-----------| |---------|/|---------| |---------( )--------
                                                                        %Q1.1

   eState=FAULT    b_BeaconState                                     q_Warn_Light
 --------| |-----------| |-------------------------------------------+--( )--------
   eState=SORTING                                                    |  %Q1.2
 --------| |---------------------------------------------------------+
```

**Each output is independently gated on `i_EStop_Mon`.** This is deliberate
redundancy: the state machine already refuses to leave `FAULT` while the relay
is open, so in correct operation these contacts never do anything. They exist
so that a *state machine bug alone* cannot energise a motor while the safety
relay is dropped out. Defence in depth —
[`test_outputs_are_independently_gated_on_the_safety_monitor`](../src/test_plc_logic.py)
forces the FSM into `RUNNING` with the relay open and asserts every output
stays dark.

**`--|/|-- Pulse.Q` on the pusher** makes the solenoid a 400 ms *pulse*, not a
level. A pneumatic valve held energised indefinitely overheats the coil and
leaves the rod extended into the path of the next box.

---

## Scan-cycle execution order

```
   ┌─────────────────────────────────────────────────┐
   │  1. Read all physical inputs into the %I image   │
   │  2. Evaluate rungs 1 → 7, strictly top to bottom │
   │  3. Write the %Q image to physical outputs       │
   │  4. Housekeeping / comms                         │
   └──────────────────────┬──────────────────────────┘
                          └──> repeat every 10 ms
```

The input image is frozen for the whole scan: a sensor that changes state
mid-scan is not seen until the next cycle. This is what makes a PLC
deterministic, and it is why the rung *order* above is part of the design
rather than an accident of layout — the safety monitor is rung 2 because
everything downstream must act on an already-updated fault state.
