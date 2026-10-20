# Project 4 — PLC-Based Conveyor Sorting System

**DecodeLabs Robotics & Automation Internship, Batch 2026**
Industrial control logic that sorts boxes by height on an automated conveyor
line, with hardwired-grade safety interlocks.

---

## What this is

A complete IEC 61131-3 control program for a conveyor sorting cell, plus an
executable scan-cycle model of it and a 23-test verification suite.

| Deliverable | File |
|---|---|
| **PLC program** (Structured Text) | [`src/ConveyorSort.st`](src/ConveyorSort.st) |
| **Ladder logic diagram** + rung-by-rung rationale | [`docs/ladder_logic.md`](docs/ladder_logic.md) |
| **I/O map** | [`docs/ladder_logic.md`](docs/ladder_logic.md#io-at-a-glance) |
| IEC function block library (TON/TOF/CTU/CTD/R_TRIG/F_TRIG) | [`src/iec_blocks.py`](src/iec_blocks.py) |
| Executable scan-cycle simulator + plant model | [`src/plc_simulator.py`](src/plc_simulator.py) |
| Verification suite — **23/23 passing** | [`src/test_plc_logic.py`](src/test_plc_logic.py) |

`ConveyorSort.st` is the deliverable that runs on a PLC. `plc_simulator.py` is
a rung-for-rung Python twin of it, because ST cannot be unit tested in CI and
reviewers cannot run it. Every safety claim below is backed by a test that
actually executes.

---

## Run it

```bash
pip install -r ../requirements.txt     # matplotlib + numpy, for the figures
python src/plc_simulator.py            # three commissioning scenarios + figures
python src/test_plc_logic.py           # verification suite
```

No PLC, no CODESYS licence, no hardware required.

---

## The machine

```
   0.00 m          1.00 m                    1.75 m              3.00 m
     |               |                          |                   |
  infeed        photo-eye %I0.1             pneumatic           discharge
                height   %I0.4               pusher %Q1.1
                   |<------- 0.75 m ------->|
                        1.5 s at 0.5 m/s
```

Boxes enter at the infeed. The photo-eye detects each one and the co-located
height sensor classifies it. A **tall** box is diverted by the pusher exactly
1.5 s later — the time it takes to travel the 0.75 m from sensor to paddle.
Five boxes complete a batch.

---

## Results

```
[1] NORMAL OPERATION -- mixed batch (short, TALL, short, TALL, short)
      boxes detected      : 5 of 5
      diverted to reject  : 2          <- both tall boxes
      passed to discharge : 2
      cleared by operator : 1          <- restart interlock, see below
      MISSED tall boxes   : 0
      batches completed   : 1

[2] EMERGENCY STOP mid-cycle, then operator recovery
      boxes detected      : 5 of 5
      diverted to reject  : 2
      MISSED tall boxes   : 0
      motor de-energised  : within 1 scan (10 ms) of the relay dropping

[3] ALL-TALL batch -- every box must be diverted
      boxes detected      : 5 of 5
      diverted to reject  : 5
      passed to discharge : 0
      MISSED tall boxes   : 0
```

**No tall box reaches discharge in any scenario**, including with an E-Stop
fired mid-cycle. That is the system's one functional guarantee and it is
asserted across four scenarios in
[`test_no_tall_box_ever_reaches_discharge`](src/test_plc_logic.py).

### Normal operation

![Timing diagram, normal operation](output/1_timing_normal.png)

Read it left to right: `%I0.1` pulses as each box crosses the photo-eye;
`%I0.4` is high only for boxes 2 and 4; the pusher fires 1.5 s after each of
those (4.4 s → 5.9 s, 9.2 s → 10.7 s) and stays dark for the short ones. The
CTU steps once per box — not once per scan — and the FSM band shows
`SORTING` entered only from `RUNNING`.

### Emergency stop

![Timing diagram, E-Stop](output/2_timing_estop.png)

The E-Stop trips at 4.2 s and `%Q1.0` drops in the same scan. The FSM latches
into `FAULT` and the beacon flashes. **The mushroom is released at 9.0 s and
nothing happens** — the machine sits in `FAULT` until the operator
acknowledges at 9.5 s. The batch counter resets on return to `IDLE`: a batch
interrupted by an E-Stop is abandoned, not silently resumed.

### All three scenarios

![FSM state trace](output/3_state_trace.png)

---

## The five checklist items

The brief's own *Control Logic Checklist* slide, and where each is satisfied:

### 1. I/O integrity — correct `%I`/`%Q` tags, 500 ms debounce

A mechanical pushbutton's contacts bounce for 5–20 ms. At a 10 ms scan the PLC
samples straight through that and sees several presses. A TON requires the
signal to be *continuously* TRUE for 500 ms before it counts.

> `test_debounce_rejects_contact_bounce` — a 200 ms press does **not** start the line
> `test_debounce_accepts_genuine_press` — a 700 ms press does

### 2. State machine exclusivity

States are a single `INT`, so two cannot be active at once by construction.
Independent BOOLs would allow two states to drive the same coil in one scan —
the exact output race this prevents. `SORTING` is unreachable from `IDLE`.

> `test_states_are_mutually_exclusive`
> `test_cannot_jump_idle_to_sorting`

### 3. Edge detection — `R_TRIG`, one count per box

A 0.30 m box at 0.5 m/s blocks the photo-eye for ~600 ms = 60 scans. Level
evaluation would log 60 boxes from one carton.

> `test_r_trig_emits_exactly_one_pulse`
> `test_one_box_counts_once_not_once_per_scan` — 40 scans of obstruction → count of 1

### 4. Time vs event — TON for duration, CTU for accumulation

They are not interchangeable. A TON self-resets the instant its input drops,
so it can never count boxes. A CTU has no notion of elapsed time and holds its
value until explicitly reset.

> `test_ton_measures_duration_accurately` — fires at 1500 ms ±1 scan
> `test_ton_self_resets_on_falling_input`
> `test_ctu_requires_explicit_reset`
> `test_pusher_fires_at_exactly_the_transit_time`

### 5. Hardwired safety

**The PLC does not stop this machine and must not be relied on to.** The
E-Stop breaks the coil circuit of a hardware safety relay, dropping motor
power directly (Stop Category 0, IEC 60204-1). `%I0.2` is a *monitoring*
contact telling the PLC what the hardware already did. Software handles
diagnostics, latching and orderly restart — never the stop itself.

> `test_estop_drops_motor_within_one_scan`
> `test_estop_fault_latches_and_blocks_silent_restart` — releasing the mushroom alone does **not** restart the line
> `test_estop_recovery_requires_both_conditions`
> `test_outputs_are_independently_gated_on_the_safety_monitor`

---

## Deviations from the supplied brief

Three places where implementing the brief literally produced a defect. Each is
documented at the point of change in both the ST and the Python.

### 1. The NC stop button was wired to fail *unsafe*

The brief specifies `i_Stop_PB` as **normally-closed** but writes the start
transition as:

```
T_1->2 = i_Start_PB AND NOT(i_Stop_PB) AND i_EStop_Mon AND NOT(b_EStop_Fault)
```

Those contradict each other. On an NC contact the circuit is closed at rest,
so the input reads TRUE when the machine is **healthy**. Taking `NOT()` of
that means:

- the conveyor can only start while somebody holds the stop button down, and
- **a severed stop wire reads as permission to run.**

The NC wiring is the safety-relevant part and is non-negotiable, so the fix is
to normalise the signal instead:

```pascal
b_Stop_Request := NOT i_Stop_PB_NC;   (* open circuit = stop demanded *)
```

The transition then reads `NOT(b_Stop_Request)`, preserving the brief's
logical shape while making a broken wire fail toward a stop.

> `test_nc_stop_button_is_fail_safe`

### 2. `NOT(b_Batch_Complete)` lost the last box of every batch

The counter rung is evaluated earlier in the same scan as the box pulse that
feeds it. On the 5th box `b_Batch_Complete` is already TRUE by the time the
state machine runs, so the sorting transition is blocked and the final box of
each batch is counted but never sorted.

Observed before the fix: **a batch of five tall boxes diverted only four and
passed one tall carton to discharge.** Comparing against the previous scan's
value fixes it without changing the brief's intent.

### 3. Gating the edge detector's clock on the state manufactured phantom boxes

Writing `fbBoxEdge(CLK := i_Prox_Sensor AND (eState = STATE_RUNNING))` looks
equivalent to gating the pulse and is not. Leaving `RUNNING` drops the clock
low; re-entering raises it again while the *same* box is still on the eye,
creating a second rising edge from one carton.

Observed before the fix: **one box counted five times in 0.1 s.** An edge
detector must only ever see the physical signal; state gating belongs on the
consumption of the pulse.

> `test_edge_detector_is_not_gated_on_state`

---

## Additions beyond the brief

### Restart interlock

When the line halts mid-cycle a box can come to rest *on* the photo-eye. At
restart the sensor is already TRUE, so no rising edge is ever produced and
that box runs the full length of the line uncounted and unsorted.

In the E-Stop scenario, five boxes reached the end of the line but only four
were detected. The uncounted one happened to be short — had it been tall it
would have reached discharge undiverted.

The line now refuses to start with the sensor blocked, forcing the operator to
remove the stranded box. The alternative — seeding the edge detector from the
sensor level at restart — risks double-counting a box that was already counted
before the stop.

> `test_restart_blocked_while_sensor_obstructed`

### Throughput limit, stated explicitly

The specified FSM handles exactly one box at a time, and a box occupies
`SORTING` for the full transit plus pusher pulse: **1500 + 400 = 1900 ms**.
Detections arriving inside that window are discarded.

This is not a bug to patch in software — it is a limit implied by the control
architecture, and it has to be enforced upstream by the infeed:

```
   minimum box pitch = 1.9 s × 0.5 m/s = 0.95 m
   max throughput    = 60 / 1.9        ≈ 31 boxes/min  (theoretical)
                       60 / 2.4        = 25 boxes/min  (with 25% margin)
```

Exceeding it needs several boxes tracked in flight simultaneously, which means
a shift register keyed on belt travel rather than a single-box state machine.
The infeed gate also re-establishes pitch after a stop, so product does not
bunch against a stopped belt and release in a burst.

---

## Verification

```
======================================================================
  ConveyorSort verification suite -- 23 tests
======================================================================
  PASS  test_all_short_batch_diverts_nothing
  PASS  test_all_tall_batch_diverts_every_box
  PASS  test_batch_completes_at_exactly_five
  PASS  test_cannot_jump_idle_to_sorting
  PASS  test_ctu_requires_explicit_reset
  PASS  test_debounce_accepts_genuine_press
  PASS  test_debounce_rejects_contact_bounce
  PASS  test_edge_detector_is_not_gated_on_state
  PASS  test_estop_drops_motor_within_one_scan
  PASS  test_estop_fault_latches_and_blocks_silent_restart
  PASS  test_estop_recovery_requires_both_conditions
  PASS  test_every_box_is_accounted_for
  PASS  test_nc_stop_button_is_fail_safe
  PASS  test_no_tall_box_ever_reaches_discharge
  PASS  test_one_box_counts_once_not_once_per_scan
  PASS  test_outputs_are_independently_gated_on_the_safety_monitor
  PASS  test_pusher_fires_at_exactly_the_transit_time
  PASS  test_r_trig_emits_exactly_one_pulse
  PASS  test_restart_blocked_while_sensor_obstructed
  PASS  test_short_box_never_fires_the_pusher
  PASS  test_states_are_mutually_exclusive
  PASS  test_ton_measures_duration_accurately
  PASS  test_ton_self_resets_on_falling_input
----------------------------------------------------------------------
  23/23 passed
```

---

## Deploying to real hardware

`ConveyorSort.st` targets Siemens S7-1200/1500 (SCL), CODESYS and TwinCAT 3.
To commission:

1. Import the ST into the IDE as a `PROGRAM` block and assign it to a cyclic
   task (OB1 for Siemens, 10 ms cycle).
2. Bind the `AT %I…` / `AT %Q…` addresses to the actual rack layout.
3. **Wire the safety relay first.** The E-Stop must break motor power through
   hardware. Verify by pulling the PLC into STOP mode and confirming the motor
   still drops when the mushroom is hit.
4. Re-measure the sensor-to-pusher distance and belt speed on the real line
   and recompute `T_TRANSIT` — 1500 ms is correct for 0.75 m at 0.5 m/s and
   nothing else.
5. Dry-run the sequence with the pneumatics isolated before admitting product.

The virtual commissioning workflow in the brief (S7-PLCSIM + a 3D physics
emulator) is what `plc_simulator.py` substitutes for here: same purpose —
validate FSM transitions, edge-detection fidelity and safety interlocks before
anything moves.

---

*Naveed Ali — DecodeLabs Robotics & Automation Internship, Batch 2026*
