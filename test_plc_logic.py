"""
test_plc_logic.py
=================
Verification suite for ConveyorSort.

Run:
    python src/test_plc_logic.py          # no pytest needed
    pytest src/test_plc_logic.py -v       # or with pytest if installed

Every claim the README makes about this system's behaviour has a test here.
The suite is organised around the five checks on the project brief's own
"Control Logic Checklist" slide, plus regression tests for each bug found
while building the thing.

Author: Naveed Ali | DecodeLabs Robotics & Automation Internship, Project 4
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from iec_blocks import CTU, R_TRIG, TON                        # noqa: E402
from plc_simulator import (BATCH_SIZE, SCAN_MS, STATE_FAULT,    # noqa: E402
                           STATE_IDLE, STATE_RUNNING, STATE_SORTING,
                           T_TRANSIT, ConveyorSortPLC, Inputs,
                           run_scenario)


# ======================================================================
#  helpers
# ======================================================================
def run_scans(plc, inputs, n):
    """Execute n scans with a fixed input image; return the last outputs."""
    out = None
    for _ in range(n):
        out = plc.scan(inputs)
    return out


def start_the_line(plc, inp=None):
    """Press -> hold past debounce -> release, leaving the PLC in RUNNING."""
    inp = inp or Inputs()
    inp.i_Start_PB = True
    run_scans(plc, inp, 80)          # 800 ms > 500 ms debounce
    inp.i_Start_PB = False
    run_scans(plc, inp, 5)
    return inp


# ======================================================================
#  CHECKLIST 1 -- I/O integrity and debounce
# ======================================================================
def test_debounce_rejects_contact_bounce():
    """A 200 ms press is contact bounce and must NOT start the line."""
    plc = ConveyorSortPLC()
    inp = Inputs()
    inp.i_Start_PB = True
    run_scans(plc, inp, 20)          # 200 ms, under the 500 ms filter
    inp.i_Start_PB = False
    run_scans(plc, inp, 5)
    assert plc.state == STATE_IDLE, "bounce shorter than the filter started the line"


def test_debounce_accepts_genuine_press():
    """A 700 ms press is a real operator action and must start the line."""
    plc = ConveyorSortPLC()
    start_the_line(plc)
    assert plc.state == STATE_RUNNING


def test_nc_stop_button_is_fail_safe():
    """
    A BROKEN WIRE on the NC stop button must stop the line.

    This is the deviation from the brief. With NC wiring a severed conductor
    reads FALSE, identical to the button being pressed, and the line must
    treat that as a stop demand. Had the brief's literal NOT(i_Stop_PB) been
    implemented, a cut wire would instead read as permission to run.
    """
    plc = ConveyorSortPLC()
    inp = start_the_line(plc)
    assert plc.state == STATE_RUNNING

    inp.i_Stop_PB_NC = False         # wire cut / button pressed
    out = run_scans(plc, inp, 3)
    assert plc.state == STATE_IDLE, "open NC stop circuit did not stop the line"
    assert out.q_Conv_Motor is False, "motor still energised with stop circuit open"


# ======================================================================
#  CHECKLIST 2 -- state machine exclusivity
# ======================================================================
def test_states_are_mutually_exclusive():
    """eState is a single INT, so two states can never be active at once."""
    plc = ConveyorSortPLC()
    inp = start_the_line(plc)
    seen = set()
    for _ in range(300):
        plc.scan(inp)
        seen.add(plc.state)
        assert plc.state in (STATE_FAULT, STATE_IDLE, STATE_RUNNING, STATE_SORTING)
    assert len(seen) >= 1


def test_cannot_jump_idle_to_sorting():
    """
    Sorting is only ever reachable through Running.

    The brief calls this out explicitly: the FSM must not short-circuit from
    Idle into Sorting, because Sorting assumes the belt is already moving.
    """
    plc = ConveyorSortPLC()
    inp = Inputs()
    inp.i_Prox_Sensor = True         # box sitting on the eye while idle
    inp.i_Tall_Sensor = True
    run_scans(plc, inp, 100)
    assert plc.state == STATE_IDLE, "entered a motion state without a start command"

    for scan, frm, to in plc.transitions:
        assert not (frm == "IDLE" and to == "SORTING"), \
            f"illegal direct transition IDLE -> SORTING at scan {scan}"


# ======================================================================
#  CHECKLIST 3 -- edge detection
# ======================================================================
def test_r_trig_emits_exactly_one_pulse():
    """R_TRIG fires once on a rising edge and never again while held."""
    trig = R_TRIG()
    pulses = sum(trig(True) for _ in range(40))
    assert pulses == 1, f"expected 1 pulse from a held input, got {pulses}"


def test_one_box_counts_once_not_once_per_scan():
    """
    A box blocking the photo-eye for 400 ms = 40 scans must count ONCE.

    This is the regression test for the brief's headline failure mode: level
    evaluation would log 40 boxes from one carton.
    """
    plc = ConveyorSortPLC()
    inp = start_the_line(plc)

    inp.i_Prox_Sensor = True
    run_scans(plc, inp, 40)          # 400 ms obstruction
    inp.i_Prox_Sensor = False
    run_scans(plc, inp, 5)

    assert plc.nTotalBoxes == 1, \
        f"one physical box counted {plc.nTotalBoxes} times -- edge detection failed"


def test_edge_detector_is_not_gated_on_state():
    """
    Regression: clocking R_TRIG on (sensor AND state=RUNNING) manufactures
    extra edges as the FSM cycles through Sorting while one box is still on
    the eye. One carton must still produce exactly one count.
    """
    plc = ConveyorSortPLC()
    inp = start_the_line(plc)

    inp.i_Prox_Sensor = True
    inp.i_Tall_Sensor = False        # short box -> instant Sorting -> Running
    run_scans(plc, inp, 60)          # FSM cycles repeatedly during this
    inp.i_Prox_Sensor = False
    run_scans(plc, inp, 5)

    assert plc.nTotalBoxes == 1, \
        f"state-gated clock produced {plc.nTotalBoxes} counts from one box"


# ======================================================================
#  CHECKLIST 4 -- time vs event (TON vs CTU)
# ======================================================================
def test_ton_measures_duration_accurately():
    """TON asserts Q at PT, not before."""
    t = TON(T_TRANSIT)
    elapsed = 0.0
    while not t(True, dt_ms=SCAN_MS):
        elapsed += SCAN_MS
        assert elapsed <= T_TRANSIT + SCAN_MS, "TON overran its preset"
    assert abs(elapsed - T_TRANSIT) <= SCAN_MS, \
        f"TON fired at {elapsed} ms, expected {T_TRANSIT} ms"


def test_ton_self_resets_on_falling_input():
    """A TON loses all accumulated time the instant IN drops."""
    t = TON(1000.0)
    for _ in range(50):
        t(True, dt_ms=SCAN_MS)       # 500 ms accumulated
    assert t.ET == 500.0
    t(False, dt_ms=SCAN_MS)
    assert t.ET == 0.0 and t.Q is False, "TON retained time after IN fell"


def test_ctu_requires_explicit_reset():
    """Unlike a TON, a counter holds its value until explicitly reset."""
    c = CTU(5)
    for _ in range(3):
        c(CU=True)
        c(CU=False)
    assert c.CV == 3
    for _ in range(100):
        c(CU=False)                  # idle does nothing to the count
    assert c.CV == 3, "counter decayed without an explicit reset"
    c(CU=False, RESET=True)
    assert c.CV == 0


def test_pusher_fires_at_exactly_the_transit_time():
    """
    The solenoid must fire 1.5 s after detection -- the time for a box to
    travel the 0.75 m from photo-eye to pusher at 0.5 m/s.
    """
    plc = ConveyorSortPLC()
    inp = start_the_line(plc)

    inp.i_Prox_Sensor = True
    inp.i_Tall_Sensor = True
    plc.scan(inp)                    # detection scan
    inp.i_Prox_Sensor = False
    inp.i_Tall_Sensor = False

    fired_at = None
    for n in range(1, 300):
        out = plc.scan(inp)
        if out.q_Reject_Push:
            fired_at = n * SCAN_MS
            break

    assert fired_at is not None, "pusher never fired for a tall box"
    assert abs(fired_at - T_TRANSIT) <= 2 * SCAN_MS, \
        f"pusher fired at {fired_at} ms, expected {T_TRANSIT} ms"


def test_short_box_never_fires_the_pusher():
    """A short box must pass through untouched."""
    plc = ConveyorSortPLC()
    inp = start_the_line(plc)

    inp.i_Prox_Sensor = True
    inp.i_Tall_Sensor = False
    plc.scan(inp)
    inp.i_Prox_Sensor = False

    for _ in range(300):
        out = plc.scan(inp)
        assert out.q_Reject_Push is False, "pusher fired on a short box"


def test_batch_completes_at_exactly_five():
    """CTU closes the batch on the 5th box, not the 4th or 6th."""
    plc = ConveyorSortPLC()
    inp = start_the_line(plc)

    for i in range(BATCH_SIZE):
        inp.i_Prox_Sensor = True
        run_scans(plc, inp, 3)
        inp.i_Prox_Sensor = False
        run_scans(plc, inp, 220)     # let the sort cycle finish
        if i < BATCH_SIZE - 1:
            assert plc.nBatchesDone == 0, f"batch closed early after {i + 1} boxes"

    assert plc.nTotalBoxes == BATCH_SIZE
    assert plc.nBatchesDone == 1, \
        f"expected 1 completed batch, got {plc.nBatchesDone}"


# ======================================================================
#  CHECKLIST 5 -- hardwired safety
# ======================================================================
def test_estop_drops_motor_within_one_scan():
    """
    Losing the safety-relay monitor must de-energise the motor in the very
    next scan -- no ramp, no sequence, no delay.
    """
    plc = ConveyorSortPLC()
    inp = start_the_line(plc)
    inp.i_Prox_Sensor = True
    run_scans(plc, inp, 3)
    inp.i_Prox_Sensor = False
    run_scans(plc, inp, 20)
    assert plc.outputs.q_Conv_Motor is True, "motor was not running before the test"

    inp.i_EStop_Mon = False
    out = plc.scan(inp)              # ONE scan
    assert out.q_Conv_Motor is False, "motor still energised one scan after E-Stop"
    assert out.q_Reject_Push is False, "pusher still energised after E-Stop"
    assert plc.state == STATE_FAULT


def test_estop_fault_latches_and_blocks_silent_restart():
    """
    Releasing the mushroom must NOT restart the machine on its own.

    Somebody may have released it from outside the cell while a colleague is
    still inside. Recovery requires a deliberate operator acknowledge.
    """
    plc = ConveyorSortPLC()
    inp = start_the_line(plc)

    inp.i_EStop_Mon = False
    run_scans(plc, inp, 10)
    assert plc.state == STATE_FAULT
    assert plc.b_EStop_Fault is True

    inp.i_EStop_Mon = True           # mushroom released, NO button press
    out = run_scans(plc, inp, 200)
    assert plc.state == STATE_FAULT, "machine left FAULT without an acknowledge"
    assert out.q_Conv_Motor is False, "motor restarted on its own after E-Stop"
    assert plc.b_EStop_Fault is True


def test_estop_recovery_requires_both_conditions():
    """Acknowledge only works once the relay is healthy again."""
    plc = ConveyorSortPLC()
    inp = start_the_line(plc)

    inp.i_EStop_Mon = False
    run_scans(plc, inp, 10)

    # Acknowledge attempt while the relay is STILL open -> must be refused.
    inp.i_Start_PB = True
    run_scans(plc, inp, 80)
    inp.i_Start_PB = False
    run_scans(plc, inp, 5)
    assert plc.state == STATE_FAULT, "fault cleared while the relay was still open"

    # Relay healthy, then acknowledge -> recovers.
    inp.i_EStop_Mon = True
    inp.i_Start_PB = True
    run_scans(plc, inp, 80)
    inp.i_Start_PB = False
    run_scans(plc, inp, 5)
    assert plc.state == STATE_IDLE, "valid acknowledge did not clear the fault"
    assert plc.b_EStop_Fault is False


def test_outputs_are_independently_gated_on_the_safety_monitor():
    """
    Even with the state machine forced into a motion state, an unhealthy
    relay must keep every output de-energised. Defence in depth: a state
    machine bug alone must not be able to start a motor.
    """
    plc = ConveyorSortPLC()
    inp = Inputs()
    inp.i_EStop_Mon = False
    plc.state = STATE_RUNNING        # forced, as if by a logic fault
    out = plc.scan(inp)
    assert out.q_Conv_Motor is False
    assert out.q_Reject_Push is False


# ======================================================================
#  Restart interlock (added beyond the brief)
# ======================================================================
def test_restart_blocked_while_sensor_obstructed():
    """
    A box stranded on the photo-eye must prevent restart.

    Otherwise the sensor is already TRUE at restart, no rising edge occurs,
    and that box runs the line uncounted and unsorted.
    """
    plc = ConveyorSortPLC()
    inp = Inputs()
    inp.i_Prox_Sensor = True         # stranded box
    inp.i_Start_PB = True
    run_scans(plc, inp, 100)
    assert plc.state == STATE_IDLE, "line restarted with the sort zone occupied"

    inp.i_Start_PB = False
    run_scans(plc, inp, 50)
    inp.i_Prox_Sensor = False        # operator clears the box
    inp.i_Start_PB = True
    run_scans(plc, inp, 80)
    assert plc.state == STATE_RUNNING, "line refused to start with a clear zone"


# ======================================================================
#  End-to-end scenarios
# ======================================================================
def test_no_tall_box_ever_reaches_discharge():
    """The core functional guarantee, across three full scenarios."""
    for name, kwargs in (
        ("mixed batch", dict(box_pattern=[False, True, False, True, False])),
        ("all tall", dict(box_pattern=[True] * 5)),
        ("all short", dict(box_pattern=[False] * 5)),
        ("estop mid-cycle", dict(box_pattern=[False, True, False, True, False],
                                 estop_at_scan=420, estop_clear_at_scan=900)),
    ):
        r = run_scenario(verbose=False, **kwargs)
        assert r["missed"] == 0, \
            f"{name}: {r['missed']} tall box(es) reached discharge undiverted"


def test_every_box_is_accounted_for():
    """Boxes fed == diverted + discharged + cleared by hand + still on belt."""
    for pattern in ([False, True, False, True, False], [True] * 5, [False] * 5):
        r = run_scenario(box_pattern=pattern, verbose=False)
        line = r["line"]
        total = (r["rejected"] + r["accepted"]
                 + r["manually_cleared"] + len(line.boxes))
        assert total == len(pattern), \
            f"{len(pattern)} fed but {total} accounted for"


def test_all_tall_batch_diverts_every_box():
    r = run_scenario(box_pattern=[True] * 5, verbose=False)
    assert r["rejected"] == 5, f"expected 5 diverts, got {r['rejected']}"
    assert r["accepted"] == 0, "a tall box reached discharge"


def test_all_short_batch_diverts_nothing():
    r = run_scenario(box_pattern=[False] * 5, verbose=False)
    assert r["rejected"] == 0, f"pusher fired {r['rejected']} times on short boxes"


# ======================================================================
#  runner
# ======================================================================
def main() -> int:
    tests = [(n, f) for n, f in sorted(globals().items())
             if n.startswith("test_") and callable(f)]

    print("=" * 70)
    print(f"  ConveyorSort verification suite -- {len(tests)} tests")
    print("=" * 70)

    passed, failed = 0, []
    for name, fn in tests:
        try:
            fn()
            print(f"  PASS  {name}")
            passed += 1
        except AssertionError as exc:
            print(f"  FAIL  {name}\n          {exc}")
            failed.append(name)
        except Exception as exc:                  # pragma: no cover
            print(f"  ERROR {name}\n          {type(exc).__name__}: {exc}")
            failed.append(name)

    print("-" * 70)
    print(f"  {passed}/{len(tests)} passed")
    if failed:
        print(f"  failed: {', '.join(failed)}")
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
