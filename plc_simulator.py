"""
plc_simulator.py
================
Executable Python model of ConveyorSort.st, plus a physics model of the
conveyor line it controls.

Why a Python twin of the ST program exists:

ST compiles and runs on a PLC, which is the deliverable -- but it cannot be
unit tested in CI, and reviewers cannot run it. This module reproduces the
same logic under the same scan-cycle semantics using the IEC function blocks
in `iec_blocks.py`, so every safety claim in the README is backed by a test
that actually executes. The two files are kept rung-for-rung parallel on
purpose: the comments in each name the rung in the other.

Run:
    python src/plc_simulator.py

Author: Naveed Ali | DecodeLabs Robotics & Automation Internship, Project 4
"""

import os
import sys
from dataclasses import dataclass, field

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from iec_blocks import CTU, F_TRIG, R_TRIG, TON   # noqa: E402

SCAN_MS = 10.0          # PLC cycle time

# State encoding -- mirrors VAR CONSTANT in ConveyorSort.st
STATE_FAULT = 0
STATE_IDLE = 1
STATE_RUNNING = 2
STATE_SORTING = 3

STATE_NAMES = {
    STATE_FAULT: "FAULT",
    STATE_IDLE: "IDLE",
    STATE_RUNNING: "RUNNING",
    STATE_SORTING: "SORTING",
}

# Machine constants
T_TRANSIT = 1500.0       # 0.75 m at 0.5 m/s
T_PUSH_EXTEND = 400.0
T_DEBOUNCE = 500.0
T_BEACON_FLASH = 500.0
BATCH_SIZE = 5


@dataclass
class Inputs:
    """The %I image table, read once at the top of each scan."""
    i_Start_PB: bool = False       # %I0.0  NO
    i_Prox_Sensor: bool = False    # %I0.1
    i_EStop_Mon: bool = True       # %I0.2  TRUE = safety relay healthy
    i_Stop_PB_NC: bool = True      # %I0.3  NC: TRUE = healthy / not pressed
    i_Tall_Sensor: bool = False    # %I0.4


@dataclass
class Outputs:
    """The %Q image table, written once at the end of each scan."""
    q_Conv_Motor: bool = False     # %Q1.0
    q_Reject_Push: bool = False    # %Q1.1
    q_Warn_Light: bool = False     # %Q1.2


class ConveyorSortPLC:
    """
    One-to-one Python execution of ConveyorSort.st.

    `scan()` is one PLC cycle: read inputs, evaluate rungs 1-7 in order,
    write outputs. Nothing in this class blocks, sleeps or loops -- a scan
    must complete in bounded time, which is the whole basis of a PLC's
    real-time guarantee.
    """

    def __init__(self):
        self.state = STATE_IDLE
        self.prev_state = STATE_IDLE

        # Latches
        self.b_EStop_Fault = False
        self.b_Batch_Complete = False
        self.b_Stop_Request = False
        self.b_Tall_Latched = False
        self.b_Restart_Blocked = False

        # Function block instances -- one per use site, never shared
        self.fbBoxEdge = R_TRIG()
        self.fbStartEdge = R_TRIG()
        self.fbEStopEdge = F_TRIG()
        self.fbTransitTimer = TON(T_TRANSIT)
        self.fbPulseTimer = TON(T_PUSH_EXTEND)
        self.fbDebounceStart = TON(T_DEBOUNCE)
        self.fbBeaconFlash = TON(T_BEACON_FLASH)
        self.fbBatchCounter = CTU(BATCH_SIZE)
        self.fbRejectCounter = CTU(32767)

        # Diagnostics
        self.nTotalBoxes = 0
        self.nTallBoxes = 0
        self.nBatchesDone = 0
        self.b_BeaconState = False

        self.outputs = Outputs()
        self.scan_count = 0
        self.transitions = []     # (scan, from, to) for the state trace

    # ------------------------------------------------------------------
    def scan(self, i: Inputs) -> Outputs:
        """Execute one complete PLC scan cycle."""
        self.scan_count += 1

        # === RUNG 1 -- input conditioning ==============================
        # NC stop button normalised: an OPEN circuit (button pressed, or a
        # cut wire) becomes an active stop request.
        self.b_Stop_Request = not i.i_Stop_PB_NC

        debounced_start = self.fbDebounceStart(i.i_Start_PB, dt_ms=SCAN_MS)
        start_pulse = self.fbStartEdge(debounced_start)

        # === RUNG 2 -- safety monitor (highest priority) ===============
        if self.fbEStopEdge(i.i_EStop_Mon):
            self.b_EStop_Fault = True          # latch on the falling edge

        if not i.i_EStop_Mon:
            self.state = STATE_FAULT           # pre-empts everything

        # === RUNG 3 -- box detection ===================================
        # The edge detector runs on the RAW sensor, unconditionally, every
        # scan. Gating CLK on the state (`i_Prox_Sensor AND state=RUNNING`)
        # looks equivalent and is not: leaving RUNNING drops CLK low, and
        # re-entering it raises CLK again while the same box is still sitting
        # on the photo-eye, manufacturing a second rising edge from one box.
        # In testing that produced five counted boxes in 0.1 s from a single
        # carton. An edge detector must only ever see the physical signal.
        raw_box_pulse = self.fbBoxEdge(i.i_Prox_Sensor)
        box_pulse = raw_box_pulse and self.state == STATE_RUNNING
        if box_pulse:
            # Height is sampled on the SAME pulse that detected the box.
            self.b_Tall_Latched = i.i_Tall_Sensor
            self.nTotalBoxes += 1
            if self.b_Tall_Latched:
                self.nTallBoxes += 1

        # === RUNG 4 -- batch counter ===================================
        # Snapshot the batch status BEFORE this scan's increment.
        #
        # The brief gates the sorting transition on NOT(b_Batch_Complete).
        # Taken literally that is an off-by-one: the counter is evaluated
        # earlier in the same scan as the box pulse that feeds it, so on the
        # 5th box b_Batch_Complete is already TRUE by the time the state
        # machine runs, and the final box of every batch is counted but never
        # sorted. In testing, a 5-tall-box batch diverted only 4 and let one
        # tall carton through to discharge.
        #
        # Comparing against the PREVIOUS scan's value is the fix: the 5th box
        # is sorted like any other, and the batch closes once it is clear.
        batch_complete_before = self.fbBatchCounter.Q
        self.b_Batch_Complete = self.fbBatchCounter(
            CU=box_pulse, RESET=(self.state == STATE_IDLE))

        # === RUNG 5 -- state machine ===================================
        self.prev_state = self.state
        s = self.state

        if s == STATE_IDLE:
            # RESTART INTERLOCK (added -- not in the supplied brief).
            #
            # Failure found in testing: when the line halts mid-cycle, a box
            # can come to rest ON the photo-eye. At restart the sensor is
            # ALREADY TRUE, so no rising edge is ever produced and that box
            # travels the whole line uncounted and unsorted. In the E-Stop
            # scenario five boxes reached the end of the line but only four
            # were detected -- if the uncounted one had been tall it would
            # have reached discharge undiverted.
            #
            # Requiring a clear sensor zone before restart makes the failure
            # impossible rather than improbable: the operator must remove the
            # stranded box, which is what they would do anyway. The
            # alternative -- seeding the edge detector from the sensor level
            # at restart -- risks double-counting a box that was already
            # counted before the stop.
            self.b_Restart_Blocked = i.i_Prox_Sensor
            if (start_pulse and not self.b_Stop_Request
                    and i.i_EStop_Mon and not self.b_EStop_Fault
                    and not self.b_Restart_Blocked):
                self.state = STATE_RUNNING

        elif s == STATE_RUNNING:
            # Order matters: a detected box is ALWAYS sorted before the batch
            # is allowed to close, otherwise the last box of each batch slips
            # through unsorted.
            if self.b_Stop_Request:
                self.state = STATE_IDLE
            elif box_pulse and not self.b_EStop_Fault and not batch_complete_before:
                self.state = STATE_SORTING
            elif self.b_Batch_Complete:
                self.nBatchesDone += 1
                self.state = STATE_IDLE

        elif s == STATE_SORTING:
            if self.b_Stop_Request:
                self.state = STATE_IDLE
            elif self.fbTransitTimer.Q and self.fbPulseTimer.Q:
                self.state = STATE_RUNNING
            elif not self.b_Tall_Latched:
                self.state = STATE_RUNNING     # short box, nothing to divert

        elif s == STATE_FAULT:
            # Both conditions required: hardware healthy AND operator ack.
            if i.i_EStop_Mon and start_pulse:
                self.b_EStop_Fault = False
                self.state = STATE_IDLE

        if self.state != self.prev_state:
            self.transitions.append(
                (self.scan_count, STATE_NAMES[self.prev_state],
                 STATE_NAMES[self.state]))

        # === RUNG 6 -- timers ==========================================
        self.fbTransitTimer(
            IN=(self.state == STATE_SORTING) and self.b_Tall_Latched,
            dt_ms=SCAN_MS)
        self.fbPulseTimer(IN=self.fbTransitTimer.Q, dt_ms=SCAN_MS)
        self.fbRejectCounter(CU=self.fbTransitTimer.Q,
                             RESET=(self.state == STATE_IDLE))

        self.fbBeaconFlash(IN=not self.fbBeaconFlash.Q, dt_ms=SCAN_MS)
        if self.fbBeaconFlash.Q:
            self.b_BeaconState = not self.b_BeaconState

        # === RUNG 7 -- outputs, written exactly once ===================
        o = self.outputs
        o.q_Conv_Motor = (
            self.state in (STATE_RUNNING, STATE_SORTING)
            and i.i_EStop_Mon
            and not self.b_EStop_Fault
            and not self.b_Stop_Request
        )
        o.q_Reject_Push = (
            self.state == STATE_SORTING
            and self.b_Tall_Latched
            and self.fbTransitTimer.Q
            and not self.fbPulseTimer.Q
            and i.i_EStop_Mon
        )
        o.q_Warn_Light = (
            (self.state == STATE_FAULT and self.b_BeaconState)
            or self.state == STATE_SORTING
        )
        return o


# ======================================================================
#  Plant model -- the machine the PLC is controlling
# ======================================================================
@dataclass
class Box:
    """A physical carton on the belt."""
    position_m: float
    tall: bool
    length_m: float = 0.30
    diverted: bool = False
    id: int = 0


@dataclass
class ConveyorLine:
    """
    Physical model of the line.

    Layout along the belt:
        0.00 m  infeed
        1.00 m  photo-eye + height sensor (co-located)
        1.75 m  pneumatic pusher        (0.75 m downstream -> 1.5 s at 0.5 m/s)
        3.00 m  discharge
    """

    belt_speed: float = 0.5           # m/s when the motor is energised
    sensor_pos: float = 1.00
    pusher_pos: float = 1.75
    discharge_pos: float = 3.00

    boxes: list = field(default_factory=list)
    accepted: int = 0
    rejected: int = 0
    missed: int = 0                   # tall box that reached discharge undiverted
    manually_cleared: int = 0         # boxes the operator lifted off at restart
    _next_id: int = 1

    def add_box(self, tall: bool) -> None:
        self._next_id += 1
        self.boxes.append(Box(position_m=0.0, tall=tall, id=self._next_id))

    def step(self, motor_on: bool, pusher_on: bool, dt_s: float) -> None:
        """Advance the plant by one scan."""
        if motor_on:
            for b in self.boxes:
                b.position_m += self.belt_speed * dt_s

        # The paddle is 0.4 m wide and strikes the box CENTRE, not its trailing
        # edge: the photo-eye triggers on the leading edge, so centre-vs-edge
        # is a fixed half-box-length offset that has to be accounted for
        # somewhere. Doing it here keeps the 1.5 s transit constant honest.
        if pusher_on:
            for b in self.boxes:
                centre = b.position_m + b.length_m / 2.0
                if not b.diverted and abs(centre - self.pusher_pos) <= 0.20:
                    b.diverted = True
                    self.rejected += 1

        remaining = []
        for b in self.boxes:
            if b.diverted:
                continue                       # left the line down the reject chute
            if b.position_m >= self.discharge_pos:
                self.accepted += 1
                if b.tall:
                    self.missed += 1           # should have been diverted
                continue
            remaining.append(b)
        self.boxes = remaining

    def clear_sensor_zone(self) -> int:
        """
        Operator lifts any box resting on the photo-eye off the line.

        Models the physical action the restart interlock forces: the line
        refuses to start with the sensor blocked, so a stranded box has to be
        removed by hand before production resumes.
        """
        removed = [b for b in self.boxes
                   if not b.diverted
                   and b.position_m <= self.sensor_pos <= b.position_m + b.length_m]
        for b in removed:
            self.boxes.remove(b)
        self.manually_cleared += len(removed)
        return len(removed)

    def sensor_blocked(self) -> bool:
        """Photo-eye: TRUE while any box body spans the sensor position."""
        return any(b.position_m <= self.sensor_pos <= b.position_m + b.length_m
                   for b in self.boxes if not b.diverted)

    def tall_detected(self) -> bool:
        """Height sensor, co-located with the photo-eye."""
        return any(b.tall and b.position_m <= self.sensor_pos <= b.position_m + b.length_m
                   for b in self.boxes if not b.diverted)


# ======================================================================
#  Scenario runner
# ======================================================================
#: Minimum infeed pitch, in scans.
#:
#: The specified FSM handles exactly one box at a time: Running -> Sorting ->
#: Running. A box occupies the Sorting state for the full transit time plus
#: the pusher extend pulse, 1500 + 400 = 1900 ms, and detections that arrive
#: during that window are discarded because the box-edge pulse is only
#: consumed in Running.
#:
#: That is not a bug to patch in software -- it is a throughput limit implied
#: by the control architecture, and it has to be enforced upstream by the
#: infeed. At 0.5 m/s a 1900 ms window is a 0.95 m minimum box pitch; 240
#: scans (2.4 s, 1.2 m) adds ~25% margin.
#:
#: Max sustained throughput = 60 / 2.4 = 25 boxes/min. Exceeding it would
#: require tracking several boxes in flight at once, which means a shift
#: register keyed on belt travel rather than a single-box state machine.
MIN_FEED_PITCH_SCANS = 240


def run_scenario(box_pattern, estop_at_scan=None, estop_clear_at_scan=None,
                 max_scans=8000, feed_interval_scans=MIN_FEED_PITCH_SCANS,
                 verbose=True):
    """
    Drive the PLC against the plant model.

    `box_pattern` is a list of booleans: True = tall box (must be diverted).
    Returns a trace dict suitable for plotting and assertions.
    """
    plc = ConveyorSortPLC()
    line = ConveyorLine()
    inp = Inputs()

    trace = {k: [] for k in ("t", "state", "motor", "pusher", "beacon",
                             "prox", "tall", "estop", "count")}

    pending = list(box_pattern)
    next_feed = 60
    dt_s = SCAN_MS / 1000.0

    # Operator model: presses and HOLDS the start button for ~700 ms whenever
    # the line is stopped but there is still work to do. The hold has to
    # outlast the 500 ms debounce filter, otherwise the press is correctly
    # rejected as contact bounce and the line never starts.
    # The RELEASE gap is not cosmetic. R_TRIG fires on a FALSE->TRUE edge, so
    # a button that is never released produces exactly one pulse no matter how
    # long it is held. An operator model that holds start continuously leaves
    # the line stuck in IDLE after the first press -- which is precisely the
    # behaviour a real start button has, and why machines need a release.
    hold_until = 0
    HOLD_SCANS = 70        # 700 ms press, outlasting the 500 ms debounce
    RELEASE_SCANS = 40     # 400 ms release, guaranteeing a falling edge

    for scan in range(1, max_scans + 1):
        # --- E-Stop stimulus ---
        if estop_clear_at_scan and scan >= estop_clear_at_scan:
            inp.i_EStop_Mon = True          # mushroom released
        elif estop_at_scan and scan >= estop_at_scan:
            inp.i_EStop_Mon = False         # safety relay drops out

        # --- operator presses start / acknowledges a fault ---
        work_remaining = bool(pending) or bool(line.boxes)
        idle_with_work = plc.state == STATE_IDLE and work_remaining
        fault_recoverable = plc.state == STATE_FAULT and inp.i_EStop_Mon
        if scan > hold_until + RELEASE_SCANS and (idle_with_work or fault_recoverable):
            # The restart interlock refuses to start with the photo-eye
            # blocked, so the operator clears the stranded box first.
            if idle_with_work and line.sensor_blocked():
                line.clear_sensor_zone()
            hold_until = scan + HOLD_SCANS
        inp.i_Start_PB = scan <= hold_until

        # Infeed gate. While the line is stopped the gate holds product back
        # and re-arms the pitch timer, so product does not bunch up against a
        # stopped belt and then release in a burst on restart. Without this,
        # two boxes arrive inside one 1.9 s sort window after a fault and the
        # second is silently never detected.
        if plc.state == STATE_FAULT:
            next_feed = max(next_feed, scan + feed_interval_scans)

        if pending and scan >= next_feed and plc.state != STATE_FAULT:
            line.add_box(tall=pending.pop(0))
            next_feed = scan + feed_interval_scans

        inp.i_Prox_Sensor = line.sensor_blocked()
        inp.i_Tall_Sensor = line.tall_detected()

        # --- one PLC scan ---
        out = plc.scan(inp)

        # --- plant responds ---
        line.step(out.q_Conv_Motor, out.q_Reject_Push, dt_s)

        trace["t"].append(scan * SCAN_MS / 1000.0)
        trace["state"].append(plc.state)
        trace["motor"].append(int(out.q_Conv_Motor))
        trace["pusher"].append(int(out.q_Reject_Push))
        trace["beacon"].append(int(out.q_Warn_Light))
        trace["prox"].append(int(inp.i_Prox_Sensor))
        trace["tall"].append(int(inp.i_Tall_Sensor))
        trace["estop"].append(int(not inp.i_EStop_Mon))
        trace["count"].append(plc.fbBatchCounter.CV)

        if not pending and not line.boxes and plc.state == STATE_IDLE and scan > 400:
            break

    result = {
        "trace": trace,
        "plc": plc,
        "line": line,
        "scans": plc.scan_count,
        "boxes_detected": plc.nTotalBoxes,
        "tall_detected": plc.nTallBoxes,
        "accepted": line.accepted,
        "rejected": line.rejected,
        "missed": line.missed,
        "manually_cleared": line.manually_cleared,
        "batches": plc.nBatchesDone,
        "transitions": plc.transitions,
    }

    if verbose:
        print(f"  scans executed      : {result['scans']} "
              f"({result['scans'] * SCAN_MS / 1000:.1f} s of machine time)")
        print(f"  boxes detected      : {result['boxes_detected']} "
              f"(fed {len(box_pattern)})")
        print(f"  tall boxes detected : {result['tall_detected']}")
        print(f"  diverted to reject  : {result['rejected']}")
        print(f"  passed to discharge : {result['accepted']}")
        print(f"  MISSED tall boxes   : {result['missed']}")
        print(f"  cleared by operator : {result['manually_cleared']} "
              f"(restart interlock)")
        print(f"  batches completed   : {result['batches']}")
    return result


def main() -> int:
    out_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "output")
    os.makedirs(out_dir, exist_ok=True)

    print("=" * 68)
    print("  PROJECT 4 -- PLC-BASED CONVEYOR SORTING SYSTEM")
    print("  IEC 61131-3 scan-cycle simulation of ConveyorSort.st")
    print("=" * 68)

    print("\n[1] NORMAL OPERATION -- mixed batch of 5 boxes")
    print("    pattern: short, TALL, short, TALL, short")
    normal = run_scenario([False, True, False, True, False])

    print("\n[2] EMERGENCY STOP mid-cycle, then operator recovery")
    estop = run_scenario([False, True, False, True, False],
                         estop_at_scan=420, estop_clear_at_scan=900)

    print("\n[3] ALL-TALL batch -- every box must be diverted")
    all_tall = run_scenario([True] * 5)

    print("\n  State transition trace (normal run):")
    for scan, frm, to in normal["transitions"][:14]:
        print(f"    t={scan * SCAN_MS / 1000:6.2f}s  {frm:<8} -> {to}")

    try:
        import plc_visualise
        plc_visualise.render_all(normal, estop, all_tall, out_dir)
        print(f"\n  Figures written to output/")
    except Exception as exc:                       # pragma: no cover
        print(f"\n  [warn] figure generation failed: {exc}")

    ok = (normal["missed"] == 0 and normal["rejected"] == 2
          and all_tall["missed"] == 0 and all_tall["rejected"] == 5)
    print(f"\n  Sorting accuracy: "
          f"{'PASS -- no tall box reached discharge' if ok else 'FAIL'}")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
