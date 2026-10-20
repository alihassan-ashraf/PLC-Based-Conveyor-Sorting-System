"""
iec_blocks.py
=============
Faithful Python implementations of the IEC 61131-3 standard function blocks
used by the conveyor program: TON, TOF, R_TRIG, F_TRIG, CTU, CTD.

These are not loose approximations. Each one reproduces the standard's exact
scan-cycle semantics, because the whole point of the exercise is that PLC
logic behaves differently from ordinary sequential code:

  * A function block instance holds state BETWEEN scans. Two TON instances are
    two independent timers; calling one block from two places in the program
    is a classic bug that makes a timer appear to reset at random.
  * Every block is evaluated exactly once per scan, in program order.
  * Edge detectors compare against the input as it was on the PREVIOUS scan,
    which is what turns a level into a one-shot pulse.

Author: Naveed Ali | DecodeLabs Robotics & Automation Internship, Project 4
"""


class TON:
    """
    On-delay timer.

    Q goes TRUE once IN has been continuously TRUE for PT. ET holds the
    elapsed time. The moment IN goes FALSE, both Q and ET reset -- no explicit
    reset input exists or is needed. That self-reset is precisely why a TON
    cannot be used to count events: any interruption loses the accumulated
    time.
    """

    def __init__(self, preset_ms: float = 0.0):
        self.PT = preset_ms
        self.ET = 0.0
        self.Q = False
        self._prev_in = False

    def __call__(self, IN: bool, PT: float = None, dt_ms: float = 10.0) -> bool:
        if PT is not None:
            self.PT = PT
        if IN:
            if self.ET < self.PT:
                self.ET = min(self.ET + dt_ms, self.PT)
            self.Q = self.ET >= self.PT
        else:
            self.ET = 0.0
            self.Q = False
        self._prev_in = IN
        return self.Q


class TOF:
    """Off-delay timer: Q stays TRUE for PT after IN falls."""

    def __init__(self, preset_ms: float = 0.0):
        self.PT = preset_ms
        self.ET = 0.0
        self.Q = False

    def __call__(self, IN: bool, PT: float = None, dt_ms: float = 10.0) -> bool:
        if PT is not None:
            self.PT = PT
        if IN:
            self.Q = True
            self.ET = 0.0
        elif self.Q:
            self.ET += dt_ms
            if self.ET >= self.PT:
                self.Q = False
        return self.Q


class R_TRIG:
    """
    Rising-edge detector. Q is TRUE for exactly one scan on a FALSE->TRUE
    transition of CLK.

        Q := CLK AND NOT CLK_previous

    This is the single most important block in the program. Without it a box
    that blocks the photo-eye for 400 ms at a 10 ms scan rate increments the
    batch counter 40 times.
    """

    def __init__(self):
        self.Q = False
        self._prev = False

    def __call__(self, CLK: bool) -> bool:
        self.Q = CLK and not self._prev
        self._prev = CLK
        return self.Q


class F_TRIG:
    """Falling-edge detector: one-scan pulse on TRUE->FALSE."""

    def __init__(self):
        self.Q = False
        self._prev = True   # assume healthy at power-up

    def __call__(self, CLK: bool) -> bool:
        self.Q = (not CLK) and self._prev
        self._prev = CLK
        return self.Q


class CTU:
    """
    Up-counter.

    CV increments on each rising edge of CU. Q is TRUE once CV >= PV. Unlike a
    TON, a CTU requires an EXPLICIT reset -- it has no idea what a duration is
    and will happily hold its count across hours of idle time.

    The internal R_TRIG on CU is part of the standard block: feeding it a
    level would otherwise increment once per scan.
    """

    def __init__(self, preset: int = 0):
        self.PV = preset
        self.CV = 0
        self.Q = False
        self._edge = R_TRIG()

    def __call__(self, CU: bool, RESET: bool = False, PV: int = None) -> bool:
        if PV is not None:
            self.PV = PV
        pulse = self._edge(CU)
        if RESET:
            self.CV = 0
        elif pulse and self.CV < 32767:
            self.CV += 1
        self.Q = self.CV >= self.PV
        return self.Q


class CTD:
    """Down-counter, included for completeness of the block library."""

    def __init__(self, preset: int = 0):
        self.PV = preset
        self.CV = preset
        self.Q = False
        self._edge = R_TRIG()

    def __call__(self, CD: bool, LOAD: bool = False, PV: int = None) -> bool:
        if PV is not None:
            self.PV = PV
        pulse = self._edge(CD)
        if LOAD:
            self.CV = self.PV
        elif pulse and self.CV > -32768:
            self.CV -= 1
        self.Q = self.CV <= 0
        return self.Q
