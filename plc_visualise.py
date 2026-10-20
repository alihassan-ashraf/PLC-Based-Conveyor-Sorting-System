"""
plc_visualise.py
================
Timing diagrams and state traces for the conveyor controller.

Produces, in ../output/:
    1_timing_normal.png    I/O timing for a normal mixed batch
    2_timing_estop.png     E-Stop event, latch, and operator recovery
    3_state_trace.png      FSM occupancy over the run

A timing diagram is the native way to read PLC behaviour: it shows the exact
scan on which each coil changed relative to each input, which is what you
actually argue about when commissioning a line.

Author: Naveed Ali | DecodeLabs Robotics & Automation Internship, Project 4
"""

import os

import matplotlib
matplotlib.use("Agg")

import matplotlib.pyplot as plt          # noqa: E402
import numpy as np                        # noqa: E402
from matplotlib.patches import Patch      # noqa: E402

plt.rcParams.update({
    "figure.facecolor": "white",
    "font.size": 9,
    "axes.titlesize": 11,
    "axes.titleweight": "bold",
})

STATE_COLOURS = {
    0: "#c92a2a",   # FAULT
    1: "#868e96",   # IDLE
    2: "#2f9e44",   # RUNNING
    3: "#f08c00",   # SORTING
}
STATE_LABELS = {0: "FAULT", 1: "IDLE", 2: "RUNNING", 3: "SORTING"}


def _digital_plot(ax, t, values, label, colour):
    """One digital signal rendered as a proper square wave."""
    v = np.asarray(values, dtype=float)
    ax.step(t, v, where="post", color=colour, lw=1.6)
    ax.fill_between(t, 0, v, step="post", color=colour, alpha=0.22)
    ax.set_ylim(-0.15, 1.3)
    ax.set_yticks([0, 1])
    ax.set_yticklabels(["0", "1"], fontsize=7)
    ax.set_ylabel(label, rotation=0, ha="right", va="center",
                  fontsize=8.5, labelpad=58, fontfamily="monospace")
    ax.grid(alpha=0.18, axis="x")
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)


def timing_figure(result, title, path, t_max=None):
    tr = result["trace"]
    t = np.asarray(tr["t"])
    mask = slice(None) if t_max is None else slice(0, int(t_max / (t[1] - t[0])))
    t = t[mask]

    signals = [
        ("%I0.1 Prox", tr["prox"], "#1971c2"),
        ("%I0.4 Tall", tr["tall"], "#7048e8"),
        ("E-Stop trip", tr["estop"], "#c92a2a"),
        ("%Q1.0 Motor", tr["motor"], "#2f9e44"),
        ("%Q1.1 Pusher", tr["pusher"], "#f08c00"),
        ("%Q1.2 Beacon", tr["beacon"], "#e8590c"),
    ]

    fig, axes = plt.subplots(len(signals) + 2, 1, figsize=(13, 9),
                             sharex=True,
                             gridspec_kw={"height_ratios":
                                          [1] * len(signals) + [1.1, 1.1]})

    for ax, (label, data, colour) in zip(axes, signals):
        _digital_plot(ax, t, np.asarray(data)[mask], label, colour)

    # --- FSM band ---
    ax = axes[len(signals)]
    states = np.asarray(tr["state"])[mask]
    for s, colour in STATE_COLOURS.items():
        ax.fill_between(t, 0, 1, where=(states == s), step="post",
                        color=colour, alpha=0.85, linewidth=0)
    ax.set_ylim(0, 1)
    ax.set_yticks([])
    ax.set_ylabel("FSM state", rotation=0, ha="right", va="center",
                  fontsize=8.5, labelpad=58, fontfamily="monospace")
    ax.legend(handles=[Patch(facecolor=c, label=STATE_LABELS[s], alpha=0.85)
                       for s, c in STATE_COLOURS.items()],
              loc="upper right", ncol=4, fontsize=7.5, framealpha=0.95)

    # --- batch counter ---
    ax = axes[-1]
    counts = np.asarray(tr["count"])[mask]
    ax.step(t, counts, where="post", color="#0b7285", lw=1.7)
    ax.fill_between(t, 0, counts, step="post", color="#0b7285", alpha=0.18)
    ax.set_ylabel("CTU count\n(batch of 5)", rotation=0, ha="right",
                  va="center", fontsize=8.5, labelpad=58,
                  fontfamily="monospace")
    ax.axhline(5, color="#c92a2a", ls="--", lw=1.0, alpha=0.7)
    ax.set_ylim(-0.3, 6)
    ax.set_xlabel("time (s)")
    ax.grid(alpha=0.18)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)

    fig.suptitle(title, fontsize=13, fontweight="bold")
    fig.tight_layout(rect=[0, 0, 1, 0.97])
    fig.savefig(path, dpi=150)
    plt.close(fig)


def state_trace_figure(normal, estop, all_tall, path):
    fig, axes = plt.subplots(3, 1, figsize=(13, 6.4), sharex=False)

    for ax, result, title in (
        (axes[0], normal, "Normal operation -- mixed batch (short/TALL/short/TALL/short)"),
        (axes[1], estop, "E-Stop trip at t=4.2 s, mushroom released t=9.0 s, "
                         "operator acknowledge required before restart"),
        (axes[2], all_tall, "All-tall batch -- every box diverted"),
    ):
        tr = result["trace"]
        t = np.asarray(tr["t"])
        states = np.asarray(tr["state"])
        for s, colour in STATE_COLOURS.items():
            ax.fill_between(t, 0, 1, where=(states == s), step="post",
                            color=colour, alpha=0.85, linewidth=0)
        pusher = np.asarray(tr["pusher"])
        for idx in np.flatnonzero(np.diff(pusher) > 0):
            ax.axvline(t[idx], color="#000000", lw=1.2, ls=":", alpha=0.75)
        ax.set_ylim(0, 1)
        ax.set_yticks([])
        ax.set_xlim(t[0], t[-1])
        ax.set_title(title, fontsize=9.5)
        ax.set_xlabel("time (s)", fontsize=8)

    axes[0].legend(handles=[Patch(facecolor=c, label=STATE_LABELS[s], alpha=0.85)
                            for s, c in STATE_COLOURS.items()]
                   + [plt.Line2D([], [], color="k", ls=":", label="pusher fires")],
                   loc="upper right", ncol=5, fontsize=7.5, framealpha=0.95)

    fig.suptitle("FSM state occupancy across three commissioning scenarios",
                 fontsize=13, fontweight="bold")
    fig.tight_layout(rect=[0, 0, 1, 0.95])
    fig.savefig(path, dpi=150)
    plt.close(fig)


def render_all(normal, estop, all_tall, out_dir):
    os.makedirs(out_dir, exist_ok=True)
    timing_figure(normal,
                  "Normal operation: one box detected, classified, "
                  "and diverted 1.5 s later",
                  os.path.join(out_dir, "1_timing_normal.png"), t_max=14.0)
    timing_figure(estop,
                  "E-Stop: motor drops within one 10 ms scan, fault latches, "
                  "restart requires operator acknowledge",
                  os.path.join(out_dir, "2_timing_estop.png"), t_max=16.0)
    state_trace_figure(normal, estop, all_tall,
                       os.path.join(out_dir, "3_state_trace.png"))
