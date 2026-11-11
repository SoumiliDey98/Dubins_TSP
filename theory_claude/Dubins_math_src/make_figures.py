"""Figures of Dubins_math.tex, drawn with dubins_path.py itself.

Run from the repository root:
    python theory_claude/Dubins_math_src/make_figures.py
It writes fig_csc.pdf, fig_ccc.pdf and fig_six_types.pdf next to this file and prints
the lengths quoted in the captions. Needs LaTeX and cm-super (text.usetex).
"""
import math
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "..", "Dubins_TSP_New_Implementation_claude"))
import dubins_path as dp  # noqa: E402

plt.rcParams.update({
    "text.usetex": True,
    "font.family": "serif",
    "font.size": 9,
    "lines.linewidth": 1.0,
    "pdf.fonttype": 42,
})

R = 5.0
PATH = "#1f4e9c"        # the path
CON = "0.45"            # circles, centres and construction lines


def unit(a):
    return np.array([math.cos(a), math.sin(a)])


def arrow(ax, p, psi, length=3.0, color="black"):
    ax.annotate("", xy=tuple(np.add(p, length * unit(psi))), xytext=tuple(p),
                arrowprops=dict(arrowstyle="-|>", color=color, lw=0.9, mutation_scale=8, shrinkA=0, shrinkB=0))


def circle(ax, c):
    t = np.linspace(0, 2 * math.pi, 300)
    ax.plot(c[0] + R * np.cos(t), c[1] + R * np.sin(t), color=CON, lw=0.6, ls=(0, (4, 3)))
    ax.plot(*c, "o", ms=2.5, color=CON)


def segment(ax, a, b, ls="-"):
    ax.plot([a[0], b[0]], [a[1], b[1]], color=CON, lw=0.6, ls=ls)


def point(ax, p):
    ax.plot(*p, "o", ms=3, color="black", zorder=5)


def text(ax, p, s, offset=(0, 0), color="black", **kw):
    kw.setdefault("ha", "center")
    kw.setdefault("va", "center")
    ax.text(p[0] + offset[0], p[1] + offset[1], s, color=color, **kw)


def angle_arc(ax, c, a0, a1, r):
    t = np.linspace(a0, a1, 60)
    ax.plot(c[0] + r * np.cos(t), c[1] + r * np.sin(t), color="black", lw=0.6)


def arc_label(ax, c, a, r, s, color=PATH):
    """Label s at angle a seen from centre c, at distance r."""
    text(ax, np.add(c, r * unit(a)), s, color=color)


def finish(ax):
    ax.set_aspect("equal")
    ax.axis("off")


def csc_figure():
    q1 = (0.0, 0.0, math.radians(-20))
    q2 = (24.0, 11.0, math.radians(-70))
    t = "LSR"
    S1, d, S2 = (float(v) for v in dp.calculateCSCLengths(*q1, *q2, R, t))
    xs, ys, _, _ = dp.calculateCSCTrajectory(*q1, R, t, S1, d, S2, 0.05)
    c1 = np.array(dp.returnCircleCenter(*q1, 1, R))
    c2 = np.array(dp.returnCircleCenter(*q2, -1, R))
    psiL = math.atan2(*(c2 - c1)[::-1])
    Stan = float(np.hypot(*(c2 - c1)))
    psiD = q1[2] + S1 / R
    co = np.array(dp.pointsOnArc(*q1, 1, R, S1))
    ci = co + d * unit(psiD)
    p1, p2 = np.array(q1[:2]), np.array(q2[:2])
    mid = 0.5 * (c1 + c2)

    fig, ax = plt.subplots(figsize=(4.6, 2.7))
    circle(ax, c1)
    circle(ax, c2)
    segment(ax, c1, c2, ls=":")
    segment(ax, c1, co)
    segment(ax, c2, ci)
    ax.plot(xs, ys, color=PATH, lw=1.5)
    arrow(ax, p1, q1[2])
    arrow(ax, p2, q2[2])
    for p in (p1, p2, co, ci):
        point(ax, p)

    text(ax, p1, r"$(x_1,y_1,\psi_1)$", (-0.6, -1.4), ha="right")
    text(ax, p2, r"$(x_2,y_2,\psi_2)$", (0.8, 0.9), ha="left")
    text(ax, c1, r"$(x_{c1},y_{c1})$", (-1.1, 0.3), ha="right", color=CON)
    text(ax, c2, r"$(x_{c2},y_{c2})$", (0.6, -1.0), ha="left", color=CON)
    text(ax, co, r"$(x_{co},y_{co})$", (0.6, -1.0), ha="left")
    text(ax, ci, r"$(x_{ci},y_{ci})$", (-0.6, 1.0), ha="right")
    text(ax, 0.5 * (c1 + co), r"$R$", (0.7, 0.4), color=CON)
    text(ax, 0.6 * c1 + 0.4 * c2, r"$S_{\tan}$", (-0.5, 0.9), color=CON)
    text(ax, 0.3 * co + 0.7 * ci, r"$d_{co,ci}$", (-0.9, 1.0), color=PATH)
    a1 = q1[2] - math.pi / 2 + 0.5 * S1 / R          # middle of the first arc, seen from c1
    arc_label(ax, c1, a1, R - 1.3, r"$S_1$")
    a2 = psiD + math.pi / 2 - 0.5 * S2 / R           # middle of the last arc, seen from c2
    arc_label(ax, c2, a2, R + 1.3, r"$S_2$")

    # psi_L at the first centre, from the x direction
    segment(ax, c1, c1 + np.array([4.5, 0.0]), ls=(0, (1, 1.5)))
    angle_arc(ax, c1, 0.0, psiL, 3.0)
    text(ax, c1 + 3.0 * unit(psiL / 2), r"$\psi_L$", (1.4, 0.1))
    # heading of the straight part
    segment(ax, co, co + np.array([5.0, 0.0]), ls=(0, (1, 1.5)))
    angle_arc(ax, co, 0.0, psiD, 3.2)
    text(ax, co + 3.2 * unit(psiD / 2), r"$\psi_D$", (1.0, 0.0), color=PATH)
    # the straight part crosses the line of centres at its midpoint, at angle beta = asin(2R / Stan)
    angle_arc(ax, mid, psiL, psiD, 2.6)
    text(ax, mid + 3.5 * unit(0.5 * (psiL + psiD)), r"$\beta$")
    finish(ax)
    fig.savefig(os.path.join(HERE, "fig_csc.pdf"), bbox_inches="tight", pad_inches=0.02)
    plt.close(fig)
    print(f"fig_csc  LSR: S1={S1:.2f} d={d:.2f} S2={S2:.2f} Stan={Stan:.2f} "
          f"psiL={math.degrees(psiL):.1f} deg psiD={math.degrees(psiD):.1f} deg "
          f"shortest={dp.findDubinsPath(*q1, *q2, R)[0]} rule={dp.findTurnType(*q1, *q2, R)}")


def ccc_figure():
    Stan_wanted, arc = 12.0, math.radians(60)
    th = math.acos(Stan_wanted / (4 * R))
    phi1 = th - arc                                       # start: 60 degrees before the cut-out on circle 1
    phi2 = math.pi - th + arc                             # end: 60 degrees after the cut-in on circle 2
    q1 = (R * math.cos(phi1), R * math.sin(phi1), phi1 + math.pi / 2)
    q2 = (Stan_wanted + R * math.cos(phi2), R * math.sin(phi2), phi2 + math.pi / 2)
    t = "LRL"
    S1, S3, S2 = (float(v) for v in dp.calculateCCCLengths(*q1, *q2, R, t))
    xs, ys, _, _ = dp.calculateCCCTrajectory(*q1, R, t, S1, S3, S2, 0.05)
    c1 = np.array(dp.returnCircleCenter(*q1, 1, R))
    c2 = np.array(dp.returnCircleCenter(*q2, 1, R))
    psiL = math.atan2(*(c2 - c1)[::-1])
    Stan = float(np.hypot(*(c2 - c1)))
    theta = math.acos(Stan / (4 * R))
    c3 = c1 + 2 * R * unit(psiL + theta)
    co = c1 + R * unit(psiL + theta)
    ci = c2 - R * unit(psiL - theta)
    p1, p2 = np.array(q1[:2]), np.array(q2[:2])

    fig, ax = plt.subplots(figsize=(4.2, 3.6))
    for c in (c1, c2, c3):
        circle(ax, c)
    segment(ax, c1, c2, ls=":")
    segment(ax, c1, c3, ls=":")
    segment(ax, c2, c3, ls=":")
    ax.plot(xs, ys, color=PATH, lw=1.5)
    arrow(ax, p1, q1[2])
    arrow(ax, p2, q2[2])
    for p in (p1, p2, co, ci):
        point(ax, p)

    text(ax, p1, r"$(x_1,y_1,\psi_1)$", (-0.4, -1.0), ha="right", va="top")
    text(ax, p2, r"$(x_2,y_2,\psi_2)$", (0.9, -1.3), ha="left", va="top")
    text(ax, c1, r"$(x_{c1},y_{c1})$", (-0.6, 0.0), ha="right", color=CON)
    text(ax, c2, r"$(x_{c2},y_{c2})$", (0.6, 0.0), ha="left", color=CON)
    text(ax, c3, r"$(x_{c3},y_{c3})$", (0.0, 0.9), color=CON)
    text(ax, co, r"$(x_{co},y_{co})$", (-0.8, 0.2), ha="right")
    text(ax, ci, r"$(x_{ci},y_{ci})$", (0.8, 0.2), ha="left")
    text(ax, 0.78 * c1 + 0.22 * c2, r"$S_{\tan}$", (0, -0.75), color=CON)
    text(ax, 0.3 * c1 + 0.7 * c3, r"$2R$", (-0.9, 0.3), color=CON)
    text(ax, 0.3 * c2 + 0.7 * c3, r"$2R$", (0.9, 0.3), color=CON)
    angle_arc(ax, c1, psiL, psiL + theta, 1.8)
    text(ax, c1 + 2.5 * unit(psiL + theta / 2), r"$\theta$")
    arc_label(ax, c1, phi1 + arc / 2, R - 1.3, r"$S_1$")
    arc_label(ax, c2, phi2 - arc / 2, R - 1.3, r"$S_2$")
    arc_label(ax, c3, math.radians(35), R + 1.0, r"$S_3$")
    finish(ax)
    fig.savefig(os.path.join(HERE, "fig_ccc.pdf"), bbox_inches="tight", pad_inches=0.02)
    plt.close(fig)
    print(f"fig_ccc  LRL: S1={S1:.2f} S3={S3:.2f} S2={S2:.2f} Stan={Stan:.2f} theta={math.degrees(theta):.2f} deg "
          f"shortest={dp.findDubinsPath(*q1, *q2, R)[0]}")


def six_types_figure():
    q1 = (0.0, 0.0, 0.0)
    q2 = (12.0, 1.5, math.radians(85))
    rule = dp.findTurnType(*q1, *q2, R)
    lengths = {t: dp.calculateLengths(*q1, *q2, R, t) for t in dp.TURN_TYPES}
    total = {t: float(sum(S)) for t, S in lengths.items()}
    best = min(total, key=total.get)
    fig, axes = plt.subplots(2, 3, figsize=(6.5, 4.1))
    for ax, t in zip(axes.flat, dp.TURN_TYPES):
        s1, s2 = dp.SIGMA[t[0]], dp.SIGMA[t[2]]
        for (x, y, psi, s) in [(*q1, s1), (*q2, s2)]:
            circle(ax, dp.returnCircleCenter(x, y, psi, s, R))
        if np.isfinite(total[t]):
            xs, ys, _, _ = dp.calculateTrajectory(*q1, R, t, *(float(v) for v in lengths[t]), 0.05)
            ax.plot(xs, ys, color=PATH, lw=1.3)
            label = rf"{t}: $S_{{\rm path}} = {total[t]:.1f}$ m"
        else:
            label = rf"{t}: does not exist"
        tags = (["shortest"] if t == best else []) + ([r"rule of Section~0.13"] if t == rule else [])
        label += "\n" + (r"\textbf{" + ", ".join(tags) + "}" if tags else r"\phantom{x}")
        arrow(ax, q1[:2], q1[2])
        arrow(ax, q2[:2], q2[2])
        point(ax, q1[:2])
        point(ax, q2[:2])
        ax.set_xlim(-11, 23)
        ax.set_ylim(-16, 18)
        finish(ax)
        ax.set_title(label, fontsize=9)
    fig.tight_layout(h_pad=0.6, w_pad=0.3)
    fig.savefig(os.path.join(HERE, "fig_six_types.pdf"), bbox_inches="tight", pad_inches=0.02)
    plt.close(fig)
    print("fig_six_types:", {t: round(v, 2) for t, v in total.items()}, "rule:", rule, "shortest:", best)


if __name__ == "__main__":
    csc_figure()
    ccc_figure()
    six_types_figure()
