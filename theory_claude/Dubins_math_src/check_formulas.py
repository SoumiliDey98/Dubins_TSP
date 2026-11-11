"""Check the closed forms written in Dubins_math.tex against dubins_path.py.

Run: python theory_claude/Dubins_math_src/check_formulas.py
Prints the number of CSC and CCC paths and the largest difference for each formula,
in metres or radians (20000 random pairs of poses, R = 5 m, points in a 40 m square).
"""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..",
                                "Dubins_TSP_New_Implementation_claude"))
import dubins_path as dp

rng = np.random.default_rng(0)
R = 5.0
worst = dict(psiD=0, cutin=0, ccc_co=0, ccc_ci=0, c3=0, S3=0, end=0, head=0, npts=0)
nccc = ncsc = 0
for _ in range(20000):
    x1, y1, x2, y2 = rng.uniform(-20, 20, 4)
    p1, p2 = rng.uniform(-math.pi, math.pi, 2)
    for t in dp.TURN_TYPES:
        S = dp.calculateLengths(x1, y1, p1, x2, y2, p2, R, t)
        if not np.isfinite(S[0]):
            continue
        s1, s2 = dp.SIGMA[t[0]], dp.SIGMA[t[2]]
        xc1, yc1 = dp.returnCircleCenter(x1, y1, p1, s1, R)
        xc2, yc2 = dp.returnCircleCenter(x2, y2, p2, s2, R)
        psiL = math.atan2(yc2 - yc1, xc2 - xc1); Stan = math.hypot(xc2 - xc1, yc2 - yc1)
        xs, ys, sss, psiT = dp.calculateTrajectory(x1, y1, p1, R, t, *S, 0.25)
        L = sum(S)
        worst["end"] = max(worst["end"], math.hypot(xs[-1] - x2, ys[-1] - y2))
        worst["head"] = max(worst["head"], abs((psiT - p2 + math.pi) % (2 * math.pi) - math.pi))
        worst["npts"] = max(worst["npts"], abs(len(xs) - (math.ceil(L / 0.25) + 1)) if L / 0.25 % 1 > 1e-6 else 0)
        a1 = S[0] / R
        xco = math.cos(a1) * (x1 - xc1) - s1 * math.sin(a1) * (y1 - yc1) + xc1
        yco = s1 * math.sin(a1) * (x1 - xc1) + math.cos(a1) * (y1 - yc1) + yc1
        if t[1] == "S":
            ncsc += 1
            # unified psiD: psiL + sigma1 * asin(...) when sigma1 != sigma2, psiL otherwise
            psiD = psiL if s1 == s2 else psiL + s1 * math.asin(min(2 * R / Stan, 1))
            got = p1 + s1 * S[0] / R
            worst["psiD"] = max(worst["psiD"], abs((got - psiD + math.pi) % (2 * math.pi) - math.pi)) if Stan > 1e-6 else worst["psiD"]
            # cut-in: forward from cut-out == A13 backward rotation of end point
            xci_f, yci_f = xco + S[1] * math.cos(got), yco + S[1] * math.sin(got)
            a2 = S[2] / R
            xci_b = math.cos(a2) * (x2 - xc2) + s2 * math.sin(a2) * (y2 - yc2) + xc2
            yci_b = -s2 * math.sin(a2) * (x2 - xc2) + math.cos(a2) * (y2 - yc2) + yc2
            worst["cutin"] = max(worst["cutin"], math.hypot(xci_f - xci_b, yci_f - yci_b))
        else:
            nccc += 1
            th = math.acos(min(Stan / (4 * R), 1))
            worst["ccc_co"] = max(worst["ccc_co"], math.hypot(xco - xc1 - R * math.cos(psiL + s1 * th), yco - yc1 - R * math.sin(psiL + s1 * th)))
            psico = p1 + s1 * S[0] / R
            xc3, yc3 = dp.returnCircleCenter(xco, yco, psico, -s1, R)
            worst["c3"] = max(worst["c3"], math.hypot(xc3 - xc1 - 2 * R * math.cos(psiL + s1 * th), yc3 - yc1 - 2 * R * math.sin(psiL + s1 * th)))
            xci, yci = dp.pointsOnArc(xco, yco, psico, -s1, R, S[1])
            worst["ccc_ci"] = max(worst["ccc_ci"], math.hypot(xci - xc2 + R * math.cos(psiL - s1 * th), yci - yc2 + R * math.sin(psiL - s1 * th)))
            worst["S3"] = max(worst["S3"], abs(S[1] - R * (math.pi + 2 * th)))
print("CSC", ncsc, "CCC", nccc)
for k, v in worst.items():
    print(f"{k:7s} {v:.3e}")
# S3 when the two CCC circles coincide
x1, y1, p1 = 0.0, 0.0, 0.3
xc1, yc1 = dp.returnCircleCenter(x1, y1, p1, 1, R)
p2 = 2.0; x2 = xc1 + R * math.cos(p2 - math.pi / 2); y2 = yc1 + R * math.sin(p2 - math.pi / 2)
print("same circle LRL:", dp.calculateCCCLengths(x1, y1, p1, x2, y2, p2, R, "LRL"), "expected S1 =", R * (p2 - p1))
# tie order of findDubinsPath
print("np.arange count check:", len(np.arange(0.0, 10.0, 0.25)), math.ceil(10 / 0.25))
