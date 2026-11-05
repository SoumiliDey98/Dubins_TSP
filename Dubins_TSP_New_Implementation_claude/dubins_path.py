"""Dubins path between two poses (x, y, heading): Sections 0.1 and 0.12-0.13 of Dubins_TSP.pdf.

The sink drives forward only, at constant speed, and turns no tighter than radius R.

The names and the order of the steps are those of the DubinsPath notebook program
(dubinspath.py, which is not stored in this repository):

    x1, y1, psi1      start point and start heading
    x2, y2, psi2      end point and end heading
    R                 minimum turning radius
    turnType          LSL, RSR, LSR, RSL, RLR or LRL (L = left arc, R = right arc, S = straight)
    sigma1, sigma2    direction of the first and the last arc: +1 = left, -1 = right
    u_vec1, u_vec2, u_vec3, cp1, cp2    the vectors and cross products that give the turn type
    xc1, yc1          centre of the first circle (xc2, yc2: last circle; xc3, yc3: middle circle of CCC)
    Stan              distance between the centres of the first and the last circle
    psiL              direction of the line from the first centre to the last centre
    psiD              heading of the straight part (the cut-out angle)
    xco, yco          cut-out point: where the first arc ends
    xci, yci          cut-in point: where the last arc starts
    psiT              heading of the sink at the end of the part drawn so far
    u_hat, dist       direction and length of the straight part
    xt1, yt1          points of the first arc (xt2, yt2: last arc; xt3, yt3: middle arc of CCC)
    pathDistance      total length of the path, in metres
    xtrajectory, ytrajectory    the points of the path

New names, which the notebook does not have:

    S1, S2            length of the first and the last arc, in metres
    S3                length of the middle arc of a CCC path, in metres
    Smiddle           length of the middle part, whichever kind it is: dist for CSC, S3 for CCC
    strajectory       distance along the path of each point (needed for the time t = s / v)
    method            how the turn type is chosen, see below

The turn type is chosen in one of two ways (p["dubins_method"] in config.py):

    "pdf"       The rule of Section 0.13, as in the notebook: the signs of cp1 and cp2 give
                the turn type, and RSR / LSL become RLR / LRL if 2R <= Stan < 3R.
    "shortest"  Compute all six turn types and keep the shortest (the default). Section 0.13
                does not describe this, but it is the true shortest path.


LIMITATIONS OF THE NOTEBOOK PROGRAM, AND HOW THIS FILE CORRECTS THEM

 1. It stopped with an error when cp2 was exactly 0, which happens when the end heading points
    exactly along the line between the two points. Here cp2 = 0 counts as a left turn, as in
    findCSCTurnType of Dubins_TSP_Antigravity/dubins.py.

 2. It stopped with an error when the two points were the same, because u_vec2 was divided by
    its length, which is 0. Only the signs of cp1 and cp2 matter, so here u_vec2 is not divided.

 3. It returned no path when Stan < 2R, and for LSR or RSL when 2R <= Stan < 3R. About 4 in 10
    random poses got no path, although a Dubins path always exists. Here LSL and RSR are used
    for any Stan, as Section 0.13 says ("else it is a CSC curve"), and LSR and RSL for any
    Stan >= 2R. LSR and RSL really do not exist when Stan < 2R, and Section 0.13 does not say
    what to do then, so here the shortest turn type that does exist is used.

 4. For LSR and RSL it used the cut-out angle atan(2R / Stan), as Section 0.13 does. That would
    be right if Stan were the length of the straight part, but Stan is the distance between the
    circle centres. So the straight part did not leave the first circle smoothly: at Stan = 3R
    the angle is wrong by 8 degrees. Here psiD = psiL +/- asin(2R / Stan), and the straight part
    has length sqrt(Stan^2 - 4R^2).

 5. It drew arcs in steps of 0.05 rad and stopped when the heading came within 0.1 rad of psiD.
    So the heading could be wrong where two parts meet, those parts could meet at a small corner,
    and the length was only approximate. If the heading started just past psiD, an almost full
    circle was skipped, and the length could be wrong by about 2 pi R (31 m when R = 5).
    Here every arc has its exact angle (see turnAngle).

 6. Its counter i grew before its first use, so each arc point was one step (0.05 rad) ahead of
    psiT, and the length counted one step too few. There is no such counter here.

 7. In CCC paths each arc stopped about 3 m before its end and jumped there, and the jumps were
    not added to pathDistance. With R = 5 the length came out about 13 m too short on average.
    With R = 1 the arcs were not drawn at all and the length could come out as 0. Here every arc
    is exact, for any R.

 8. Its points were unevenly spaced (small steps on arcs, 3 m on straight parts) and had no time
    information. Here the points are `step` metres apart everywhere, and strajectory gives the
    distance along the path of each one.


A LIMITATION THAT REMAINS WITH method="pdf"

 9. The rule of Section 0.13 does not always give the shortest path. The signs of cp1 and cp2 only
    say on which side the next point and the end heading lie, and CCC is only tried when
    2R <= Stan < 3R, although it exists up to Stan = 4R and is often the shortest above 3R.
    Following the rule means keeping this limitation. For the true shortest path use
    method="shortest". The README measures the difference.
"""
import math

import numpy as np

cos = np.cos
sin = np.sin
atan2 = np.arctan2
asin = np.arcsin
acos = np.arccos
sqrt = np.sqrt
pi = np.pi

TURN_TYPES = ("LSL", "RSR", "LSR", "RSL", "RLR", "LRL")
SIGMA = {"L": 1, "R": -1}


# ---------------------------------------------------------------------------
# Basic geometry
# ---------------------------------------------------------------------------

def returnCircleCenter(x, y, psi, sigma, R):
    """Centre of the circle of direction sigma that passes through (x, y) with heading psi."""
    xc = x + R * cos(psi + sigma * pi / 2)
    yc = y + R * sin(psi + sigma * pi / 2)
    return xc, yc


def distance(x1, x2, y1, y2):
    return sqrt((y2 - y1) ** 2 + (x2 - x1) ** 2)


def turnAngle(psiFrom, psiTo, sigma):
    """Angle, from 0 up to 2 pi, turned on a circle of direction sigma to go from heading
    psiFrom to heading psiTo. This replaces the 0.05 rad steps of the notebook (limitation 5)
    and needs no check for psi going past pi or -pi."""
    angle = np.mod(sigma * (psiTo - psiFrom), 2 * pi)
    # A tiny rounding error must not turn "no turn" into a full circle. Headings can be off by
    # about 1e-7 rad when Stan is almost exactly 2R, because asin is very sensitive near 1.
    return np.where(angle > 2 * pi - 1e-6, 0.0, angle)


# ---------------------------------------------------------------------------
# The rule of Section 0.13: which turn type?
# ---------------------------------------------------------------------------

def findCSCTurnType(cp1, cp2):
    """CSC turn type from the signs of the cross products cp1 and cp2."""
    if cp1 <= 0 and cp2 >= 0:                               # limitation 1: the notebook has cp2 > 0 here
        turn = "RSL"
        sigma1 = -1
        sigma2 = 1
    elif cp1 <= 0 and cp2 < 0:
        turn = "RSR"
        sigma1 = -1
        sigma2 = -1
    elif cp1 > 0 and cp2 < 0:
        turn = "LSR"
        sigma1 = 1
        sigma2 = -1
    else:                                                   # cp1 > 0 and cp2 >= 0 (limitation 1)
        turn = "LSL"
        sigma1 = 1
        sigma2 = 1
    return sigma1, sigma2, turn


def findCCCTurnType(turnOld):
    """CCC turn type for a CSC turn type: RSR gives RLR and LSL gives LRL. LSR and RSL have no
    CCC turn type, so proceed stays False."""
    proceed = False
    sigma1 = 0
    sigma2 = 0
    turn = turnOld
    if turnOld[0] == turnOld[2]:
        if turnOld[0] == "R":
            turn = "RLR"
            sigma1 = -1
            sigma2 = -1
        elif turnOld[0] == "L":
            turn = "LRL"
            sigma1 = 1
            sigma2 = 1
        proceed = True
    return sigma1, sigma2, turn, proceed


def findTurnType(x1, y1, psi1, x2, y2, psi2, R):
    """The turn type chosen by the rule of Section 0.13. It may be one that does not exist for
    these poses (LSR or RSL with Stan < 2R); findDubinsPath deals with that."""
    u_vec1 = (cos(psi1), sin(psi1))
    u_vec2 = (x2 - x1, y2 - y1)                             # limitation 2: not divided by its length
    u_vec3 = (cos(psi2), sin(psi2))
    cp1 = u_vec1[0] * u_vec2[1] - u_vec1[1] * u_vec2[0]
    cp2 = u_vec2[0] * u_vec3[1] - u_vec2[1] * u_vec3[0]

    sigma1, sigma2, turnType = findCSCTurnType(cp1, cp2)
    xc1, yc1 = returnCircleCenter(x1, y1, psi1, sigma1, R)
    xc2, yc2 = returnCircleCenter(x2, y2, psi2, sigma2, R)

    if 2 * R <= distance(xc1, xc2, yc1, yc2) < 3 * R:
        sigma1, sigma2, turn, proceed = findCCCTurnType(turnType)
        if proceed:
            turnType = turn
    return turnType                                         # limitation 3: the notebook returned no path in some cases


# ---------------------------------------------------------------------------
# Lengths of the three parts of a path. These work on single numbers and on arrays
# of headings, because trajectory.py compares many start and end headings at once.
# ---------------------------------------------------------------------------

def calculateCSCLengths(x1, y1, psi1, x2, y2, psi2, R, turnType):
    """(S1, dist, S2): first arc, straight part and last arc, in metres. All are inf if this
    turn type does not exist for these poses."""
    sigma1, sigma2 = SIGMA[turnType[0]], SIGMA[turnType[2]]
    xc1, yc1 = returnCircleCenter(x1, y1, psi1, sigma1, R)
    xc2, yc2 = returnCircleCenter(x2, y2, psi2, sigma2, R)
    psiL = atan2(yc2 - yc1, xc2 - xc1)
    Stan = distance(xc1, xc2, yc1, yc2)

    if turnType == "RSR" or turnType == "LSL":              # straight line parallel to the line joining the centres
        exists = True
        psiD = np.where(Stan < 1e-9, psi1, psiL)            # same circle twice: psiL means nothing, just keep turning
        dist = Stan
    else:                                                   # limitation 4: asin, and the straight part is shorter than Stan
        exists = Stan >= 2 * R - 1e-9                       # closer than this, the two circles overlap: no straight line fits
        cutOutAngle = asin(np.minimum(2 * R / np.maximum(Stan, 1e-9), 1.0))
        psiD = psiL + cutOutAngle if turnType == "LSR" else psiL - cutOutAngle
        dist = sqrt(np.maximum(Stan ** 2 - 4 * R ** 2, 0.0))

    S1 = R * turnAngle(psi1, psiD, sigma1)
    S2 = R * turnAngle(psiD, psi2, sigma2)
    return tuple(np.where(exists, S, np.inf) for S in (S1, dist, S2))


def calculateCCCLengths(x1, y1, psi1, x2, y2, psi2, R, turnType):
    """(S1, S3, S2): first arc, middle arc and last arc, in metres. All are inf if this turn
    type does not exist for these poses (Stan > 4R: the middle circle cannot touch both).

    The first and the middle circle touch at the cut-out point, and the middle and the last
    circle touch at the cut-in point. As in the notebook, the centre of the middle circle is
    (xc3, yc3) = (xc1, yc1) + 2R (cos, sin)(psiL + sigma1 * theta), with cos(theta) = Stan / 4R.
    The sink's heading where two circles touch is a right angle from the line joining their
    centres, which gives the headings psiCO and psiCI below."""
    sigma1 = sigma2 = SIGMA[turnType[0]]
    xc1, yc1 = returnCircleCenter(x1, y1, psi1, sigma1, R)
    xc2, yc2 = returnCircleCenter(x2, y2, psi2, sigma2, R)
    psiL = atan2(yc2 - yc1, xc2 - xc1)
    Stan = distance(xc1, xc2, yc1, yc2)

    exists = Stan <= 4 * R + 1e-9
    theta = acos(np.minimum(Stan / (4 * R), 1.0))
    psiCO = psiL + sigma1 * (theta + pi / 2)                # heading at the cut-out point
    psiCI = psiL - sigma1 * (theta + pi / 2)                # heading at the cut-in point

    S1 = R * turnAngle(psi1, psiCO, sigma1)
    S3 = R * turnAngle(psiCO, psiCI, -sigma1)               # the middle circle turns the other way
    S2 = R * turnAngle(psiCI, psi2, sigma2)
    return tuple(np.where(exists, S, np.inf) for S in (S1, S3, S2))


def calculateLengths(x1, y1, psi1, x2, y2, psi2, R, turnType):
    if turnType[1] == "S":
        return calculateCSCLengths(x1, y1, psi1, x2, y2, psi2, R, turnType)
    return calculateCCCLengths(x1, y1, psi1, x2, y2, psi2, R, turnType)


# ---------------------------------------------------------------------------
# Choosing the path
# ---------------------------------------------------------------------------

def findDubinsPath(x1, y1, psi1, x2, y2, psi2, R, method="shortest"):
    """The chosen path: (turnType, S1, Smiddle, S2), with the three lengths in metres."""
    if method == "pdf":
        turnType = findTurnType(x1, y1, psi1, x2, y2, psi2, R)
        lengths = calculateLengths(x1, y1, psi1, x2, y2, psi2, R, turnType)
        if np.isfinite(lengths[0]):
            return (turnType, *(float(S) for S in lengths))
        # limitation 3: this turn type does not exist here, so continue with the shortest one that does
    allLengths = {t: tuple(float(S) for S in calculateLengths(x1, y1, psi1, x2, y2, psi2, R, t)) for t in TURN_TYPES}
    turnType = min(allLengths, key=lambda t: sum(allLengths[t]))
    return (turnType, *allLengths[turnType])


def returnPathDistances(x1, y1, psi1, x2, y2, psi2, R, method="shortest"):
    """Only the pathDistance, for arrays of start and end headings. The arrays broadcast, so
    psi1[:, None] and psi2[None, :] give a table with one row per start heading."""
    pathDistances = {t: sum(calculateLengths(x1, y1, psi1, x2, y2, psi2, R, t)) for t in TURN_TYPES}
    pathDistance = np.min(list(pathDistances.values()), axis=0)    # the shortest, also the fallback of "pdf"
    if method == "pdf":
        chosen = np.vectorize(findTurnType)(x1, y1, psi1, x2, y2, psi2, R)
        for turnType, S in pathDistances.items():
            pathDistance = np.where((chosen == turnType) & np.isfinite(S), S, pathDistance)
    return pathDistance


# ---------------------------------------------------------------------------
# The points of the path
# ---------------------------------------------------------------------------

def pointsOnArc(x, y, psi, sigma, R, S):
    """Where the sink is after driving S metres (a number or an array) on the circle of direction
    sigma, starting at (x, y) with heading psi. This is the rotation Mco of the notebook and of
    Section 0.13.1, with the exact angle S / R."""
    xc, yc = returnCircleCenter(x, y, psi, sigma, R)
    vec1 = (x - xc, y - yc)
    angle = S / R
    return (cos(angle) * vec1[0] - sigma * sin(angle) * vec1[1] + xc,
            sigma * sin(angle) * vec1[0] + cos(angle) * vec1[1] + yc)


def _inPart(s, start, length):
    """Those of the distances s that fall in a part of the path, measured from the start of that part."""
    return s[(s >= start) & (s < start + length)] - start


def calculateCSCTrajectory(x1, y1, psi1, R, turnType, S1, dist, S2, step):
    """Points of a CSC path, `step` metres apart: first arc to the cut-out point, straight part to
    the cut-in point, last arc to (x2, y2). Returns (xtrajectory, ytrajectory, strajectory, psiT)."""
    sigma1, sigma2 = SIGMA[turnType[0]], SIGMA[turnType[2]]
    pathDistance = S1 + dist + S2
    s = np.arange(0.0, pathDistance, step)

    xt1, yt1 = pointsOnArc(x1, y1, psi1, sigma1, R, _inPart(s, 0.0, S1))
    xco, yco = pointsOnArc(x1, y1, psi1, sigma1, R, S1)
    psiD = psi1 + sigma1 * S1 / R

    u_hat = (cos(psiD), sin(psiD))
    sLine = _inPart(s, S1, dist)
    xci, yci = xco + dist * u_hat[0], yco + dist * u_hat[1]

    xt2, yt2 = pointsOnArc(xci, yci, psiD, sigma2, R, _inPart(s, S1 + dist, S2))
    xEnd, yEnd = pointsOnArc(xci, yci, psiD, sigma2, R, S2)
    psiT = psiD + sigma2 * S2 / R

    xtrajectory = np.concatenate([xt1, xco + sLine * u_hat[0], xt2, [xEnd]])
    ytrajectory = np.concatenate([yt1, yco + sLine * u_hat[1], yt2, [yEnd]])
    return xtrajectory, ytrajectory, np.append(s, pathDistance), psiT


def calculateCCCTrajectory(x1, y1, psi1, R, turnType, S1, S3, S2, step):
    """Points of a CCC path, `step` metres apart: first arc to the cut-out point, middle arc to the
    cut-in point, last arc to (x2, y2). Returns (xtrajectory, ytrajectory, strajectory, psiT)."""
    sigma1 = sigma2 = SIGMA[turnType[0]]
    pathDistance = S1 + S3 + S2
    s = np.arange(0.0, pathDistance, step)

    xt1, yt1 = pointsOnArc(x1, y1, psi1, sigma1, R, _inPart(s, 0.0, S1))
    xco, yco = pointsOnArc(x1, y1, psi1, sigma1, R, S1)
    psiT = psi1 + sigma1 * S1 / R

    xt3, yt3 = pointsOnArc(xco, yco, psiT, -sigma1, R, _inPart(s, S1, S3))
    xci, yci = pointsOnArc(xco, yco, psiT, -sigma1, R, S3)
    psiT = psiT - sigma1 * S3 / R

    xt2, yt2 = pointsOnArc(xci, yci, psiT, sigma2, R, _inPart(s, S1 + S3, S2))
    xEnd, yEnd = pointsOnArc(xci, yci, psiT, sigma2, R, S2)
    psiT = psiT + sigma2 * S2 / R

    xtrajectory = np.concatenate([xt1, xt3, xt2, [xEnd]])
    ytrajectory = np.concatenate([yt1, yt3, yt2, [yEnd]])
    return xtrajectory, ytrajectory, np.append(s, pathDistance), psiT


def calculateTrajectory(x1, y1, psi1, R, turnType, S1, Smiddle, S2, step):
    if turnType[1] == "S":
        return calculateCSCTrajectory(x1, y1, psi1, R, turnType, S1, Smiddle, S2, step)
    return calculateCCCTrajectory(x1, y1, psi1, R, turnType, S1, Smiddle, S2, step)


def returnDubinsPath(x1, y1, psi1, x2, y2, psi2, R, method="shortest", step=0.25):
    """The Dubins path from (x1, y1, psi1) to (x2, y2, psi2): (xtrajectory, ytrajectory,
    pathDistance, strajectory). The first three are those of the notebook program."""
    turnType, S1, Smiddle, S2 = findDubinsPath(x1, y1, psi1, x2, y2, psi2, R, method)
    xtrajectory, ytrajectory, strajectory, _ = calculateTrajectory(x1, y1, psi1, R, turnType, S1, Smiddle, S2, step)
    return xtrajectory, ytrajectory, S1 + Smiddle + S2, strajectory
