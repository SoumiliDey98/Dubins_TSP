# ==============================================================================
# HOW TO READ THIS FILE
# Lines that start with "#" are notes for people. The computer skips them.
# Every other line is Python: instructions the computer follows from top to bottom.
# Text between three quote marks (""" ... """) is a "docstring": a description
# written for programmers. The computer keeps it but does not run it. The long
# docstring below lists every name used here, and the mistakes of the older
# notebook program that this file corrects.
#
# WHAT THIS FILE IS FOR
# The robot car can only drive forwards, at one speed, and it cannot turn
# tighter than a circle of radius R (5 m). So it cannot simply drive straight
# from A to B if it is pointing the wrong way. A "pose" is a position plus a
# heading: where the car is (x, y) and which way it points (psi, an angle).
# Lester Dubins proved in 1957 that the shortest way from one pose to another,
# for such a car, is always one of six shapes made of at most three pieces:
# circle arcs turning Left (L) or Right (R), and Straight lines (S):
#   LSL, RSR, LSR, RSL   arc, straight, arc   (called "CSC"; C means circle)
#   RLR, LRL             arc, arc, arc        (called "CCC")
# This file works out those shapes, their lengths, and points along them.
#
# ANGLES are in "radians": a full turn is 2*pi (about 6.283), half a turn is
# pi (about 3.1416), a right angle is pi/2. Heading 0 points along the x axis
# (east), and angles grow anticlockwise (towards north).
# ==============================================================================
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
# "import" borrows a toolbox. "math" has maths words; this file uses numpy's
# versions instead, but keeps this import.
import math

# numpy (nicknamed np) works with whole lists ("arrays") of numbers at once.
import numpy as np

# Short nicknames for numpy's maths, so the formulas below read like the paper
# and the notebook program. They work on single numbers and whole arrays alike.
# cos and sin: cosine and sine. For a heading psi, (cos psi, sin psi) is a step
# of length 1 in that direction.
cos = np.cos
# sin: sine.
sin = np.sin
# atan2(dy, dx): the direction (angle) of the arrow (dx, dy), right in every
# quarter of the circle.
atan2 = np.arctan2
# asin and acos: the angle whose sine, or whose cosine, is the given number.
asin = np.arcsin
# acos: the angle whose cosine is the given number.
acos = np.arccos
# sqrt: square root. pi: 3.14159..., half a turn in radians.
sqrt = np.sqrt
# pi: 3.14159..., half a turn in radians.
pi = np.pi

# The six Dubins shapes. A "tuple" (round brackets) is a fixed list.
TURN_TYPES = ("LSL", "RSR", "LSR", "RSL", "RLR", "LRL")
# sigma, the turning direction as a number: +1 for Left (anticlockwise), -1 for
# Right (clockwise). A dictionary looks a value up by its key (here a letter).
SIGMA = {"L": 1, "R": -1}


# ---------------------------------------------------------------------------
# Basic geometry
# ---------------------------------------------------------------------------

# "def" starts a recipe (a "function"). This one finds the middle of the circle
# the car drives round if it starts turning now. The car is at (x, y), points at
# heading psi, and turns left (sigma = +1) or right (sigma = -1).
def returnCircleCenter(x, y, psi, sigma, R):
    """Centre of the circle of direction sigma that passes through (x, y) with heading psi."""
    # The centre is R metres from the car, at a right angle to its heading: to
    # the left (psi + pi/2) for a left turn, to the right (psi - pi/2) for a right
    # turn. Its x is the car's x plus R times the cosine of that direction ...
    xc = x + R * cos(psi + sigma * pi / 2)
    # ... and its y is the car's y plus R times the sine of that direction.
    yc = y + R * sin(psi + sigma * pi / 2)
    # "return" hands the answer back: the centre (xc, yc).
    return xc, yc


# The straight-line distance between (x1, y1) and (x2, y2).
def distance(x1, x2, y1, y2):
    # Pythagoras: the square root of (y-difference squared + x-difference
    # squared). "** 2" means "squared" (a number times itself).
    return sqrt((y2 - y1) ** 2 + (x2 - x1) ** 2)


# How far round a circle the car must turn to change its heading from psiFrom
# to psiTo, when it may only turn one way (sigma).
def turnAngle(psiFrom, psiTo, sigma):
    """Angle, from 0 up to 2 pi, turned on a circle of direction sigma to go from heading
    psiFrom to heading psiTo. This replaces the 0.05 rad steps of the notebook (limitation 5)
    and needs no check for psi going past pi or -pi."""
    # The change of heading, times sigma: a right turn makes the heading go
    # DOWN, so multiplying by sigma = -1 makes the amount positive. np.mod(..., 2*pi)
    # keeps the result between 0 and one full turn: turning 7 radians ends where
    # turning 7 - 6.283 = 0.717 radians does.
    angle = np.mod(sigma * (psiTo - psiFrom), 2 * pi)
    # A tiny rounding error must not turn "no turn" into a full circle. Headings can be off by
    # about 1e-7 rad when Stan is almost exactly 2R, because asin is very sensitive near 1.
    # np.where(question, a, b) picks a where the answer is yes and b where it is
    # no: an angle within 0.000001 of a full turn becomes 0.
    return np.where(angle > 2 * pi - 1e-6, 0.0, angle)


# ---------------------------------------------------------------------------
# The rule of Section 0.13: which turn type?
# ---------------------------------------------------------------------------

# The paper's rule for which arc-straight-arc shape to use, from the signs of
# two "cross products", cp1 and cp2 (made in findTurnType below). The cross
# product of two arrows is positive if the second arrow points to the LEFT of
# the first one, and negative if it points to the right.
def findCSCTurnType(cp1, cp2):
    """CSC turn type from the signs of the cross products cp1 and cp2."""
    # cp1 <= 0: the end point is on the car's right at the start, so begin with
    # a Right arc. cp2 >= 0: the end heading turns left from the line between
    # the points, so finish with a Left arc. Together: RSL.
    if cp1 <= 0 and cp2 >= 0:                               # limitation 1: the notebook has cp2 > 0 here
        # The shape's name ...
        turn = "RSL"
        # ... the first arc turns right (-1) ...
        sigma1 = -1
        # ... and the last arc turns left (+1).
        sigma2 = 1
    # "elif" means "else, if": right at the start and right at the end: RSR.
    elif cp1 <= 0 and cp2 < 0:
        # The shape's name.
        turn = "RSR"
        # First arc right ...
        sigma1 = -1
        # ... last arc right.
        sigma2 = -1
    # Left at the start, right at the end: LSR.
    elif cp1 > 0 and cp2 < 0:
        # The shape's name.
        turn = "LSR"
        # First arc left ...
        sigma1 = 1
        # ... last arc right.
        sigma2 = -1
    # "else" catches what is left: left at the start and at the end: LSL.
    else:                                                   # cp1 > 0 and cp2 >= 0 (limitation 1)
        # The shape's name.
        turn = "LSL"
        # First arc left ...
        sigma1 = 1
        # ... last arc left.
        sigma2 = 1
    # Hand back the two turning directions and the name.
    return sigma1, sigma2, turn


# Turns an arc-straight-arc shape into its arc-arc-arc partner, if it has one.
def findCCCTurnType(turnOld):
    """CCC turn type for a CSC turn type: RSR gives RLR and LSL gives LRL. LSR and RSL have no
    CCC turn type, so proceed stays False."""
    # proceed says whether a partner exists; start with "no" (False).
    proceed = False
    # The turning directions start at 0, meaning "none yet" ...
    sigma1 = 0
    # The last arc's direction: none yet either.
    sigma2 = 0
    # ... and the name starts as the old one.
    turn = turnOld
    # turnOld[0] is the first letter and turnOld[2] the third. Only shapes that
    # turn the same way at both ends (RSR, LSL) have a three-arc partner.
    if turnOld[0] == turnOld[2]:
        # RSR becomes RLR: right, left, right.
        if turnOld[0] == "R":
            # The new name.
            turn = "RLR"
            # The outer arcs turn right.
            sigma1 = -1
            # (both of them: this is the last one)
            sigma2 = -1
        # LSL becomes LRL: left, right, left.
        elif turnOld[0] == "L":
            # The new name.
            turn = "LRL"
            # The outer arcs turn left.
            sigma1 = 1
            # (both of them: this is the last one)
            sigma2 = 1
        # Yes, a partner exists.
        proceed = True
    # Hand back the directions, the name and the yes/no.
    return sigma1, sigma2, turn, proceed


# The whole rule of Section 0.13: which shape, for a start pose (x1, y1, psi1)
# and an end pose (x2, y2, psi2)?
def findTurnType(x1, y1, psi1, x2, y2, psi2, R):
    """The turn type chosen by the rule of Section 0.13. It may be one that does not exist for
    these poses (LSR or RSL with Stan < 2R); findDubinsPath deals with that."""
    # u_vec1: an arrow of length 1 pointing the way the car starts.
    u_vec1 = (cos(psi1), sin(psi1))
    # u_vec2: the arrow from the start point to the end point.
    u_vec2 = (x2 - x1, y2 - y1)                             # limitation 2: not divided by its length
    # u_vec3: an arrow of length 1 pointing the way the car must end.
    u_vec3 = (cos(psi2), sin(psi2))
    # cp1: the cross product of u_vec1 and u_vec2 (ax * by - ay * bx). Positive
    # if the end point is on the car's left at the start. [0] is the x part of
    # an arrow and [1] its y part.
    cp1 = u_vec1[0] * u_vec2[1] - u_vec1[1] * u_vec2[0]
    # cp2: the cross product of u_vec2 and u_vec3. Positive if the end heading
    # points to the left of the line from start to end.
    cp2 = u_vec2[0] * u_vec3[1] - u_vec2[1] * u_vec3[0]

    # The arc-straight-arc shape from the two signs.
    sigma1, sigma2, turnType = findCSCTurnType(cp1, cp2)
    # The centre of the first circle ...
    xc1, yc1 = returnCircleCenter(x1, y1, psi1, sigma1, R)
    # ... and of the last circle.
    xc2, yc2 = returnCircleCenter(x2, y2, psi2, sigma2, R)

    # The paper's rule: if the centres are between 2R and 3R apart (10 to 15 m
    # when R = 5), use the three-arc shape instead, if there is one.
    if 2 * R <= distance(xc1, xc2, yc1, yc2) < 3 * R:
        # Look for the partner.
        sigma1, sigma2, turn, proceed = findCCCTurnType(turnType)
        # Switch only if one exists.
        if proceed:
            # Use the three-arc shape.
            turnType = turn
    # Hand back the chosen shape's name.
    return turnType                                         # limitation 3: the notebook returned no path in some cases


# ---------------------------------------------------------------------------
# Lengths of the three parts of a path. These work on single numbers and on arrays
# of headings, because trajectory.py compares many start and end headings at once.
# ---------------------------------------------------------------------------

# The lengths of the three pieces of an arc-straight-arc (CSC) shape.
def calculateCSCLengths(x1, y1, psi1, x2, y2, psi2, R, turnType):
    """(S1, dist, S2): first arc, straight part and last arc, in metres. All are inf if this
    turn type does not exist for these poses."""
    # The turning directions, from the first and third letters ("LSR" gives +1 and -1).
    sigma1, sigma2 = SIGMA[turnType[0]], SIGMA[turnType[2]]
    # The centre of the first circle ...
    xc1, yc1 = returnCircleCenter(x1, y1, psi1, sigma1, R)
    # ... and of the last circle.
    xc2, yc2 = returnCircleCenter(x2, y2, psi2, sigma2, R)
    # psiL: the direction from the first centre to the last centre.
    psiL = atan2(yc2 - yc1, xc2 - xc1)
    # Stan: how far apart the two centres are.
    Stan = distance(xc1, xc2, yc1, yc2)

    # Both arcs turn the same way: the straight piece runs along the outside of
    # both circles, parallel to the line joining their centres.
    if turnType == "RSR" or turnType == "LSL":              # straight line parallel to the line joining the centres
        # This shape always exists.
        exists = True
        # psiD, the heading along the straight piece, is psiL. If the two circles
        # are the same circle (centres 0 m apart), psiL means nothing, so use the
        # start heading instead.
        psiD = np.where(Stan < 1e-9, psi1, psiL)            # same circle twice: psiL means nothing, just keep turning
        # The straight piece is exactly as long as the gap between the centres.
        dist = Stan
    # The arcs turn opposite ways (LSR, RSL): the straight piece crosses between
    # the circles, like the cross-over in a figure 8.
    else:                                                   # limitation 4: asin, and the straight part is shorter than Stan
        # Possible only if the centres are at least 2R (10 m) apart; any closer
        # and the circles overlap. (1e-9 forgives tiny rounding errors.)
        exists = Stan >= 2 * R - 1e-9                       # closer than this, the two circles overlap: no straight line fits
        # The crossing line is tilted from the centre line by the angle whose
        # sine is 2R / Stan: a right-angled triangle with long side Stan and
        # short side 2R. np.maximum stops a division by 0; np.minimum keeps the
        # value at most 1, because a sine is never bigger than 1.
        cutOutAngle = asin(np.minimum(2 * R / np.maximum(Stan, 1e-9), 1.0))
        # Tilted one way for LSR and the other way for RSL. ("a if question
        # else b" picks a when the answer is yes, b when it is no.)
        psiD = psiL + cutOutAngle if turnType == "LSR" else psiL - cutOutAngle
        # The straight piece is that triangle's third side (Pythagoras):
        # sqrt(Stan^2 - (2R)^2). np.maximum(..., 0) stops a negative number.
        dist = sqrt(np.maximum(Stan ** 2 - 4 * R ** 2, 0.0))

    # An arc's length is its radius times its angle in radians. S1: the first
    # arc, turning from the start heading to the straight piece's heading.
    S1 = R * turnAngle(psi1, psiD, sigma1)
    # S2: the last arc, from the straight piece's heading to the end heading.
    S2 = R * turnAngle(psiD, psi2, sigma2)
    # Hand back the three lengths, with "infinity" (np.inf) wherever the shape
    # does not exist, so it can never be the shortest.
    return tuple(np.where(exists, S, np.inf) for S in (S1, dist, S2))


# The lengths of the three arcs of an arc-arc-arc (CCC) shape.
def calculateCCCLengths(x1, y1, psi1, x2, y2, psi2, R, turnType):
    """(S1, S3, S2): first arc, middle arc and last arc, in metres. All are inf if this turn
    type does not exist for these poses (Stan > 4R: the middle circle cannot touch both).

    The first and the middle circle touch at the cut-out point, and the middle and the last
    circle touch at the cut-in point. As in the notebook, the centre of the middle circle is
    (xc3, yc3) = (xc1, yc1) + 2R (cos, sin)(psiL + sigma1 * theta), with cos(theta) = Stan / 4R.
    The sink's heading where two circles touch is a right angle from the line joining their
    centres, which gives the headings psiCO and psiCI below."""
    # The first and last arcs turn the same way (RLR: right; LRL: left).
    sigma1 = sigma2 = SIGMA[turnType[0]]
    # The centres of the first and the last circle.
    xc1, yc1 = returnCircleCenter(x1, y1, psi1, sigma1, R)
    # The centre of the last circle.
    xc2, yc2 = returnCircleCenter(x2, y2, psi2, sigma2, R)
    # The direction and the distance from the first centre to the last.
    psiL = atan2(yc2 - yc1, xc2 - xc1)
    # Stan: how far apart the two centres are.
    Stan = distance(xc1, xc2, yc1, yc2)

    # The middle circle touches both outer circles, so its centre is 2R from
    # each of their centres. That only works if those are at most 4R (20 m) apart.
    exists = Stan <= 4 * R + 1e-9
    # The three centres make a triangle with sides 2R, 2R and Stan. theta is its
    # angle at the first centre: cos(theta) = (Stan / 2) / (2R) = Stan / 4R.
    theta = acos(np.minimum(Stan / (4 * R), 1.0))
    # Where two circles touch, the car points at a right angle (pi/2) to the line
    # joining their centres. psiCO: the heading where the first arc ends (the
    # "cut-out" point).
    psiCO = psiL + sigma1 * (theta + pi / 2)                # heading at the cut-out point
    # psiCI: the heading where the last arc begins (the "cut-in" point).
    psiCI = psiL - sigma1 * (theta + pi / 2)                # heading at the cut-in point

    # First arc: from the start heading to psiCO.
    S1 = R * turnAngle(psi1, psiCO, sigma1)
    # Middle arc: from psiCO to psiCI, turning the other way (-sigma1).
    S3 = R * turnAngle(psiCO, psiCI, -sigma1)               # the middle circle turns the other way
    # Last arc: from psiCI to the end heading.
    S2 = R * turnAngle(psiCI, psi2, sigma2)
    # Hand back the three lengths; infinity where the shape does not exist.
    return tuple(np.where(exists, S, np.inf) for S in (S1, S3, S2))


# Picks the right length recipe for any of the six shapes.
def calculateLengths(x1, y1, psi1, x2, y2, psi2, R, turnType):
    # The middle letter is "S" (straight): it is an arc-straight-arc shape.
    if turnType[1] == "S":
        # Use the arc-straight-arc recipe.
        return calculateCSCLengths(x1, y1, psi1, x2, y2, psi2, R, turnType)
    # Otherwise it is arc-arc-arc.
    return calculateCCCLengths(x1, y1, psi1, x2, y2, psi2, R, turnType)


# ---------------------------------------------------------------------------
# Choosing the path
# ---------------------------------------------------------------------------

# Chooses the shape for one leg and hands it back with its three lengths.
# method="shortest" is used when no method is given.
def findDubinsPath(x1, y1, psi1, x2, y2, psi2, R, method="shortest"):
    """The chosen path: (turnType, S1, Smiddle, S2), with the three lengths in metres."""
    # With the paper's rule ...
    if method == "pdf":
        # ... take the shape that Section 0.13 picks ...
        turnType = findTurnType(x1, y1, psi1, x2, y2, psi2, R)
        # ... and work out its lengths.
        lengths = calculateLengths(x1, y1, psi1, x2, y2, psi2, R, turnType)
        # If it exists (its length is not infinity), hand it back. The "*"
        # unpacks the three lengths into the answer; float(...) makes each a
        # plain decimal number.
        if np.isfinite(lengths[0]):
            # Hand back the rule's shape and its three lengths.
            return (turnType, *(float(S) for S in lengths))
        # limitation 3: this turn type does not exist here, so continue with the shortest one that does
    # The "shortest" way (and the fallback): the three lengths of all six
    # shapes, in a dictionary keyed by each shape's name.
    allLengths = {t: tuple(float(S) for S in calculateLengths(x1, y1, psi1, x2, y2, psi2, R, t)) for t in TURN_TYPES}
    # The shape whose three lengths add up to the least. "lambda" makes a tiny
    # nameless recipe: give it a name, it gives the total length.
    turnType = min(allLengths, key=lambda t: sum(allLengths[t]))
    # Hand back the name and its three lengths.
    return (turnType, *allLengths[turnType])


# Only the total length, but for MANY headings at once: trajectory.py compares
# 17 start headings with 17 end headings in one go.
def returnPathDistances(x1, y1, psi1, x2, y2, psi2, R, method="shortest"):
    """Only the pathDistance, for arrays of start and end headings. The arrays broadcast, so
    psi1[:, None] and psi2[None, :] give a table with one row per start heading."""
    # The total length of each of the six shapes, for every pair of headings.
    pathDistances = {t: sum(calculateLengths(x1, y1, psi1, x2, y2, psi2, R, t)) for t in TURN_TYPES}
    # For every pair of headings, the shortest of the six (axis=0 compares the
    # six shapes against each other).
    pathDistance = np.min(list(pathDistances.values()), axis=0)    # the shortest, also the fallback of "pdf"
    # With the paper's rule instead:
    if method == "pdf":
        # The rule's shape for every pair of headings. np.vectorize lets a
        # one-at-a-time recipe run over whole arrays.
        chosen = np.vectorize(findTurnType)(x1, y1, psi1, x2, y2, psi2, R)
        # For each shape and its lengths ...
        for turnType, S in pathDistances.items():
            # ... where the rule chose this shape and it exists, use its length;
            # everywhere else keep what is there (the shortest, as the fallback).
            pathDistance = np.where((chosen == turnType) & np.isfinite(S), S, pathDistance)
    # Hand back the table of lengths.
    return pathDistance


# ---------------------------------------------------------------------------
# The points of the path
# ---------------------------------------------------------------------------

# Where the car is after driving S metres round a circle. S may be one number,
# or a whole array of distances, giving many points at once.
def pointsOnArc(x, y, psi, sigma, R, S):
    """Where the sink is after driving S metres (a number or an array) on the circle of direction
    sigma, starting at (x, y) with heading psi. This is the rotation Mco of the notebook and of
    Section 0.13.1, with the exact angle S / R."""
    # The centre of the circle.
    xc, yc = returnCircleCenter(x, y, psi, sigma, R)
    # vec1: the arrow from the centre to the car.
    vec1 = (x - xc, y - yc)
    # Driving S metres round a circle of radius R turns the car by S / R radians.
    angle = S / R
    # Turn the arrow by that angle (anticlockwise if sigma = +1, clockwise if
    # -1) with the rotation formula x' = cos*x - sin*y, y' = sin*x + cos*y, and
    # add the centre back. This is the matrix Mco of Section 0.13.1.
    return (cos(angle) * vec1[0] - sigma * sin(angle) * vec1[1] + xc,
            sigma * sin(angle) * vec1[0] + cos(angle) * vec1[1] + yc)


# Of all the distances s along the path, keeps those inside one piece (from
# "start" to "start + length") and measures them from the start of that piece.
def _inPart(s, start, length):
    """Those of the distances s that fall in a part of the path, measured from the start of that part."""
    # "&" means "and": keep the ones in the piece, then subtract its start.
    return s[(s >= start) & (s < start + length)] - start


# The points along an arc-straight-arc path, "step" metres apart.
def calculateCSCTrajectory(x1, y1, psi1, R, turnType, S1, dist, S2, step):
    """Points of a CSC path, `step` metres apart: first arc to the cut-out point, straight part to
    the cut-in point, last arc to (x2, y2). Returns (xtrajectory, ytrajectory, strajectory, psiT)."""
    # The turning directions from the first and third letters.
    sigma1, sigma2 = SIGMA[turnType[0]], SIGMA[turnType[2]]
    # The total length of the path.
    pathDistance = S1 + dist + S2
    # The distances of the points along the path: 0, step, 2*step, ... up to
    # (but not including) the total length.
    s = np.arange(0.0, pathDistance, step)

    # The points on the first arc.
    xt1, yt1 = pointsOnArc(x1, y1, psi1, sigma1, R, _inPart(s, 0.0, S1))
    # The cut-out point: exactly where the first arc ends.
    xco, yco = pointsOnArc(x1, y1, psi1, sigma1, R, S1)
    # The heading there: the start heading turned by S1 / R radians.
    psiD = psi1 + sigma1 * S1 / R

    # u_hat: a step of length 1 along the straight piece.
    u_hat = (cos(psiD), sin(psiD))
    # The distances that fall on the straight piece.
    sLine = _inPart(s, S1, dist)
    # The cut-in point: the cut-out point moved "dist" metres along u_hat.
    xci, yci = xco + dist * u_hat[0], yco + dist * u_hat[1]

    # The points on the last arc, which starts at the cut-in point.
    xt2, yt2 = pointsOnArc(xci, yci, psiD, sigma2, R, _inPart(s, S1 + dist, S2))
    # The very end of the path (this is the end point x2, y2).
    xEnd, yEnd = pointsOnArc(xci, yci, psiD, sigma2, R, S2)
    # The heading at the end.
    psiT = psiD + sigma2 * S2 / R

    # Join all the x values in order: first arc, straight piece (cut-out point
    # plus distance times direction), last arc, end point.
    xtrajectory = np.concatenate([xt1, xco + sLine * u_hat[0], xt2, [xEnd]])
    # The same for the y values.
    ytrajectory = np.concatenate([yt1, yco + sLine * u_hat[1], yt2, [yEnd]])
    # Hand back the points, the distance along the path of each one (the
    # total length is added for the end point), and the end heading.
    return xtrajectory, ytrajectory, np.append(s, pathDistance), psiT


# The points along an arc-arc-arc path, "step" metres apart.
def calculateCCCTrajectory(x1, y1, psi1, R, turnType, S1, S3, S2, step):
    """Points of a CCC path, `step` metres apart: first arc to the cut-out point, middle arc to the
    cut-in point, last arc to (x2, y2). Returns (xtrajectory, ytrajectory, strajectory, psiT)."""
    # The outer arcs turn the same way.
    sigma1 = sigma2 = SIGMA[turnType[0]]
    # The total length of the path.
    pathDistance = S1 + S3 + S2
    # The distances of the points along the path, "step" apart.
    s = np.arange(0.0, pathDistance, step)

    # The points on the first arc.
    xt1, yt1 = pointsOnArc(x1, y1, psi1, sigma1, R, _inPart(s, 0.0, S1))
    # The cut-out point, where the first arc ends.
    xco, yco = pointsOnArc(x1, y1, psi1, sigma1, R, S1)
    # The heading there.
    psiT = psi1 + sigma1 * S1 / R

    # The points on the middle arc, which turns the other way (-sigma1).
    xt3, yt3 = pointsOnArc(xco, yco, psiT, -sigma1, R, _inPart(s, S1, S3))
    # The cut-in point, where the middle arc ends.
    xci, yci = pointsOnArc(xco, yco, psiT, -sigma1, R, S3)
    # The heading there.
    psiT = psiT - sigma1 * S3 / R

    # The points on the last arc.
    xt2, yt2 = pointsOnArc(xci, yci, psiT, sigma2, R, _inPart(s, S1 + S3, S2))
    # The very end of the path.
    xEnd, yEnd = pointsOnArc(xci, yci, psiT, sigma2, R, S2)
    # The heading at the end.
    psiT = psiT + sigma2 * S2 / R

    # Join the x values: first arc, middle arc, last arc, end point.
    xtrajectory = np.concatenate([xt1, xt3, xt2, [xEnd]])
    # The same for the y values.
    ytrajectory = np.concatenate([yt1, yt3, yt2, [yEnd]])
    # Hand back the points, their distances along the path, and the end heading.
    return xtrajectory, ytrajectory, np.append(s, pathDistance), psiT


# Picks the right points recipe: arc-straight-arc if the middle letter is S.
def calculateTrajectory(x1, y1, psi1, R, turnType, S1, Smiddle, S2, step):
    # The middle letter is S (straight): arc-straight-arc ...
    if turnType[1] == "S":
        # ... so use the arc-straight-arc points recipe.
        return calculateCSCTrajectory(x1, y1, psi1, R, turnType, S1, Smiddle, S2, step)
    # Otherwise arc-arc-arc.
    return calculateCCCTrajectory(x1, y1, psi1, R, turnType, S1, Smiddle, S2, step)


# The main recipe the other files use: the whole path from one pose to another.
# step=0.25 puts the points 0.25 m apart unless another step is given.
def returnDubinsPath(x1, y1, psi1, x2, y2, psi2, R, method="shortest", step=0.25):
    """The Dubins path from (x1, y1, psi1) to (x2, y2, psi2): (xtrajectory, ytrajectory,
    pathDistance, strajectory). The first three are those of the notebook program."""
    # Choose the shape and get its three lengths.
    turnType, S1, Smiddle, S2 = findDubinsPath(x1, y1, psi1, x2, y2, psi2, R, method)
    # Work out the points. "_" throws away the end heading, which is not needed here.
    xtrajectory, ytrajectory, strajectory, _ = calculateTrajectory(x1, y1, psi1, R, turnType, S1, Smiddle, S2, step)
    # Hand back the x values, the y values, the total length, and how far along
    # the path each point is (strajectory).
    return xtrajectory, ytrajectory, S1 + Smiddle + S2, strajectory
