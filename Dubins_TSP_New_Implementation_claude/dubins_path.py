"""Dubins paths between two poses (Sections 0.1, 0.12-0.13).

The sink drives forward only, at constant speed, and turns no tighter than
radius R. The shortest path between two poses (x, y, heading) is one of six
words: LSL, RSR, LSR, RSL, RLR, LRL (L = left arc, R = right arc, S = straight).
Segment lengths (t, p, q) are in units of R: arcs are angles in radians, the
straight part is p * R metres.

The word is chosen in one of two ways (p["dubins_method"]):
  "shortest"  compute all six and keep the shortest, as dubins.py of the original
              code does. This is not what Section 0.13 describes, but it is the
              true shortest path.
  "pdf"       the rule of Section 0.13: the signs of the cross products cp1, cp2
              give one CSC word, and RSR / LSL become RLR / LRL if 2R <= S_tan < 3R.
              This is not always the shortest path.

Section 0.13 leaves some cases open. "pdf" settles them as follows:
  - if the chosen word does not exist (LSR or RSL with S_tan < 2R), the shortest
    word that does exist is used;
  - cp2 = 0 counts as a left turn, as in findCSCTurnType of dubins.py.
Both methods use the same exact closed-form geometry; the Delta-psi stepping of
Section 0.13.1 is not used, so "pdf" follows the paper's choice of word only.
"""
import math

import numpy as np

TWO_PI = 2 * math.pi
WORDS = ("LSL", "RSR", "LSR", "RSL", "RLR", "LRL")


def _words(a, b, d):
    """Segment lengths (t, p, q) of the six words. a, b: headings relative to the
    line joining the two points; d: their distance divided by R. Works on numpy
    arrays; an impossible word gets length inf."""
    a, b, d = np.broadcast_arrays(np.asarray(a, float), np.asarray(b, float), np.asarray(d, float))
    sa, sb, ca, cb, cab = np.sin(a), np.sin(b), np.cos(a), np.cos(b), np.cos(a - b)
    m = lambda x: np.mod(x, TWO_PI)
    ok = lambda mask, t, p, q: tuple(np.where(mask, x, np.inf) for x in (t, p, q))
    with np.errstate(invalid="ignore"):
        p2 = 2 + d * d - 2 * cab + 2 * d * (sa - sb)
        t = np.arctan2(cb - ca, d + sa - sb)
        lsl = ok(p2 >= 0, m(-a + t), np.sqrt(p2), m(b - t))
        p2 = 2 + d * d - 2 * cab + 2 * d * (sb - sa)
        t = np.arctan2(ca - cb, d - sa + sb)
        rsr = ok(p2 >= 0, m(a - t), np.sqrt(p2), m(-b + t))
        p2 = -2 + d * d + 2 * cab + 2 * d * (sa + sb)
        p = np.sqrt(p2)
        t = np.arctan2(-ca - cb, d + sa + sb) - np.arctan2(-2.0, p)
        lsr = ok(p2 >= 0, m(-a + t), p, m(-b + t))
        p2 = -2 + d * d + 2 * cab - 2 * d * (sa + sb)
        p = np.sqrt(p2)
        t = np.arctan2(ca + cb, d - sa - sb) - np.arctan2(2.0, p)
        rsl = ok(p2 >= 0, m(a - t), p, m(b - t))
        c = (6 - d * d + 2 * cab + 2 * d * (sa - sb)) / 8
        p = m(TWO_PI - np.arccos(c))
        t = m(a - np.arctan2(ca - cb, d - sa + sb) + p / 2)
        rlr = ok(np.abs(c) <= 1, t, p, m(a - b - t + p))
        c = (6 - d * d + 2 * cab + 2 * d * (sb - sa)) / 8
        p = m(TWO_PI - np.arccos(c))
        t = m(-a - np.arctan2(ca - cb, d + sa - sb) + p / 2)
        lrl = ok(np.abs(c) <= 1, t, p, m(m(b) - a - t + p))
    return dict(zip(WORDS, (lsl, rsr, lsr, rsl, rlr, lrl)))


def _relative(xy0, h0, xy1, h1, R):
    dx, dy = xy1[0] - xy0[0], xy1[1] - xy0[1]
    th = math.atan2(dy, dx) if (dx or dy) else 0.0
    return np.mod(np.asarray(h0) - th, TWO_PI), np.mod(np.asarray(h1) - th, TWO_PI), math.hypot(dx, dy) / R


def _pdf_word_index(xy0, h0, xy1, h1, R):
    """Index in WORDS of the word chosen by the rule of Section 0.13 (works on arrays)."""
    h0, h1 = np.asarray(h0, float), np.asarray(h1, float)
    dx, dy = xy1[0] - xy0[0], xy1[1] - xy0[1]
    cp1 = np.cos(h0) * dy - np.sin(h0) * dx                # u1 x u2
    cp2 = dx * np.sin(h1) - dy * np.cos(h1)                # u2 x u3
    s1, s2 = np.where(cp1 <= 0, -1.0, 1.0), np.where(cp2 >= 0, 1.0, -1.0)
    # S_tan: distance between the circle centres (x, y) + R (cos, sin)(heading + sigma * pi / 2)
    s_tan = np.hypot(dx - R * (s2 * np.sin(h1) - s1 * np.sin(h0)), dy + R * (s2 * np.cos(h1) - s1 * np.cos(h0)))
    ccc = (2 * R <= s_tan) & (s_tan < 3 * R)
    return np.select([(s1 > 0) & (s2 > 0) & ccc, (s1 < 0) & (s2 < 0) & ccc,        # LRL, RLR
                      (s1 > 0) & (s2 > 0), (s1 < 0) & (s2 < 0), s1 > 0],           # LSL, RSR, LSR
                     [WORDS.index("LRL"), WORDS.index("RLR"),
                      WORDS.index("LSL"), WORDS.index("RSR"), WORDS.index("LSR")], WORDS.index("RSL"))


def lengths(xy0, h0, xy1, h1, R, method="shortest"):
    """Dubins length from point xy0 to point xy1 for arrays of start and end headings
    (they broadcast, e.g. h0[:, None] and h1[None, :] give a table)."""
    a, b, d = _relative(xy0, h0, xy1, h1, R)
    total = np.array([sum(seg) for seg in _words(a, b, d).values()])
    best = total.min(axis=0)
    if method == "pdf":
        i = np.broadcast_to(_pdf_word_index(xy0, h0, xy1, h1, R), best.shape)
        picked = np.take_along_axis(total, i[None], axis=0)[0]
        best = np.where(np.isfinite(picked), picked, best)
    return best * R


def shortest(q0, q1, R):
    """Shortest path from pose q0 = (x, y, heading) to pose q1: (word, (t, p, q), length in metres)."""
    a, b, d = _relative(q0[:2], q0[2], q1[:2], q1[2], R)
    words = {w: tuple(float(x) for x in seg) for w, seg in _words(a, b, d).items()}
    word, seg = min(words.items(), key=lambda kv: sum(kv[1]))
    return word, seg, sum(seg) * R


def pdf_word(q0, q1, R):
    """The word the rule of Section 0.13 picks for these poses. It may not exist; see select."""
    return WORDS[int(_pdf_word_index(q0[:2], q0[2], q1[:2], q1[2], R))]


def select(q0, q1, R, method="shortest"):
    """Path from pose q0 to pose q1 with the chosen method: (word, (t, p, q), length in metres)."""
    if method == "pdf":
        a, b, d = _relative(q0[:2], q0[2], q1[:2], q1[2], R)
        word = pdf_word(q0, q1, R)
        seg = tuple(float(x) for x in _words(a, b, d)[word])
        if math.isfinite(sum(seg)):
            return word, seg, sum(seg) * R
    return shortest(q0, q1, R)


def sample(q0, word, seg, R, step):
    """Points every `step` metres along the path (and its end point), with their
    arc length s from the start. Returns (xy (n, 2), s (n,), end heading)."""
    x, y, th = q0
    s0 = 0.0
    total = sum(seg) * R
    grid = np.arange(0.0, total, step)
    pts = []
    for turn, length in zip(word, seg):
        L = length * R
        u = grid[(grid >= s0) & (grid < s0 + L)] - s0
        if turn == "S":
            pts.append(np.column_stack([x + u * np.cos(th), y + u * np.sin(th)]))
            x, y = x + L * np.cos(th), y + L * np.sin(th)
        elif turn == "L":
            pts.append(np.column_stack([x + R * (np.sin(th + u / R) - np.sin(th)),
                                        y - R * (np.cos(th + u / R) - np.cos(th))]))
            x, y = x + R * (np.sin(th + L / R) - np.sin(th)), y - R * (np.cos(th + L / R) - np.cos(th))
            th += L / R
        else:
            pts.append(np.column_stack([x - R * (np.sin(th - u / R) - np.sin(th)),
                                        y + R * (np.cos(th - u / R) - np.cos(th))]))
            x, y = x - R * (np.sin(th - L / R) - np.sin(th)), y + R * (np.cos(th - L / R) - np.cos(th))
            th -= L / R
        s0 += L
    pts.append([[x, y]])                       # the exact end point
    return np.vstack(pts), np.append(grid, total), th
