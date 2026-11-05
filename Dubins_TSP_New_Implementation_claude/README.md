# Dubins TSP with multiple mobile sinks — new implementation

An independent implementation of the method in `Dubins_TSP.pdf` (repository
root): several mobile sinks, modelled as Dubins vehicles, leave a base station,
collect data from every sensor and return, avoiding obstacles and each other.
The code is plain Python functions working on NumPy arrays and dicts.

Nothing outside this folder is used or changed. The only exception is that one
test imports `Dubins_TSP_Antigravity/dubins.py`, read-only, as a reference.

## Run

```bash
cd Dubins_TSP_New_Implementation_claude
pip install -r requirements.txt
python planner.py --seed 1               # summary + output/seed1.png
python planner.py --seed 1 --budget 100  # another mission budget B (seconds)
python planner.py --seed 1 --gif         # also output/seed1.gif (slow)
python planner.py --seed 1 --resolver simple   # "wait your turn" instead of the game; output/seed1_simple.png
python planner.py --seed 1 --dubins pdf  # turn-type rule of Section 0.13; output/seed1_pdf.png
python compare_dubins.py                 # both Dubins methods on seeds 1–10, 10–13 minutes on 2 cores
python tests/run_tests.py                # all tests, about 5 minutes
```

To change any parameter, edit `config.py` or pass it to
`default_params(...)`, e.g. `default_params(B=100, wheelbase=3.0)`.

## Files and the paper

| File | Paper | What it does |
|---|---|---|
| `config.py` | 0.2–0.7 | All parameters. R = wheelbase / tan(δ_max) (eq. 7), r_safe = w/2 + δ_margin (eq. 5). |
| `scenario.py` | 0.2, 0.4 | Seeded random map: disjoint polygon obstacles, sensors with dwell times τᵢ, many of them close to the obstacles and around their concave corners. |
| `geodesic.py` | eq. 9, 0.9 step 1, 0.11 step 2 | Visibility graph around obstacles inflated by r_safe + R, then Dijkstra: geodesic distance and path between every pair of sensors and the base. |
| `dubins_path.py` | 0.1, 0.12–0.13 | Dubins path between two poses, with the names and steps of the DubinsPath notebook program (`turnType`, `cp1`, `Stan`, `psiD`, …). A "word" in this README is a `turnType` in the code. By default all six are computed and the shortest is kept, which differs from Section 0.13; `dubins_method="pdf"` uses the rule of Section 0.13 (see the comparison below). The opening comment of the file lists the limitations of the notebook program and how they are corrected. |
| `trajectory.py` | 0.5, eq. 17, eq. 19 | Tour → waypoints → headings → Dubins legs → positions every dt seconds. T_k = L_k/v + Σ τᵢ. |
| `ga_tour.py` | 0.8 | Radially unimodal GA: initial tours with one radial peak; fitness = L_euclid + m(π)·π·R. |
| `partition.py` | 0.9 | Geodesic spectral bisection with the min–max workload threshold. |
| `conflict_game.py` | eq. 20, 0.10 | Conflict detection and the GO / WAIT / DETOUR best-response game (from `theory_claude/corridor_game_gt.py`). |
| `simple_resolver.py` | eq. 20 | The simplest alternative to the game, no game theory: sinks take turns, longest tour first; each stops just long enough to clear a meeting. |
| `planner.py` | 0.6, 0.11 | Steps 1–12, the checks of Section 0.7, the objectives K, T_max, L_total, and the command line. |
| `compare_dubins.py` | 0.13 | Runs both Dubins methods on the same maps and prints the numbers of the comparison below. |
| `plot.py` | — | PNG of the routes; optional GIF. |
| `tests/run_tests.py` | — | Unit tests per module plus end-to-end runs. |

## How it works (Section 0.11)

1. **Steps 1–7.** Start with one sink that has all sensors. For each group of sensors:
   - the GA orders the sensors (Step 3);
   - the order is turned into a real Dubins trajectory around the obstacles, and its time T = L/v + Σ τ is computed (Steps 2, 4);
   - if T ≤ B the tour is accepted; otherwise the group is split in two by the spectral bisection and both halves are planned again (Steps 5–7).
2. **Steps 8–10.** All sinks start at t = 0. The earliest moment two sinks come closer than d_safe starts a game among the sinks involved. They choose GO, WAIT or DETOUR by best responses until nobody can improve (a pure Nash equilibrium). The equilibrium is applied, and the search continues after that moment. With `--resolver simple`, `simple_resolver.py` does this step instead; the rest of the algorithm is the same.
3. **Step 11.** Every constraint is measured independently:
   - coverage (eq. 15–16), budget (eq. 18), obstacle clearance (eq. 19);
   - turning radius;
   - separation between sinks (eq. 20);
   - K ≤ K_max.
4. **Step 12.** If conflicts remain, each sink involved tries its reversed visiting order and a fresh GA run. It keeps whichever has the least risk against the others, within the budget. Steps 8–11 are then repeated, at most `repair_rounds` times, and the round with the least remaining risk is kept.

## Choices where the paper is silent (defaults)

| Topic | Choice |
|---|---|
| Sensor layout | 60 % of the sensors are placed 3–7 m from an obstacle. Half of the placement attempts aim at the concave (inner) corners: the inside of an L, the mouth of a C, the notches of a cross. Those areas are small, so fewer sensors end up there (4–17 per map in the results below). Such a sensor is kept only if the round trip base → sensor → base keeps r_safe from all obstacles. The other sensors are spread out, as in the original code. Set `sensor_layout="uniform"` for the old layout. |
| Free space for route finding | Lines between obstacle corners stay outside the obstacles inflated by r_plan = r_safe + R, which leaves room to turn. A line that starts or ends at a sensor only has to stay r_safe + 0.5 m away, so sensors close to obstacles can be reached. Clearance of the final path is checked against r_safe (eq. 19). |
| Corners | A corner closer than R to its neighbouring waypoints is skipped, to avoid tiny loops, unless the shortcut would pass closer than r_safe + 0.5 m to an obstacle. |
| Headings θ (Section 0.5) | At each waypoint, chosen from 16 directions plus the bisector of the turn, by dynamic programming. The choice minimises the tour's Dubins length, and each leg must keep r_safe from the obstacles. If no such choice exists, the bisectors are used and the check reports the problem. |
| Open cases of the Section 0.13 rule (`dubins_method="pdf"`) | If the chosen word does not exist (LSR or RSL with S_tan < 2R), the shortest existing word is used. cp2 = 0 counts as a left turn, as in `findCSCTurnType` of the original `dubins.py`. Geometry is exact in both modes: the Δψ stepping of Section 0.13.1 is not used, so only the choice of word follows the paper. For LSR and RSL the cut-out angle is ψ_L ± sin⁻¹(2R/S_tan), not the paper's tan⁻¹(2R/S_tan), because S_tan is the distance between the circle centres and not the length of the straight part. |
| GA fitness | Exactly as in the paper: straight-line length + m(π)·π·R. The radial distance rᵢ is geodesic (eq. 9). |
| σ in the affinity matrix | σ = max(mean geodesic nearest-neighbour spacing, R). Gaps smaller than the turning radius cannot shorten a Dubins tour. |
| Workload T̂ | A sweep tour in polar-angle order with geodesic legs, divided by v, plus Σ τ. |
| Conflict game strategies | GO / WAIT / DETOUR, as in `theory_claude/corridor_game_gt.py`, instead of the paper's Left / Right / Detour. A strategy that would exceed B is rejected (0.10 step 6), so WAIT only uses the time left in the budget. |
| Base station | Two sinks that are both within 5 m of the base at their closest moment are exempt from eq. (20), since everyone starts there at t = 0. Eq. (20) is only applied while both sinks are on their mission, t ≤ min(T_k, T_l), as the paper states. |
| Corridors (0.10 steps 1–3) | Not extracted. Conflicts are found between whole timed trajectories (Step 8), so they are caught in corridors and in open space alike. |

## What is guaranteed, and what is only measured

**By construction (and tested):**
- Every sensor is served exactly once.
- T_k = L_k/v + Σ τᵢ exactly.
- Every accepted tour has T_k ≤ B, and the game never pushes a sink over B. If this cannot hold, it is reported: a sensor too far to reach alone, or more than K_max sinks needed.
- Dubins legs are the shortest for their end poses (with the default `dubins_method="shortest"`; the rule of Section 0.13 does not guarantee this).
- Headings are the best ones within the candidate set.
- Every leg chosen by the heading search, and every DETOUR, keeps r_safe from the obstacles. The bisector fallback is the one exception, and it is reported.
- The game is an exact potential game, so best response always stops at a pure Nash equilibrium (proof in `theory_claude/Corridor_Game_Theory.pdf`, numerical check in the tests).

**Not guaranteed:**
- Separation between sinks (eq. 20): the game removes most conflicts but not always all of them.
- Optimal K, T_max or L_total: the GA and the bisection are heuristics, as in the paper.

## Measured results (10 random maps, default parameters)

These numbers come from running the planner on seeds 1–10 with the default
sensor layout (sensors close to the obstacles). They describe these maps; they
are not guarantees.

| Map (seed) | Sensors | Near obstacles / near a concave corner | K | T_max (s) | L_total (m) | Obstacle clearance (m) | Conflicting pairs: before / after game / after repair | Closest approach (m) |
|---|---|---|---|---|---|---|---|---|
| 1 | 98 | 59 / 9 | 15 | 79.6 | 2927 | 2.13 | 16 / 0 / 0 | 3.16 |
| 2 | 102 | 61 / 16 | 16 | 68.7 | 2869 | 2.10 | 16 / 1 / 0 | 3.01 |
| 3 | 103 | 62 / 4 | 16 | 68.3 | 2842 | 2.02 | 16 / 0 / 0 | 3.11 |
| 4 | 109 | 65 / 14 | 16 | 72.4 | 3043 | 2.13 | 19 / 0 / 0 | 3.02 |
| 5 | 94 | 56 / 7 | 14 | 79.7 | 2853 | 2.06 | 16 / 1 / 1 | 0.28 |
| 6 | 101 | 61 / 8 | 16 | 73.9 | 3027 | 2.08 | 13 / 0 / 0 | 3.26 |
| 7 | 107 | 64 / 17 | 15 | 79.1 | 2579 | 2.06 | 13 / 0 / 0 | 3.03 |
| 8 | 99 | 59 / 13 | 16 | 79.8 | 3038 | 2.05 | 12 / 0 / 0 | 3.09 |
| 9 | 93 | 56 / 10 | 15 | 75.5 | 2720 | 2.05 | 15 / 1 / 0 | 3.13 |
| 10 | 109 | 65 / 7 | 16 | 78.3 | 2917 | 2.06 | 15 / 0 / 0 | 3.12 |

"Near obstacles" counts the sensors placed 3–7 m from an obstacle. "Near a concave corner" counts the sensors within 8 m of an inner corner. The median distance from a sensor to the nearest obstacle is 6.0–6.5 m per map; in the "uniform" layout every sensor is at least 7.5 m away.

- **Checked on every map:** every sensor served once, every T_k ≤ B = 80 s, obstacle clearance ≥ r_safe = 2 m (lowest 2.02 m), curvature ≤ 1.014/R (Dubins radius respected), no problems reported.
- **Separation between sinks (eq. 20):** 151 conflicting pairs (375.5 s of risk in total) before the game, 3 (2.5 s) after the game alone, and 1 (1.5 s) after the Step 12 repair. 9 of 10 maps end with no conflict. On map 5 a conflict remains, with a closest approach of 0.28 m against d_safe = 3 m: a real near-miss that the method reports but does not remove. The result on map 2 depends on rounding at one exact limit (see Limitations), so it does not show that the method always clears that map.
- **Strategies chosen:** GO 95, WAIT 91, DETOUR 4, over 40 games.
- **Run time:** 25–66 s per map on this machine (map generation with the reachability check, route planning, conflict game).
- Reproduce K, T_max, L_total and the final conflicts of a row with `python planner.py --seed N --no-plot`.

## Conflict game or simple rule? (measured on the same 10 maps)

`simple_resolver.py` uses no game theory: sinks take turns, longest tour first, and each one stops just long enough to clear a meeting with the sinks that went before it. For this comparison both methods start from the same planned routes on seeds 1–10 and are scored by the same check (`conflict_game.summary`).

| | Conflict game | Simple rule |
|---|---|---|
| Code | 278 lines | 75 lines |
| **Resolver alone** (151 conflicting pairs, 375.5 s of risk before): pairs left | 3 | 10 |
| Risk left | 2.5 s | 22.5 s |
| Maps with no conflict | 7 of 10 | 2 of 10 |
| **With the Step 12 repair** (the default): pairs left | 1 | 1 |
| Risk left | 1.5 s | 5.5 s |
| Maps with no conflict | 9 of 10 | 9 of 10 |
| Worst closest approach (d_safe = 3 m) | 0.28 m | 0.90 m |
| Mission time added, all sinks on all maps | 276.5 s | 269.5 s |
| Extra distance, all maps | 4.9 m | 0 m |

Coverage, budget, obstacle clearance and turning radius held on every map with both methods. On its own the game clearly removes more conflicts. With the repair step the two are close, and neither is better on every measure; 10 maps are too few to rank them. Compare any map with `python planner.py --seed N --no-plot` and the same command with `--resolver simple`.

## Shortest of six or the rule of Section 0.13? (measured on the same 10 maps)

`dubins_method="shortest"` is the default. `dubins_method="pdf"` picks the word from the signs of cp1 and cp2 and from S_tan, as Section 0.13 describes. Each method plans its own routes on seeds 1–10, with the conflict game. All values are totals over the 10 maps unless stated. `python compare_dubins.py` prints every number in this section, plus a row per map; add `--seeds N` for chosen maps.

| | Shortest of six | Rule of Section 0.13 |
|---|---|---|
| Sinks K | 155 | 156 |
| L_total | 28816 m | 29506 m (+2.4 %, longer on every map) |
| Conflicting pairs left | 1 | 2 |
| Risk left | 1.5 s | 1.0 s |
| Maps with no conflict | 9 of 10 | 8 of 10 |
| Worst closest approach (d_safe = 3 m) | 0.28 m | 1.30 m |
| Lowest obstacle clearance (r_safe = 2 m) | 2.02 m | 2.01 m |
| Highest curvature × R | 1.014 | 1.019 |

- **Same visiting orders:** rebuilding the 155 tours of the default plan with the rule gives 28811 m → 29309 m (+1.7 %), and 2 of them then exceed B. This isolates the rule from differences in GA runs and splits.
- **Conflicts:** the differences are small and come from routes that differ, so they do not show that either method separates sinks better.
- Coverage, budget, obstacle clearance and turning radius held on every map with both methods.

**How the rule does on each leg.** This table classifies the rule's own word on the legs of each final plan. The default plan does not use the rule, so its column shows how the rule would have done there.

| Legs of the final plan | Default plan | Plan made with the rule |
|---|---|---|
| Rule's word is the shortest | 982 of 1505 (65.2 %) | 1442 of 1509 (95.6 %) |
| Rule's word exists but is longer | 498 of 1505 (33.1 %) | 6 of 1509 (0.4 %) |
| Rule's word does not exist (fallback) | 25 of 1505 (1.7 %) | 61 of 1509 (4.0 %) |

The heading search steers the plan towards poses where the rule's word is the shortest, so the cost appears as less favourable headings, not as bad individual legs. Fallback legs are the true shortest by construction, so the search has no reason to avoid them, and a different fallback would change the +2.4 %.

## Changes marked in `Dubins_TSP.pdf` (Section 0.13)

The PDF was not changed beyond what is listed here. No original line was removed: additions are printed in blue, and lines to remove are shaded red with a red note and a PDF comment. Page numbers are those printed at the foot of the marked-up PDF, which is 1 less than the PDF page. The notes in the PDF are labelled R1-R8; the additions carry no labels there, and A1-A15 below follow their reading order.

**Lines to remove**

| Mark | Page | Lines | Why | Replaced by |
|---|---|---|---|---|
| R1 | 10 | "If `2R <= S_tan < 3R`, then it is a CCC curve. Else it is a CSC curve" | Not true for LSR and RSL, which stay CSC curves in that range | A5 |
| R2 | 10 | `psi_L = (y_c2 - y_c1) / (x_c2 - x_c1)` | `psi_L` is an angle. This ratio is only its tangent, and it divides by zero when `x_c2 = x_c1` | A7 |
| R3 | 11 | LSR: `psi_d = psi_L + tan^-1(2R / S_tan)` | `S_tan` is the distance between the circle centres, not the length of the straight part. The angle is 8.1 degrees off at `S_tan = 3R` | A8 |
| R4 | 11 | RSL: `psi_d = psi_L - tan^-1(2R / S_tan)` | Same as R3 | A9 |
| R5 | 11 | "Let the angular increment be" and `delta_psi = 0.05` | The exact arcs need no increment | A11 |
| R6 | 11-12 | First arc: the rotation matrix with `i delta_psi`, "Initially," and its start values, "iterating until" with `abs(psi_T - psi_D) <= 0.1`, the heading update, `delta_S`, and the sum for `S_1` | Stepping can leave the heading up to 0.1 rad wrong where two parts meet, which makes a small corner and an approximate length. If the heading starts just past `psi_D`, an almost full circle is skipped, an error of about `2 pi R` | A11, A12 |
| R7 | 12 | Second arc: the single word "Initially," in the heading line, its start values, the rotation matrix with `i delta_psi`, the heading update, the iteration, and the sum for `S_2` | Same as R6 | A11, A13 |
| R8 | 13 | The two lines stepping by `3 u_hat`, "for", and `j = 1, ..., floor(d_co,ci / 3)` | The spacing of 3 applies to the straight part only, so points are unevenly spaced along the path | A14 |

**Lines added**

| Mark | Page | What it says |
|---|---|---|
| A1 | 9 | Legend under the section heading, explaining the colours. Delete it once the marked lines are gone |
| A2 | 9 | The rule does not always give the shortest path. The true shortest comes from computing `S_path` for all six turn types and keeping the smallest |
| A3 | 9 | Only the signs of `cp_1` and `cp_2` are used, so `u_hat_2` need not have unit length, which also avoids dividing by zero when the two points coincide |
| A4 | 10 | The table does not cover `cp_2 = 0`, which is treated like `cp_2 > 0` |
| A5 | 10 | Only RSR and LSL can become CCC curves. Every other case is a CSC curve |
| A6 | 10 | When each turn type exists: RSR and LSL for every `S_tan`, LSR and RSL only if `S_tan >= 2R`, RLR and LRL only if `S_tan <= 4R`. If the rule picks LSR or RSL with `S_tan < 2R`, the shortest turn type that exists is used. A CCC curve can exist up to `4R` and be the shortest above `3R`, but the rule tries it only for `2R <= S_tan < 3R` |
| A7 | 10 | `psi_L = atan2(y_c2 - y_c1, x_c2 - x_c1)` |
| A8 | 11 | LSR: `psi_d = psi_L + sin^-1(2R / S_tan)` |
| A9 | 11 | RSL: `psi_d = psi_L - sin^-1(2R / S_tan)` |
| A10 | 11 | Straight part: `d_co,ci = S_tan` for RSR and LSL, `d_co,ci = sqrt(S_tan^2 - 4R^2)` for LSR and RSL. `psi_D` is the same angle as `psi_d` |
| A11 | 11 | Exact arcs: `S_1 = R [sigma_1 (psi_D - psi_1) mod 2 pi]`, `S_2 = R [sigma_2 (psi_2 - psi_D) mod 2 pi]`, `alpha_1 = S_1 / R`, `alpha_2 = S_2 / R` |
| A12 | 11 | Cut-out point: `x_co = cos(alpha_1)(x_1 - x_c1) - sigma_1 sin(alpha_1)(y_1 - y_c1) + x_c1`, `y_co = sigma_1 sin(alpha_1)(x_1 - x_c1) + cos(alpha_1)(y_1 - y_c1) + y_c1` |
| A13 | 12 | Cut-in point: `x_ci = cos(alpha_2)(x_2 - x_c2) + sigma_2 sin(alpha_2)(y_2 - y_c2) + x_c2`, `y_ci = -sigma_2 sin(alpha_2)(x_2 - x_c2) + cos(alpha_2)(y_2 - y_c2) + y_c2` |
| A14 | 13 | Points on the path step by one spacing `delta_s`, with `j = 1, ..., floor(d_co,ci / delta_s)`. The arcs use the same spacing, an angle of `delta_s / R` per point |
| A15 | 13-14 | New Section 0.13.2, CCC Trajectory. It exists only if `S_tan <= 4R`. `theta = cos^-1(S_tan / 4R)`. Middle circle: `x_c3 = x_c1 + 2R cos(psi_L + sigma_1 theta)`, `y_c3 = y_c1 + 2R sin(psi_L + sigma_1 theta)`. Cut-out point: `x_co = x_c1 + R cos(psi_L + sigma_1 theta)`, `y_co = y_c1 + R sin(psi_L + sigma_1 theta)`. Cut-in point: `x_ci = x_c2 - R cos(psi_L - sigma_1 theta)`, `y_ci = y_c2 - R sin(psi_L - sigma_1 theta)`. Headings: `psi_co = psi_L + sigma_1 (theta + pi / 2)`, `psi_ci = psi_L - sigma_1 (theta + pi / 2)`. Arcs: `S_1 = R [sigma_1 (psi_co - psi_1) mod 2 pi]`, `S_3 = R (pi + 2 theta)`, `S_2 = R [sigma_2 (psi_2 - psi_ci) mod 2 pi]`. Total: `S_path = S_1 + S_3 + S_2` |

**Noticed but not marked in the PDF**

These are small wording points, not errors in the method.

- Section 0.13, page 9: `(x_ci, y_ci)` means the centre of circle `i`, while pages 11-12 use the same symbols for the cut-in point. One of them needs another name.
- Section 0.13, page 9: "the above two vectors" follows a list of three vectors.
- Section 0.12, page 8: the sentence beginning "However, the sensors near the BS may exhaust their energy" appears twice, and "senser" is a typo.

**Not checked**

Only Section 0.13 was compared with the code line by line. Where the code is known to differ from other sections, for example GO / WAIT / DETOUR in place of Left / Right / Detour, or corridors not being extracted, the README's "Choices where the paper is silent" table records it, but those differences are not marked in the PDF.

## Differences from the original code (now in `Dubins_TSP_Antigravity/`)

| Topic | Original code | This implementation |
|---|---|---|
| Distances | Straight-line placeholder for "geodesic" | Real obstacle-aware distances (visibility graph + Dijkstra) |
| Budget check | Straight-line length | Length of the actual Dubins trajectory (eq. 17) |
| Spectral split | σ = 1 m: almost no sensors count as neighbours, so the split is nearly random | σ from the neighbour spacing and R; the halves are spatially separate |
| Headings | Bisector rule, every sink starts facing east | Optimised over a discrete set, with obstacle clearance |
| Conflicts | Only in the animation, after planning | Steps 8–12 inside the planner, followed by checks |
| Timing | Frame index used as time, small drifts | One exact time line, t = s/v + dwell, sampled every dt |
| Failures | Silent straight-line fallbacks | Reported by the checks |

## Limitations

- Sinks follow the planned timing exactly (open loop) and can stop and restart at once (needed for WAIT and for dwelling at sensors).
- Between frames, sinks are assumed to move in a straight line when conflicts are checked.
- Headings come from a discrete set (16 directions + bisector), not a continuous optimum.
- The map has no outer boundary, so routes may leave the 100 × 100 square, as in the original code.
- Run time is about 25–66 s per map of 90–110 sensors, mostly for building trajectories and checking that sensors near obstacles can be visited.
- Conflict results can depend on rounding at one exact limit. With v = 5 m/s and dt = 0.5 s, a sink leaving the base in a straight line is exactly 5.0 m from it after two frames, which equals `base_radius`, so a difference of about 1e-15 m can decide whether the docking-zone exemption applies. On map 2 this one check changed the outcome from 2 pairs left (closest 0.67 m) to none (closest 3.01 m) when a rewrite of `dubins_path.py` moved points by about 1e-13 m. Treat the conflict result of a single map as sensitive.
