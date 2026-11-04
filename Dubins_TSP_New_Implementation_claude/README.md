# Dubins TSP with multiple mobile sinks — new implementation

An independent implementation of the method in `Dubins_TSP.pdf` (repository
root): several mobile sinks, modelled as Dubins vehicles, leave a base station,
collect data from every sensor and return, avoiding obstacles and each other.
The code is plain Python functions working on NumPy arrays and dicts.

Nothing outside this folder is used or changed. The only exception is that one
test imports `dubins.py` from the repository root, read-only, as a reference.

## Run

```bash
cd Dubins_TSP_New_Implementation_claude
pip install -r requirements.txt
python planner.py --seed 1               # summary + output/seed1.png
python planner.py --seed 1 --budget 100  # another mission budget B (seconds)
python planner.py --seed 1 --gif         # also output/seed1.gif (slow)
python planner.py --seed 1 --resolver simple   # "wait your turn" instead of the game; output/seed1_simple.png
python tests/run_tests.py                # all tests, about 2 minutes
```

To change any parameter, edit `config.py` or pass it to
`default_params(...)`, e.g. `default_params(B=100, wheelbase=3.0)`.

## Files and the paper

| File | Paper | What it does |
|---|---|---|
| `config.py` | 0.2–0.7 | All parameters. R = wheelbase / tan(δ_max) (eq. 7), r_safe = w/2 + δ_margin (eq. 5). |
| `scenario.py` | 0.2, 0.4 | Seeded random map: disjoint polygon obstacles, sensors with dwell times τᵢ, many of them close to the obstacles and around their concave corners. |
| `geodesic.py` | eq. 9, 0.9 step 1, 0.11 step 2 | Visibility graph around obstacles inflated by r_safe + R, then Dijkstra: geodesic distance and path between every pair of sensors and the base. |
| `dubins_path.py` | 0.1, 0.12–0.13 | Shortest Dubins path between two poses: all six words, keep the shortest. |
| `trajectory.py` | 0.5, eq. 17, eq. 19 | Tour → waypoints → headings → Dubins legs → positions every dt seconds. T_k = L_k/v + Σ τᵢ. |
| `ga_tour.py` | 0.8 | Radially unimodal GA: initial tours with one radial peak; fitness = L_euclid + m(π)·π·R. |
| `partition.py` | 0.9 | Geodesic spectral bisection with the min–max workload threshold. |
| `conflict_game.py` | eq. 20, 0.10 | Conflict detection and the GO / WAIT / DETOUR best-response game (from `theory_claude/corridor_game_gt.py`). |
| `simple_resolver.py` | eq. 20 | The simplest alternative to the game, no game theory: sinks take turns, longest tour first; each stops just long enough to clear a meeting. |
| `planner.py` | 0.6, 0.11 | Steps 1–12, the checks of Section 0.7, the objectives K, T_max, L_total, and the command line. |
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
- Dubins legs are the shortest for their end poses.
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
| 2 | 102 | 61 / 16 | 16 | 74.2 | 2880 | 2.10 | 16 / 3 / 2 | 0.67 |
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
- **Separation between sinks (eq. 20):** 151 conflicting pairs (376.0 s of risk in total) before the game, 5 (4.0 s) after the game alone, and 3 (3.0 s) after the Step 12 repair. 8 of 10 maps end with no conflict. On maps 2 and 5 conflicts remain, with closest approaches of 0.67 m and 0.28 m against d_safe = 3 m: real near-misses that the method reports but does not remove.
- **Strategies chosen:** GO 99, WAIT 90, DETOUR 4, over 42 games.
- **Run time:** 25–66 s per map on this machine (map generation with the reachability check, route planning, conflict game).
- Reproduce K, T_max, L_total and the final conflicts of a row with `python planner.py --seed N --no-plot`.

## Conflict game or simple rule? (measured on the same 10 maps)

`simple_resolver.py` uses no game theory: sinks take turns, longest tour first, and each one stops just long enough to clear a meeting with the sinks that went before it. For this comparison both methods start from the same planned routes on seeds 1–10 and are scored by the same check (`conflict_game.summary`).

| | Conflict game | Simple rule |
|---|---|---|
| Code | 276 lines | 75 lines |
| **Resolver alone** (151 conflicting pairs, 376 s of risk before): pairs left | 5 | 10 |
| Risk left | 4.0 s | 22.5 s |
| Maps with no conflict | 7 of 10 | 2 of 10 |
| **With the Step 12 repair** (the default): pairs left | 3 | 1 |
| Risk left | 3.0 s | 5.5 s |
| Maps with no conflict | 8 of 10 | 9 of 10 |
| Worst closest approach (d_safe = 3 m) | 0.28 m | 0.90 m |
| Mission time added, all sinks on all maps | 283.5 s | 269.5 s |
| Extra distance, all maps | 4.9 m | 0 m |

Coverage, budget, obstacle clearance and turning radius held on every map with both methods. On its own the game clearly removes more conflicts. With the repair step the two are close, and neither is better on every measure; 10 maps are too few to rank them. Compare any map with `python planner.py --seed N --no-plot` and the same command with `--resolver simple`.

## Differences from the original code in the repository root

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
