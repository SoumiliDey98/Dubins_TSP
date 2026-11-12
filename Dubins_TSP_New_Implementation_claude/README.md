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
python compare_dubins.py --figures       # the same, and also saves output/compare_*.png
python ga_evolution.py --seed 1          # how the GA improves: table of its runs, output/ga_seed1_run5_*.png and *.gif
python ga_evolution.py --seed 1 --run 0  # the same figures for another GA run of the table
python pipeline_gifs.py --seed 1         # four GIFs of the whole algorithm improving the plan, output/seed1_*.gif
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
| `ga_tour.py` | 0.8 | Radially unimodal GA: initial tours with one radial peak; fitness = L_euclid + m(π)·π·R. Can record every generation (`history`) without changing the result. |
| `ga_evolution.py` | 0.8 | Plans Steps 1–7 as `planner.py` does, records every GA run, prints a table of them, and draws one run's convergence graph, fitness split and three GIFs (see "How the GA improves" below). |
| `pipeline_gifs.py` | 0.11 | Plans a map as `planner.py` does and makes four GIFs of the whole algorithm improving the plan (see "Animations of the whole algorithm" below). |
| `animations.py` | 0.8–0.11 | The drawing for `pipeline_gifs.py`. |
| `partition.py` | 0.9 | Geodesic spectral bisection with the min–max workload threshold. |
| `conflict_game.py` | eq. 20, 0.10, 0.10.1 | Conflict detection and the GO / WAIT / DETOUR best-response game (from `theory_claude/corridor_game_gt.py`). |
| `simple_resolver.py` | eq. 20, 0.10.2 | The simplest alternative to the game, no game theory: sinks take turns, longest tour first; each stops just long enough to clear a meeting. |
| `planner.py` | 0.6, 0.11 | Steps 1–12, the checks of Section 0.7, the objectives K, T_max, L_total, and the command line. |
| `compare_dubins.py` | 0.13 | Runs both Dubins methods on the same maps and prints the numbers of the comparison below; `--figures` also saves its five figures. |
| `plot.py` | — | PNG of the routes; optional GIF; the comparison figures; the GA figures and GIF. |
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

Coverage, budget, obstacle clearance and turning radius held on every map with both methods. On its own the game clearly removes more conflicts. With the repair step the two are close, and neither is better on every measure; 10 maps are too few to rank them, so the comparison below uses 30. Compare any map with `python planner.py --seed N --no-plot` and the same command with `--resolver simple`.

### The same comparison on 30 maps

Seeds 1–30, measured on 2026-11-06 with scripts that are not in the repository. Seeds 1–10 reproduce the table above. `output/compare_resolvers.pdf` is the illustrated report on this comparison, with figures for everything below. Both methods start from the same planned routes and are scored by the same check.

| | Conflict game | Simple rule |
|---|---|---|
| **Resolver alone** (506 conflicting pairs, 1201.0 s of risk before): pairs left | 17 | 50 |
| Risk left | 13.0 s | 103.5 s |
| Maps with no conflict | 17 of 30 | 7 of 30 |
| Maps with less risk left (5 tie, Wilcoxon p < 0.001) | 22 | 3 |
| **With the Step 12 repair** (the default): pairs left | 8 | 12 |
| Risk left | 6.0 s | 20.0 s |
| Maps with no conflict | 22 of 30 | 20 of 30 |
| Maps with less risk left (20 tie, p = 0.06) | 8 | 2 |
| Worst closest approach (d_safe = 3 m) | 0.28 m | 0.04 m |
| Maps closer than 1 m | 3 | 5 |
| Mean T_max | 76.45 s | 74.63 s |
| Mission time added, all sinks on all maps | 991.5 s | 795.0 s |
| Extra distance, all maps | 10.7 m | 0 m |
| CPU time for Steps 8–12, per map | 2.9 s | 5.0 s |

Both methods give K = 470 and L_total = 88,044–88,045 m. Coverage, budget and turning radius held on every map with both.

- **Which is better.** The game is better on eq. 20 and the simple rule on T_max. On the 18 maps where both end with no conflict, the simple rule has the lower T_max on 10, the game on 1, and 7 are equal, so the case for the game rests on eq. 20 being a constraint.
- **Seeds 1–10 flatter both methods.** Each ends with no conflict on 9 of those 10 maps, but only 13 of 20 (game) and 11 of 20 (simple rule) on seeds 11–30.
- **Heavier traffic.** With the same routes and a larger d_safe, resolver alone, the simple rule collapses just above the default:

| d_safe (m) | 2 | 2.5 | 3 | 3.25 | 3.5 | 4 | 5 | 6 | 8 |
|---|---|---|---|---|---|---|---|---|---|
| Risk removed, game | 99 % | 99 % | 99 % | 99 % | 99 % | 97 % | 96 % | 82 % | 65 % |
| Risk removed, simple rule | 99 % | 97 % | 91 % | 33 % | 23 % | 20 % | 18 % | 20 % | 20 % |

- **Why the simple rule gives up.** A stop is accepted only if the whole window is clear, or no stop is made at all (219 give-ups, with a median of 25 s of budget left; B = 120 s changes nothing). Only the later sink in the order may yield, even when it is in front of the other (18 of 26 same-way conflicts left). Its five looks are stricter than the final check near the base: 520 steps in the plans are flagged only by them, 509 of those in the first 5 s, and they missed no conflict. Near the base the earliest stop is 2.5 m out, and a sink leaving on the same heading is 3.125 m from it at its first look outside the docking zone, which causes the collapse above that d_safe.
- **Why the game leaves conflicts.** `exposure` counts risk to the end of the mission, so a stop is charged for later conflicts that later games could solve. On seed 5, sink 5 keeps GO (3.5 s in total) over a 6.0 s WAIT (4.0 s in total, none of it in the next 6 s), which leaves a 0.28 m near-miss. Timing changes also cannot fix a head-on meeting when no DETOUR fits (seed 24). DETOUR is a small part of the game's lead: chosen in 11 of 613 decisions, and removing it leaves 15.5 s.

**What-if variants, resolver alone.** These were patched in memory for the measurement and are not in the code.

| Variant | Risk left, d_safe = 3 m | Maps with no conflict | Mission time added | Risk left, d_safe = 4 m |
|---|---|---|---|---|
| Simple rule as written | 103.5 s | 7 | 817.0 s | 1295.5 s |
| Simple rule, shortest tour first | 99.0 s | 7 | 887.0 s | not run |
| Simple rule, exact closest-distance test | 63.0 s | 10 | 809.5 s | 111.5 s |
| Simple rule, accepting the lowest-risk stop | 33.5 s | 10 | 914.0 s | 66.5 s |
| Simple rule, both fixes | 31.0 s | 10 | 840.5 s | 62.0 s |
| Game without DETOUR | 15.5 s | 16 | 985.0 s | not run |
| Game as written | 13.0 s | 17 | 971.5 s | 43.5 s |
| Game, `exposure` limited to the next 10 s | 6.5 s | 24 | 985.5 s | 21.0 s |

Through the full Steps 8–12, the game with the 10 s limit leaves 3.0 s of risk (4 pairs), ends with no conflict on 27 of 30 maps, has a worst closest approach of 2.43 m and a mean T_max of 75.65 s. It is better than the game as written on 7 maps and worse on 2. Coverage and budget held, and curvature × R reached 1.094 (limit 1.15). It has not been through the tests.

## Shortest of six or the rule of Section 0.13? (measured on the same 10 maps)

`dubins_method="shortest"` is the default. `dubins_method="pdf"` picks the word from the signs of cp1 and cp2 and from S_tan, as Section 0.13 describes; it gives a feasible path, which is not always the shortest. Each method plans its own routes on seeds 1–10, with the conflict game. All values are totals over the 10 maps unless stated. `python compare_dubins.py` prints every number in this section, plus a row per map; add `--seeds N` for chosen maps.

| | Shortest of six | Rule of Section 0.13 (feasible) |
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

**How the rule does by leg length.** With `--figures`, the script also prints this table, from 5000 random poses per band. They are random, so this is not the planner's cost: the heading search avoids many of these poses. Mean extra length includes fallback legs, which have none.

| Distance between the two points | Rule's word is the shortest | Exists but is longer | Does not exist (fallback) | Mean extra length |
|---|---|---|---|---|
| 0-10 m | 8.6 % | 43.1 % | 48.3 % | +30.3 % |
| 10-20 m | 39.0 % | 35.6 % | 25.4 % | +39.1 % |
| 20-40 m | 84.9 % | 15.1 % | 0.0 % | +10.1 % |
| 40-100 m | 95.2 % | 4.8 % | 0.0 % | +1.2 % |

**Speed.** Measured once with nothing else running, on seeds 1-4, with the shortest-of-six run first on each map; `compare_dubins.py` does not print these.

| | Shortest of six | Rule of Section 0.13 (feasible) |
|---|---|---|
| One path | 251 µs | 32 µs |
| Heading table for one leg (289 heading pairs) | 0.59 ms | 1.79 ms |
| Whole planner, mean per map | 32.8 s | 27.4 s |

The rule's saving on single paths most likely outweighs its slower table, because it needs all six lengths only when its word does not exist.

**Figures** (`python compare_dubins.py --figures`, saved in `output/`):

- `compare_example_legs.png`: both paths on four example legs.
- `compare_extra_length.png`: extra total length on each map.
- `compare_same_tour.png`: the tour that changes most on the first seed, drawn with both methods.
- `compare_leg_types.png`: how the rule's word does on the legs of each final plan.
- `compare_by_distance.png`: the table above, by leg length.

## How the GA improves, generation by generation (seed 1)

`python ga_evolution.py --seed 1` plans Steps 1–7 exactly as `planner.py` does, with the same random numbers, and records every generation of every GA run. It prints a table of the runs and saves three figures for one run: by default the run whose best tour improved most (run 5 on seed 1); `--run N` picks another.

- `output/ga_seed1_run5_convergence.png`: the objective the GA minimises, Fitness(π) = straight-line length + m(π)·π·R (Section 0.8.1), for the best tour, with the population mean and the range from best to worst, per generation.
- `output/ga_seed1_run5_fitness_split.png`: the best tour's fitness split into straight-line length and peak penalty m(π)·π·R, and the number of peaks m(π) of the best tour and of the population on average.
- `output/ga_seed1_run5_evolution.gif`: the 50 tours of the population (grey) and the best tour (blue) on the map, one frame per generation, with both curves drawn alongside. Tours are drawn as straight lines between sensors, because that is what the fitness scores.
- `output/ga_seed1_run5_population.gif`: every tour of the population as a dot, straight-line length across and m(π) up, one frame per generation. Diagonal lines join tours of equal fitness: one more peak is worth π·R = 15.7 m of length, so a tour improves by moving left or down across them. The cloud of 50 dots shrinks to a few points.
- `output/ga_seed1_run5_radial.gif`: the distance from the base of each sensor in the order the best tour visits them, one frame per generation, with each peak marked. Generation 0 has one hill (radially unimodal, Section 0.8.2); by generation 100 run 5's best tour has five.

What it shows on seed 1 (default parameters, B = 80 s):

- **The best tour never gets worse, but it improves in steps.** The GA keeps the better half of each generation, so the best fitness cannot rise, but most generations leave it unchanged. Run 5 (20 sensors) improves in 16 of 100 generations, from 1357 m to 832 m (−38.7 %). Run 0 (all 98 sensors) improves in 12, from 4389 m to 4054 m (−7.6 %), and is still improving at generation 100.
- **The GA improves the large groups, which are split anyway.** Of the 29 GA runs, the 14 on groups that are later split (9–98 sensors) gain a median of 8.9 % over generation 0, at most 38.7 %.
- **The final sinks' tours mostly come straight from the initial population.** In 13 of the 15 runs that give the final sinks' tours (3–13 sensors), the GA never beats the best tour of generation 0, which the radially unimodal initialisation (Section 0.8.2) already contains. The other two gain 3.1 % and 10.2 %.
- **Radial unimodality is not kept on large groups.** A peak costs only π·R = 15.7 m, so the GA trades peaks for length: the best tour of run 0 ends with m(π) = 9, 461 m shorter, and that of run 5 with m(π) = 5, 588 m shorter. Of the 15 final sinks' tours, 14 keep m(π) = 1 and one has 2.
- **The population loses its variety.** Run 0 falls from 50 different tours to 2 by generation 50.
- **Recording changes nothing.** It makes no random draws, so the plan is the same: `planner.py --seed 1` still gives K = 15, T_max = 79.6 s, L_total = 2927 m. `test_ga_history_does_not_change_the_result` checks this for the GA, and `test_plan_routes_log_changes_nothing_and_follows_the_queue` for the planner.

## Animations of the whole algorithm (seed 1)

`python pipeline_gifs.py --seed 1` plans the map once, exactly as `planner.py` does, and makes four GIFs that each show the plan getting better (about 4 minutes in all; `--only` picks some). Every GIF uses one colour palette for all its frames, so a colour stays the same throughout. Red always means a problem (closer than d_safe, or T > B) and green means fixed (T ≤ B, no conflict), each with a text label.

- `output/seed1_conflicts.gif` (Steps 8–10, 18 s): two maps side by side, the 15 sinks moving as planned and after the conflict game. A red link joins two sinks closer than d_safe = 3 m outside the docking zone. Below, the closest distance between any two sinks over time, and above each map the time in conflict so far: 63.0 s summed over 16 pairs in the plan, 0.0 s after the game. Most of the planned conflicts happen in the first 10 s, as the sinks leave the base together.
- `output/seed1_splitting.gif` (Steps 1–7, 31 s): one step per GA run. A group's tour is drawn in black, then turns green (T ≤ B, kept as a sink) or red (T > B, split in two, with the halves outlined); groups still waiting are grey. Beside it, the tour time of every run so far against B on a log scale: one sink with all 98 sensors would need 1202 s, and after 29 GA runs every one of the 15 tours fits within 80 s.
- `output/seed1_sink8_refine.gif` (8 s): one sink's tour in three stages, each morphing into the next. Sink 8's 5 sensors in the GA's order are 121 m apart as straight lines (what the fitness scores), 128 m along the waypoints around the obstacles, and 177 m along the Dubins path it drives (+47 %), so T = 177 / 5 + 15.6 s of dwell = 51.0 s. The default sink is the one whose Dubins path is longest compared with its straight lines; `--sink` picks another.
- `output/seed1_pipeline.gif` (36 s): all of it in one GIF. The GA orders all sensors for one sink, then groups are split until every tour fits B, then the final Dubins routes, the sinks moving as planned with their conflicts, and the same after the conflict game. Score tiles show K, T_max, L_total and the time in conflict as they change. L_total rises from 2925.6 m planned to 2926.8 m after the game, because of the game's one DETOUR, and T_max stays at 79.6 s.

## Changes marked in `Dubins_TSP.pdf` (Section 0.13)

The PDF was not changed beyond what is listed here. No original line was removed: additions are printed in blue, and lines to remove are shaded red with a red note and a PDF comment. Page numbers are those printed at the foot of the PDF. Printed page n is PDF page n + 1 up to page 7, and PDF page n + 3 from page 8 on, because pages 7a and 7b are PDF pages 9 and 10. The notes in the PDF are labelled R1-R8; the additions carry no labels there, and A1-A15 below follow their reading order.

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

**Sections added on 2026-11-06**

Pages 7a and 7b sit between Algorithm 1 and Section 0.11. Section 0.10.1 is the theory of `conflict_game.py` and Section 0.10.2 is the theory of `simple_resolver.py`, in black like the rest of the paper. The blue legend under Section 0.13 covers that section only.

**Noticed but not marked in the PDF**

These are small wording points, not errors in the method.

- Section 0.13, page 9: `(x_ci, y_ci)` means the centre of circle `i`, while pages 11-12 use the same symbols for the cut-in point. One of them needs another name.
- Section 0.13, page 9: "the above two vectors" follows a list of three vectors.
- Section 0.12, page 8: the sentence beginning "However, the sensors near the BS may exhaust their energy" appears twice, and "senser" is a typo.

**Not checked**

Only Section 0.13 was compared with the code line by line. Where the code is known to differ from other sections, the "Choices where the paper is silent" table records it. Section 0.10.1 states the two differences for Section 0.10 (GO / WAIT / DETOUR in place of Left / Right / Detour, and corridors not being extracted); the original text of Section 0.10 is unchanged.

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

- Obstacle clearance (eq. 19) is not always met. On seed 22, outside the 10 maps of the main table, sink 13 passes 1.41 m from an obstacle against r_safe = 2 m in the plan, before any resolver runs. `planner.py` prints the value without flagging it. The cause has not been investigated.
- Sinks follow the planned timing exactly (open loop) and can stop and restart at once (needed for WAIT and for dwelling at sensors).
- Between frames, sinks are assumed to move in a straight line when conflicts are checked.
- Headings come from a discrete set (16 directions + bisector), not a continuous optimum.
- The map has no outer boundary, so routes may leave the 100 × 100 square, as in the original code.
- Run time is about 25–66 s per map of 90–110 sensors, mostly for building trajectories and checking that sensors near obstacles can be visited.
- Conflict results can depend on rounding at one exact limit. With v = 5 m/s and dt = 0.5 s, a sink leaving the base in a straight line is exactly 5.0 m from it after two frames, which equals `base_radius`, so a difference of about 1e-15 m can decide whether the docking-zone exemption applies. On map 2 this one check changed the outcome from 2 pairs left (closest 0.67 m) to none (closest 3.01 m) when a rewrite of `dubins_path.py` moved points by about 1e-13 m. Treat the conflict result of a single map as sensitive.
