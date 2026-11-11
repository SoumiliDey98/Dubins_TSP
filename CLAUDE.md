# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Keeping this file current

The user asked for this file to be kept up to date as work proceeds. In the same turn as any change to behaviour, commands, decisions, measured results or open items, update the affected section and add a dated entry to the work log. Correct stale statements instead of appending contradictions. Full result tables belong in the implementation's README; this file points to them.

## Repository layout

Dubins TSP for wireless sensor networks: several mobile sinks, modelled as Dubins vehicles, leave a base station, collect data from every sensor and return, avoiding obstacles and each other.

- `Dubins_TSP.pdf`: the paper (Soumili Dey, May 2026) and the specification for the code. Sections 0.10.1 and 0.10.2, on inserted pages 7a and 7b, give the theory of `conflict_game.py` and `simple_resolver.py`. Section 0.13 is currently marked up: blue lines are proposed additions, and red shaded lines are for the user to remove (open item 1). The unmarked version is in git at commit `374a424`.
- `Dubins_TSP_New_Implementation_claude/`: the active implementation. Everything below refers to it unless stated. Its `README.md` has the file-to-paper mapping, the choices made where the paper is silent, what is guaranteed versus only measured, and all result tables.
- `theory_claude/`: theory PDFs for the conflict game and the simple resolver, plus `corridor_game_gt.py`, which `conflict_game.py` was adapted from. `Dubins_math.pdf` and `geodesic_math.pdf` state every formula of `dubins_path.py` and `geodesic.py`, in the notation and LaTeX style of Section 0.13. Each has its source, figure script and check script in a `*_src/` folder next to it. Change a document whenever its module changes.
- `research_notes/Dubins mobile sink literature review/`: the literature review of the draft's novelty. There are five topic notes from 2026-11-06 (closest prior art, conflict-resolution games, Dubins multi-vehicle routing, mobile-sink WSNs, the radially unimodal heuristic) and `follow_up_2026-11-09.md`. Every citation carries a tag saying how far it was read. Continue the review from the follow-up note's last two sections.
- `Dubins_TSP_Antigravity/`: a separate implementation with its own README and requirements. The active implementation does not use it, except that `test_dubins_matches_original_and_ends_at_target` imports its `dubins.py`, the original Dubins code, as a read-only reference. Do not move or change that file without updating the test.

## Commands

There is no build step, and no linter or formatter is configured. Run these from `Dubins_TSP_New_Implementation_claude/`:

```bash
pip install -r requirements.txt
python planner.py --seed 1 --no-plot           # plan, check, print a summary; writes nothing
python planner.py --seed 1                     # also writes output/seed1.png
python planner.py --seed 1 --budget 100        # another mission budget B [s]
python planner.py --seed 1 --gif               # also writes output/seed1.gif (slow)
python planner.py --seed 1 --resolver simple   # "wait your turn" instead of the conflict game
python planner.py --seed 1 --dubins pdf        # Section 0.13 turn-type rule instead of the shortest of six
python compare_dubins.py                       # both Dubins methods on seeds 1-10; prints the README comparison
python compare_dubins.py --seeds 1 --jobs 2    # same, on chosen maps
python compare_dubins.py --figures             # also saves output/compare_*.png; the first seed gives the same-tour figure
python tests/run_tests.py                      # whole suite

# One test: tests are plain functions and run_tests.py has no selector
python -c "import sys; sys.path.insert(0, 'tests'); import run_tests; run_tests.test_pdf_rule_picks_the_word_of_section_0_13()"
```

pytest is not in `requirements.txt` and not installed here. After `pip install pytest`, name the file explicitly, because it does not match `test_*.py`:

```bash
python -m pytest tests/run_tests.py -k <name>
python -m pytest tests/run_tests.py -k "not end_to_end and not same_seed"   # skips the full planner runs
```

- On the 2-core codespace the suite takes 4-5 minutes (seven full planner runs) and the 10-map `compare_dubins.py` 10-13 minutes, so run them in the background and not at the same time.
- New `test_*` functions in `run_tests.py` are picked up automatically.
- `output/seed1.png`, `seed1_simple.png`, `seed2.png`, `seed3.png` and `seed3_simple.png` are tracked in git, so plotting those runs dirties the tree. Use `--no-plot` for checks.
- There is no `.gitignore`: the user had it deleted. `__pycache__/` and GIFs are therefore not ignored, so stage files by name instead of using `git add -A`, and run Python with `PYTHONDONTWRITEBYTECODE=1` so that `__pycache__/` is not created.
- Building `theory_claude/Dubins_math.pdf` or `geodesic_math.pdf`: from its `*_src/` folder, run `python make_figures.py`, then `pdflatex` on the `.tex` file twice, then move the PDF up one folder and delete the `.aux`, `.log` and `.out` files. `python check_formulas.py` reproduces the checks that the text quotes (about 3 minutes for the geodesic one). TeX Live and `cm-super-minimal` (needed by matplotlib's LaTeX text) were installed from apt on 2026-11-11. A new codespace needs `sudo apt-get install -y --no-install-recommends texlive-latex-base texlive-latex-recommended texlive-fonts-recommended texlive-latex-extra cm-super-minimal`.
- Reading PDFs: `poppler-utils` lets the Read tool show PDF pages as images, which is the reliable way to read equations; extracted text garbles them. It was installed in this codespace on 2026-11-05, and a new codespace needs `sudo apt-get install -y poppler-utils`. Section 0.13 is on PDF pages 12-17.
- A codespace restart wipes `/tmp`, including the session scratchpad, so a long measurement kept only there can be lost. Write results per seed so a run can resume, and copy what must survive before leaving a run idle.
- Literature searches: OpenAlex (`api.openalex.org`) works without a key, but it shares a small daily budget per network address and returns 429 when hit quickly, so pause between calls. The arXiv API needs `https://export.arxiv.org`. IEEE Xplore refuses automated downloads, even of open-access papers, and the auto-mode classifier blocks retries with browser headers, so ask the user to download those. On 2026-11-09 the ETH Research Collection returned 429 and the TU Delft repository refused connections.
- PDF viewer in VS Code: `analytic-signal.preview-pdf`, installed with `~/.vscode-remote/bin/*/bin/remote-cli/code --install-extension analytic-signal.preview-pdf`. The plain `code` command does not work from the agent shell, and `tomoki1207.pdf` refuses to install on a Codespace. A new codespace needs this again.

## Architecture

`planner.run(seed, p)` drives Steps 1-12 of Section 0.11:

```
scenario.make(seed, p)    -> scen {xy, base, tau, obstacles, n_near}
geodesic.build(scen, p)   -> geo  {nodes, D, pred, n_terminals, obstacles}

plan_routes (Steps 1-7), a queue of sensor groups:
    ga_tour.optimize      -> visiting order
    trajectory.build      -> tour {order, frames, path, L, T}
    T > B                 -> partition.bisect, queue both halves

resolve_and_check (Steps 8-12), at most repair_rounds + 1 rounds:
    conflict_game.resolve or simple_resolver.resolve
    conflicts left        -> planner.reoptimize the sinks involved, try again
    keeps the round with the least remaining exposure

check                     -> measures every constraint from the final frames
```

What spans several modules:

- **Indices.** `geo["D"]` and `geodesic.path` use terminals: 0 is the base, i + 1 is sensor i. Tours, `tau` and `partition` use 0-based sensor ids, so `geo["D"][0, 1:]` is `r` and `geo["D"][1:, 1:]` is `Ds`.
- **Three length measures.** The GA's fitness is straight-line length + m(π)·π·R, with no Dubins paths; only its radial distance `r` is geodesic. `partition.workload` uses geodesic legs. Only the budget test `T <= B` uses real Dubins lengths, from `trajectory.build`.
- **Choosing the turn type.** `p["dubins_method"]` is read only in `trajectory.py`, at three call sites that must agree: the heading search's cost table (`dubins_path.returnPathDistances`), and `_leg_clear` and `build` (`dubins_path.returnDubinsPath`). Otherwise the heading search would optimise paths the sink does not drive. `findDubinsPath(..., "shortest")` is always the true minimum of the six turn types. `calculateCSCLengths` and `calculateCCCLengths` also work on arrays of headings, which keeps the heading search fast.
- **Frames are the shared timeline.** A trajectory is an (F, 2) array with one position every `dt` seconds, and all sinks start at t = 0. Resolvers see only frames: WAIT repeats frames, DETOUR replaces a window, and both update T and L. So `result["tours"]` is the plan before resolving, while `check` reads the final `result["frames"]`, `result["T"]` and `result["L"]`.
- **Interchangeable resolvers.** Both have the signature `resolve(trajs, T, L, geo, p) -> (trajs, T, L, records)`, and `conflict_game.summary` scores both. Eq. 20 applies only while both sinks are on their mission, and not when both are inside the docking zone (`base_radius`).
- **Game invariant.** A pair's danger is charged equally to both players, which makes it an exact potential game, so best response always stops. Changes to the costs in `conflict_game.play` must preserve this; `test_game_is_an_exact_potential_game` checks it. The proof is in `theory_claude/Corridor_Game_Theory.pdf`.
- **Obstacle margins.** Corner-to-corner lines stay outside obstacles inflated by `r_plan` = `r_safe` + R. Lines touching a sensor or the base only need `r_safe` + 0.5 m. Final clearance is measured against the original obstacles and needs `r_safe`.
- **Curvature.** Dense Dubins paths measure 1.000/R. `trajectory.max_curvature` fits circles through points at least 1 m apart, so sampled frames read slightly higher, and `curvature_tol` = 0.15 allows for that.
- **Parameters.** Everything is one dict from `config.default_params(**changes)`. `R`, `r_safe` and `r_plan` are recomputed after the changes are applied, so `default_params(R=...)` has no effect; change `wheelbase` or `delta_max` instead.
- **Determinism.** `run` uses `rng(seed)` for Steps 1-7 and `rng(seed + 1000)` for Steps 8-12, so the resolvers start those steps alike. An extra random draw or a geometry change shifts every result, and the README's tables were measured on seeds 1-10. Re-measure and update them when a change affects them; `compare_dubins.py` regenerates the Dubins comparison.

## Conventions

- The code is plain functions on NumPy arrays and dicts, with no classes. Docstrings cite paper sections and equations.
- `Dubins_TSP.pdf` is the specification, and the user checks the code against it. Citing a section is a claim that the code follows it, so read the section first. State any departure in the docstring and the README; the README's "Choices where the paper is silent" table records how gaps were filled.
- `dubins_path.py` uses the names and the order of steps of the user's DubinsPath notebook program (`dubinspath.py`, not in the repo), because the user knows that program. Keep those names. The file's opening comment lists that program's limitations in plain language; update it when behaviour changes. A "word" in the README and `compare_dubins.py` is a `turnType` in `dubins_path.py`. When the user says "feasible path", that means `pdf` mode, and "shortest path" means `shortest` mode.
- Comparison figures keep one colour per method: blue solid for shortest, orange dashed for feasible (`plot.py`). `compare_dubins.py` prints no timings, because they change with machine load.
- New options follow the pattern of `resolver` and `dubins_method`: a key in `config.py`, a flag in `planner.main`, a filename suffix so outputs do not overwrite each other, and a README section with measurements.

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

Pages 7a and 7b sit between Algorithm 1 and Section 0.11. Section 0.10.1 is the theory of `conflict_game.py`, and Section 0.10.2 is the theory of `simple_resolver.py`. They are short versions of `theory_claude/conflict_game_math.pdf` and `simple_resolver_math.pdf`, in black like the rest of the paper. The blue legend under Section 0.13 covers that section only. They use `F_k` for the risk against non-players, where the note and the code say `B_k`/`Bk`, because `B` is the budget in the paper. Change these sections whenever either resolver changes.

**Noticed but not marked in the PDF**

These are small wording points, not errors in the method.

- Section 0.13, page 9: `(x_ci, y_ci)` means the centre of circle `i`, while pages 11-12 use the same symbols for the cut-in point. One of them needs another name.
- Section 0.13, page 9: "the above two vectors" follows a list of three vectors.
- Section 0.12, page 8: the sentence beginning "However, the sensors near the BS may exhaust their energy" appears twice, and "senser" is a typo.

**Not checked**

Only Section 0.13 was compared with the code line by line. Where the code is known to differ from other sections, the README's "Choices where the paper is silent" table records it. Section 0.10.1 states the two differences for Section 0.10 (GO / WAIT / DETOUR in place of Left / Right / Detour, and corridors not being extracted); the original text of Section 0.10 is unchanged.

## Open items

1. **Section 0.13 of `Dubins_TSP.pdf` is marked up and waiting for the user**, who will remove the lines shaded red (R1-R8). Edit the PDF only when the user asks; the last request added Sections 0.10.1 and 0.10.2 and said not to delete anything. The full list is under "Changes marked in `Dubins_TSP.pdf`" above.
2. **Eq. 20 is not guaranteed.** Conflicts are reported, not always removed. On seeds 1-30 the game leaves conflicts on 8 maps (5, 15, 17, 19, 24, 26, 27, 28; worst 0.28 m on seed 5, against `d_safe` = 3 m) and the simple rule on 10 (worst 0.04 m). On seeds 1-10, `pdf` mode leaves them on seeds 5 and 9 (1.30 m and 1.88 m).
3. **No dedicated test for the detour fix below.** Only `test_end_to_end_with_pdf_rule` (seed 1) exercises it.
4. **The `pdf` cost depends on the fallback.** Fallback legs use the true shortest word, and the heading search uses them more than the default plan's poses would (4.0 % against 1.7 % of legs). A different fallback, which the paper would have to define, would change the +2.4 %. No other fallback has been measured.
5. **The docking-zone check sits on an exact limit.** `conflict_game.closest` and `simple_resolver.too_close` test `distance <= base_radius`. With `v` = 5 and `dt` = 0.5, a sink leaving the base in a straight line is exactly 5.0 m = `base_radius` from it after two frames, so rounding of about 1e-15 m can decide the exemption. This changed the result of seed 2 on 2026-11-05 (see the work log). Not fixed: a tolerance would change conflict detection, which is the user's decision and needs a re-measure.
6. **Some README numbers have no script in the repo.** The "after game alone" pairs, the strategy counts and the game-versus-simple table came from a throwaway script on 2026-11-05. It reproduced the previous README totals when given the old code on seed 2. The 30-map comparison and its what-if variants of 2026-11-06 also came from throwaway scripts in the session scratchpad.
7. **Seed 22 breaks eq. 19 in the plan.** Sink 13 passes 1.41 m from an obstacle against `r_safe` = 2 m, before any resolver runs. `planner.py` prints the value without flagging it, and `problems` stays empty. The cause has not been investigated.
8. **Decision for the user: limit the game's look-ahead?** `conflict_game.exposure` counts risk to the end of the mission. Limiting it to the next 10 s (20 frames), patched in memory, gave 27 of 30 clean maps and 3.0 s of risk after repair, against 22 and 6.0 s. It is not in the code and has not been through the tests. It would change the results in the README's tables.
9. **Decision for the user: fix `simple_resolver.py` if it stays as the paper's baseline?** Its all-or-nothing stop and its stricter five-look test near the base make it collapse just above `d_safe` = 3 m. With both fixed, patched in memory, it leaves 31.0 s against 103.5 s, resolver alone. Not in the code.
10. **Literature review: five papers unread, three claims to narrow, one modelling decision.**
    - Unread: Wang 2026 (CC BY, but IEEE Xplore refuses automated downloads), Zanardi 2021 and Fabiani & Grammatico 2020 (green open access, but their repositories refused this network), and Kumar & Mukherjee 2026 and Fu 2026 (closed). `follow_up_2026-11-09.md` gives links. If the user adds PDFs to the notes folder, they can be read.
    - Claims: Kumar & Mukherjee 2026 already minimises the number of sinks under a per-sink length limit with obstacles. Picard 2022 already resolves conflicts on pre-planned UAV trajectories with discrete per-conflict manoeuvres. The repair step is the MAPF-LNS2 loop.
    - Decision for the user, raised in `closest_prior_art.md`: Section 0.1 says sinks move forward at constant speed, yet WAIT and the dwell at sensors stop them. Every precedent found that resolves conflicts by stopping uses vehicles that can stop or hover. No code or PDF was changed.

## Work log

Newest first.

### 2026-11-11

- **Wrote `theory_claude/geodesic_math.pdf`**, at the user's request: every formula that `geodesic.py` follows, in the same style as `Dubins_math.pdf` (6 pages, 20 numbered equations, 3 figures drawn by `geodesic.py`, 3 tables). It covers the three margins, the shape of the inflated polygons, the visibility test, the graph, Dijkstra and the path, what `D` is, and how it relates to eqs. (4), (5), (9) and Section 0.9 step 1. The source is in `theory_claude/geodesic_math_src/`. No code or README was changed.
- **The inflated polygons, measured.** At a convex corner turning through `theta`, shapely's `buffer(r, quad_segs=2)` uses `max(1, floor(4 theta / pi + 1/2))` equal chords with their ends on the circle of radius `r`. So the polygon comes no closer than `r cos(3 pi / 16) = 0.8315 r` to the obstacle: at least 2.08 m for lines that touch a terminal (more than `r_safe`) and 5.82 m between corners. All 29047 corners whose arc lies on the boundary matched exactly (3000 random obstacles, both margins); at the other 6119, another arm of a C or cross covers part of the arc. A first version of this check skipped every corner and passed without testing anything; the fixed one is in the repo.
- **What `D` is, measured on seeds 1-10.** `D` is a metric and is never below the straight-line distance. It lies between the true obstacle-avoiding distances at margins 2.5 m and 7 m, as the document proves. 577 of 1015 sensors lie inside the 7 m inflation. 245 of the 1015 shortest paths from the base pass through another sensor, where both segments only need the 2.5 m margin, so a sensor near an obstacle acts as a tighter corner. That follows from the code as written and is stated in the document, not changed.
- **Wrote `theory_claude/Dubins_math.pdf`**, at the user's request: every formula that `dubins_path.py` follows, in the notation and LaTeX style of Section 0.13 (letter paper, 10 pt Computer Modern, 9 pages). It has 38 numbered equations, 3 figures drawn by `dubins_path.py` itself, and 4 tables (symbols with their code names, when each turn type exists, numerical tolerances, and Section 0.13 against the code). The source is in `theory_claude/Dubins_math_src/`.
- **Checked the formulas against the code.** On 20000 random pose pairs (R = 5 m; 74236 CSC and 18450 CCC paths), every closed form in the document agrees with the code within 1e-13 m. That includes `psi_D = psi_L + sigma_1 asin(2R / S_tan)`, the cut-in point found forward from the cut-out point against Section 0.13's backward rotation, the CCC centre and cut points, and `S_3 = R(pi + 2 theta)`.
- **One edge case.** When the two outer circles of a CCC path coincide (`S_tan = 0`), the code gives `S_3 = 0`, not the `2 pi R` of A15's `S_3 = R(pi + 2 theta)`. The path is still valid, and LSL or RSR is shorter there. The document states this. No code or PDF was changed.
- **Installed TeX Live** from apt. On 2026-11-06, downloading a compiler had been blocked; apt was allowed.
- **Commit status.** The user committed `Dubins_math.pdf`, its source and this file as `8905e4b`. `geodesic_math.pdf`, its source and the later changes to this file are not committed.

### 2026-11-09

- **Continued the literature review**, at the user's request, from the follow-up list in `closest_prior_art.md`. The results are in `research_notes/Dubins mobile sink literature review/follow_up_2026-11-09.md`. The five earlier notes were corrected in place, with each change marked "(2026-11-09)".
  - Citation trails, all screened in OpenAlex: Huang & Savkin (62 citing works), Fu 2026 (11), Gottlieb 2018 (11), Nayak & Rathinam 2023 (12), Wichmann & Korkmaz 2015 (51), Kumar & Mukherjee 2023 (4). The queries refused on 2026-11-06 were run.
  - New close papers: Kumar & Mukherjee 2026 (fewest sinks under a length limit, with obstacles, at abstract level); Picard 2022, AAMAS (postpone / elevate / skip per conflict on pre-planned UAV trajectories, in full text); Xu et al. 2023, T-ASE (non-holonomic fleet with task assignment and reactive collision avoidance, Python code public); and a 2026 Dubins survey.
  - Now read in full: MAPF-LNS2, Potential iLQR, Hansen & La Cour-Harbo 2016 (a GA with fitness from Dubins times). Raj et al. 2020's game picks rendezvous points, so it is not a deconfliction precedent.
  - The pipeline as a whole was still not found in any paper. Open item 10 added. No code, README or PDF was changed.
- **Commit status.** Not committed.

### 2026-11-06

- **Compared `conflict_game.py` and `simple_resolver.py` on seeds 1-30**, at the user's request, and published an illustrated report: https://claude.ai/artifact/AiPAeR9qeArao1uUxenVQN (private to the user until shared). The table and what-if variants are in the README under "The same comparison on 30 maps". The verdict is that the game is better: resolver alone it leaves 13.0 s of risk against 103.5 s, and after repair 6.0 s against 20.0 s, with 22 and 20 of 30 maps clean (p = 0.06 after repair, so that last lead is not conclusive). The simple rule is better on T_max (74.63 s against 76.45 s), and on the 18 maps where both end clean it has the lower T_max on 10. Seeds 1-10 reproduced the README's earlier table. No resolver code changed; every variant was patched in memory. New open items 7-9; open item 2 updated.
- **Saved the report as a PDF**, at the user's request: `Dubins_TSP_New_Implementation_claude/output/compare_resolvers.pdf` (A4, 12 pages, static copy of the online report; all three incidents shown, tables expanded, Figure 7 on seed 1). It was printed with a temporary headless browser in the scratchpad, so `planner.py` and `plot.py` do not produce it. While checking it, Figure 6's distance charts were found to plot whole frames only, which hid the true closest approach between frames; this was fixed in both the PDF and the online report.
- **A codespace restart lost the first run.** It wiped `/tmp`, including the scratchpad, while the session was idle partway through the work. The main run was repeated and every total measured twice matched. The stress totals at `d_safe` 2, 5, 6 and 8 m, the B sweep, and the give-up and leftover counts are kept from the first run.
- **Added the theory of both resolvers to `Dubins_TSP.pdf`**, at the user's request, as Sections 0.10.1 and 0.10.2 on inserted pages 7a and 7b (15 to 17 PDF pages). Formulas were checked against the code. The file is an incremental update: the old PDF's bytes are unchanged at the start of the new file, all 15 original pages are pixel-identical and have the same text, and the 11 annotations are kept. Downloading a LaTeX compiler was blocked by the permission system, so the pages were typeset with matplotlib's Computer Modern fonts and inserted by a stdlib script; both scripts are in the session scratchpad only. Extracted text of the new formulas comes out garbled, because of that font encoding; they display correctly.
- **Literature review of the draft's novelty.** Five notes in `research_notes/Dubins mobile sink literature review/`. It was cut short by the web-search limit and an exhausted OpenAlex allowance, and continued on 2026-11-09. This entry was added on 2026-11-09, because the review had not been logged here.
- **Commit status.** The README, this file, `Dubins_TSP.pdf` and `output/compare_resolvers.pdf` were not committed. The report PDF is a new untracked file. All of these, and the literature notes, were later committed as `d93954f` and `a4c709a`.

### 2026-11-05

- **Compared the shortest and feasible paths in figures**, at the user's request. `compare_dubins.py --figures` now saves five figures to `output/compare_*.png` and prints a table by leg length; the drawing is in `plot.py`. Over seeds 1-10 the results match the earlier run: feasible is 2.4 % longer in total, longer on every map (+1.4 % to +3.8 %), with 156 sinks against 155, and the three largest increases are on the three maps where the number of sinks changed. Measured alone on seeds 1-4, the whole planner takes 27.4 s per map with feasible against 32.8 s with shortest. Added two tests; all 28 pass. The new figures and code were uncommitted when this entry was written.
- **Checked the course slides behind Section 0.13** (nine slides from a Udemy lecture, "Dubins Curve: Part 1", shown by the user). The heading and axis definitions, circle centres, `S_tan`, `psi_L` with `atan2`, LSL/RSR geometry, the rotation matrix and `d <= 4R` are correct. Four things are wrong: LSR/RSL use `atan(2R/S_tan)` and `S_str = sqrt(S_tan^2 + 4R^2)`, although `S_tan` is the centre distance (correct: `asin(2R/S_tan)` and `sqrt(S_tan^2 - 4R^2)`); arc length `R|delta psi|` ignores turn direction and wrap-around (wrong on 24.8 % of arcs on 20000 random poses); the LSL box shows `sigma_2 = -1` (should be +1); and "No path exists" (40.6 % of those poses) really means this method has none, because a Dubins path always exists. The lecture's objective is a feasible path, not the shortest, whereas the PDF says shortest. No code or PDF was changed.
- **Listed every PDF change in this file and the README**, at the user's request, under "Changes marked in `Dubins_TSP.pdf`": R1-R8 to remove, A1-A15 added, the small wording points left unmarked, and the fact that only Section 0.13 was checked line by line. The PDF itself was not edited again.
- **Marked up Section 0.13 of `Dubins_TSP.pdf`**, at the user's request. No original line was removed: lines were shifted down to open gaps, with additions in blue, and lines to remove shaded red with a PDF comment and a visible red note (R1-R8). The PDF went from 13 to 15 pages, and pages 10-15 carry newly drawn page numbers. A check found the first build had deleted original text, and that was fixed before the file was replaced: all 108 original line groups are now pixel-identical to the original and no word is missing. Every added formula matched `dubins_path.py` on 74192 CSC and 18400 CCC paths.
- **Further clean-up, at the user's request.** Removed the blank line from `geodesic.py`. Deleted `.gitignore`, after the user had been told what it protects. Deleted `__pycache__/` and the untracked 7 MB `output/seed3.gif`, which `python planner.py --seed 3 --gif` recreates. Every `.py` file is in use, so none was removed. Refreshed `output/seed2.png`, which still showed the route from before the seed 2 change. Installed `poppler-utils` and the PDF Preview extension.
- **Repo clean-up, at the user's request.** Deleted the root `dubins.py`, which was byte-identical to `Dubins_TSP_Antigravity/dubins.py` and used by only one test; that test now imports the Antigravity copy. Kept `compare_dubins.py`, because it still produces the README's comparison of the two Dubins modes; it can go if the `pdf` mode is ever dropped. All 26 tests pass.
- **Rewrote `dubins_path.py` with the names and steps of the user's DubinsPath notebook program**, at the user's request, and corrected that program's limitations, which the file's opening comment lists in plain language. Both modes remain, and the default is still `"shortest"`. Old to new names: `select` became `findDubinsPath` and `returnDubinsPath`, `lengths` became `returnPathDistances`, `sample` became `calculateTrajectory`, `shortest(...)` became `findDubinsPath(..., "shortest")`, `pdf_word` became `findTurnType`, and `WORDS` became `TURN_TYPES`. `trajectory.py`, `compare_dubins.py` and the tests were updated to match.
- **Measured the notebook program** (R = 5, 3000 random poses in a 40 m square): no path on 41 % of poses, a crash when `cp2 = 0` or the points coincide, CCC lengths about 13 m too short on average, and a mean CSC length error of 1.3 m with a worst case of 31.7 m. It does match Section 0.13's turn-type rule, circle centres and `Stan` band, but it shares the section's `atan` error.
- **Checked the rewrite against the old module.** On 20000 random poses the choices were the same, lengths agreed within 1e-12 m and points within 2e-13 m. On 18000 awkward poses in `shortest` mode, built by driving known paths (zero-length parts, circles that only just touch), the new code was never longer than the known path, and the old module was longer on 1636. This testing also found two defects in the first draft of the rewrite, both fixed: false full circles near `Stan = 2R`, and touching circles treated as impossible.
- **Seed 2 changed, the other 19 rows of `compare_dubins.py` did not.** In default mode T_max went from 74.2 to 68.7 s, L_total from 2880 to 2869 m, and the pairs left from 2 (closest 0.67 m) to 0 (closest 3.01 m). The plans before the game agree within 7e-14 m. The cause is one docking-zone check flipping (sink 5 at 5.000000000000002 m versus 4.999999999999997 m from the base; open item 5), so this is not evidence of better separation. The README tables were re-measured.
- **Tests.** Added `test_dubins_every_turn_type_reaches_the_target_smoothly` and `test_dubins_cases_where_the_notebook_program_failed`; all 26 pass.
- **Commit status.** All of this work was committed and merged into `main`, at the user's request.
- **Pushed the README and CLAUDE.md corrections**, at the user's request, on branch `docs/correct-dubins-leg-counts`. That branch was merged into `main` later the same day.

### 2026-11-04

- **Added `compare_dubins.py`**, at the user's request, to replace a throwaway script. It prints every number in the README's Dubins comparison, and a 10-map run matched the throwaway run on all 20 per-map rows. `classify_leg` (using a helper now called `dubins_path.findTurnType`) gives fallback legs their own category; the throwaway merged them into "shortest", which produced a wrong "1504 of 1509" in the README and in a report to the user, now corrected to 1443. Added `test_compare_counts_fallback_legs_as_missing_not_shortest`; all 24 tests passed at that point. Corrected the README's test time to about 5 minutes (measured 4 min 55 s).
- **Mismatch found by the user.** `dubins_path.py` cited Sections 0.12-0.13 but computes all six words and keeps the shortest, as the original `dubins.py` does (its turn-type functions are commented out as unused). Section 0.13 instead picks one word from `cp1`, `cp2` and `S_tan`. Given the options, the user chose to support both behind a switch.
- **Added `dubins_method`.** Values are `"shortest"` (default) and `"pdf"`, with CLI flag `--dubins`; the code was `dubins_path.select` and `_pdf_word_index` (renamed on 2026-11-05). Default results are unchanged on seeds 1-10. `pdf` follows the paper only in the choice of word: both modes share the exact closed-form geometry, so the Δψ stepping of 0.13.1 is not used. Gaps were filled by falling back to the shortest existing word and treating `cp2 = 0` as a left turn.
- **Measured over seeds 1-10** (values at that time; the README has the current ones).
  - `pdf` gives 2.4 % more total length (longer on every map) and 156 sinks instead of 155.
  - With the same visiting orders it adds 1.8 %, and 2 of 155 tours then exceed B.
  - The heading search steers towards poses where the rule's word is the shortest: 65.2 % of legs on the default plan's poses against 95.6 % on the rule's own plan, while "exists but longer" falls from 33.1 % to 0.3 %. The cost shows up as less favourable headings.
  - On random poses (a one-off experiment, not in the repo) the rule looks much worse, picking the shortest word only 8 % of the time below 2R and 39 % from 2R to 4R. Do not quote those figures as the planner's cost.
  - The conflict differences between modes are too small to rank them.
- **Fixed `conflict_game.detour_traj`.** The curvature check covered only the detour and one neighbouring frame on each side. A sink parked at a sensor just before a detour hid its incoming direction, which let a 1.607/R turn past the 1.15/R limit. It now checks the whole resulting trajectory. Default K, T_max and L_total are unchanged on seeds 1-10.
- **Tests.** Added three `test_pdf_rule_*` tests and `test_end_to_end_with_pdf_rule`; all 23 passed at that point.
- **README.** Added the comparison section and corrected its description of `dubins_path.py`.
- **Created this file.**
- **Commit status.** Commit `374a424` ("dubins path code modified", made under the user's git name and pushed to `origin/main`) holds this day's code, including `compare_dubins.py`, with older versions of the README and this file that still have the wrong 1504 figure.

### Earlier (from git history)

- 2026-10-30: new implementation added.
- 2026-11-02: Antigravity code moved to its own folder.
- 2026-11-04: `simple_resolver.py` added as a non-game alternative.
