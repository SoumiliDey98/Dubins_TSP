# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Keeping this file current

The user asked for this file to be kept up to date as work proceeds. In the same turn as any change to behaviour, commands, decisions, measured results or open items, update the affected section and add a dated entry to the work log. Correct stale statements instead of appending contradictions. Full result tables belong in the implementation's README; this file points to them.

## Repository layout

Dubins TSP for wireless sensor networks: several mobile sinks, modelled as Dubins vehicles, leave a base station, collect data from every sensor and return, avoiding obstacles and each other.

- `Dubins_TSP.pdf`: the paper (Soumili Dey, May 2026) and the specification for the code.
- `Dubins_TSP_New_Implementation_claude/`: the active implementation. Everything below refers to it unless stated. Its `README.md` has the file-to-paper mapping, the choices made where the paper is silent, what is guaranteed versus only measured, and all result tables.
- `dubins.py` (root): the original Dubins code. Treat as read-only: `test_dubins_matches_original_and_ends_at_target` imports it as the reference.
- `theory_claude/`: theory PDFs for the conflict game and the simple resolver, plus `corridor_game_gt.py`, which `conflict_game.py` was adapted from.
- `Dubins_TSP_Antigravity/`: a separate implementation with its own README and requirements. The active implementation does not use it.

## Commands

There is no build step, and no linter or formatter is configured. Run these from `Dubins_TSP_New_Implementation_claude/`:

```bash
pip install -r requirements.txt
python planner.py --seed 1 --no-plot           # plan, check, print a summary; writes nothing
python planner.py --seed 1                     # also writes output/seed1.png
python planner.py --seed 1 --budget 100        # another mission budget B [s]
python planner.py --seed 1 --gif               # also writes output/seed1.gif (slow)
python planner.py --seed 1 --resolver simple   # "wait your turn" instead of the conflict game
python planner.py --seed 1 --dubins pdf        # Section 0.13 word rule instead of the shortest of six
python tests/run_tests.py                      # whole suite

# One test: tests are plain functions and run_tests.py has no selector
python -c "import sys; sys.path.insert(0, 'tests'); import run_tests; run_tests.test_pdf_rule_picks_the_word_of_section_0_13()"
```

pytest is not in `requirements.txt` and not installed here. After `pip install pytest`, name the file explicitly, because it does not match `test_*.py`:

```bash
python -m pytest tests/run_tests.py -k <name>
python -m pytest tests/run_tests.py -k "not end_to_end and not same_seed"   # skips the full planner runs
```

- The suite makes seven full planner runs, at roughly 40-80 s each on the 2-core codespace, so run it in the background.
- New `test_*` functions in `run_tests.py` are picked up automatically.
- `output/seed1.png`, `seed1_simple.png`, `seed2.png`, `seed3.png` and `seed3_simple.png` are tracked in git, so plotting those runs dirties the tree. Use `--no-plot` for checks. GIFs are ignored.
- Reading the PDFs: `pdftotext`, `pdftoppm` and `pypdf` are not installed, and the Read tool cannot render PDFs without poppler. Use `pip install --target <scratch dir> pypdf` and extract the text. Equations come out garbled and can lose symbols, so read them with care. Section 0.13 is on PDF pages 10-13.

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
- **Choosing the Dubins word.** `p["dubins_method"]` is read only in `trajectory.py`, at three call sites that must agree: the heading search's cost table (`dubins_path.lengths`), and `_leg_clear` and `build` (`dubins_path.select`). Otherwise the heading search would optimise paths the sink does not drive. `dubins_path.shortest` is always the true six-word minimum.
- **Frames are the shared timeline.** A trajectory is an (F, 2) array with one position every `dt` seconds, and all sinks start at t = 0. Resolvers see only frames: WAIT repeats frames, DETOUR replaces a window, and both update T and L. So `result["tours"]` is the plan before resolving, while `check` reads the final `result["frames"]`, `result["T"]` and `result["L"]`.
- **Interchangeable resolvers.** Both have the signature `resolve(trajs, T, L, geo, p) -> (trajs, T, L, records)`, and `conflict_game.summary` scores both. Eq. 20 applies only while both sinks are on their mission, and not when both are inside the docking zone (`base_radius`).
- **Game invariant.** A pair's danger is charged equally to both players, which makes it an exact potential game, so best response always stops. Changes to the costs in `conflict_game.play` must preserve this; `test_game_is_an_exact_potential_game` checks it. The proof is in `theory_claude/Corridor_Game_Theory.pdf`.
- **Obstacle margins.** Corner-to-corner lines stay outside obstacles inflated by `r_plan` = `r_safe` + R. Lines touching a sensor or the base only need `r_safe` + 0.5 m. Final clearance is measured against the original obstacles and needs `r_safe`.
- **Curvature.** Dense Dubins paths measure 1.000/R. `trajectory.max_curvature` fits circles through points at least 1 m apart, so sampled frames read slightly higher, and `curvature_tol` = 0.15 allows for that.
- **Parameters.** Everything is one dict from `config.default_params(**changes)`. `R`, `r_safe` and `r_plan` are recomputed after the changes are applied, so `default_params(R=...)` has no effect; change `wheelbase` or `delta_max` instead.
- **Determinism.** `run` uses `rng(seed)` for Steps 1-7 and `rng(seed + 1000)` for Steps 8-12, so the resolvers start those steps alike. An extra random draw or a geometry change shifts every result, and the README's tables were measured on seeds 1-10. Re-measure and update them when a change affects them.

## Conventions

- The code is plain functions on NumPy arrays and dicts, with no classes. Docstrings cite paper sections and equations.
- `Dubins_TSP.pdf` is the specification, and the user checks the code against it. Citing a section is a claim that the code follows it, so read the section first. State any departure in the docstring and the README; the README's "Choices where the paper is silent" table records how gaps were filled.
- New options follow the pattern of `resolver` and `dubins_method`: a key in `config.py`, a flag in `planner.main`, a filename suffix so outputs do not overwrite each other, and a README section with measurements.

## Open items

1. **Section 0.13 of the PDF.** It cannot be implemented as written, and the user has not yet decided whether to edit it. If the `pdf` mode is described in the paper, these gaps need addressing:
   - There is no fallback when the rule picks LSR or RSL with `S_tan < 2R`, where they do not exist.
   - CCC is only considered for `2R <= S_tan < 3R`, but it can exist up to 4R. For (0, 0, 0°) to (1.5R, 0, 180°), `S_tan` = 1.5R and the true shortest path is RLR at 6.72R.
   - Section 0.13.1 has no CCC construction.
   - The cut-out angle uses `tan⁻¹(2R/S_tan)`. With `S_tan` as the distance between centres, the correct form is `sin⁻¹(2R/S_tan)`.
   - `cp2 = 0` is not covered.
2. **Comparison script not in the repo.** The README's per-leg and same-order numbers came from a throwaway script. `python planner.py --seed N --no-plot --dubins pdf` reproduces only K, T_max, L_total and the final conflicts. Adding the script was offered; the user has not answered.
3. **Eq. 20 is not guaranteed.** Conflicts are reported, not always removed. Default mode leaves them on seeds 2 and 5 (closest 0.67 m and 0.28 m against `d_safe` = 3 m); `pdf` mode leaves them on seeds 5 and 9 (1.30 m and 1.88 m).
4. **No dedicated test for the detour fix below.** Only `test_end_to_end_with_pdf_rule` (seed 1) exercises it.
5. **Stale README figure.** The README says the tests take "about 2 minutes", which does not match the planner run times above.

## Work log

Newest first.

### 2026-11-04

- **Mismatch found by the user.** `dubins_path.py` cited Sections 0.12-0.13 but computes all six words and keeps the shortest, as root `dubins.py` does (its turn-type functions are commented out as unused). Section 0.13 instead picks one word from `cp1`, `cp2` and `S_tan`. Given the options, the user chose to support both behind a switch.
- **Added `dubins_method`.** Values are `"shortest"` (default) and `"pdf"`, with CLI flag `--dubins`; the code is `dubins_path.select` and `_pdf_word_index`. Default results are unchanged on seeds 1-10. `pdf` follows the paper only in the choice of word: both modes share the exact closed-form geometry, so the Δψ stepping of 0.13.1 is not used. Gaps were filled by falling back to the shortest existing word and treating `cp2 = 0` as a left turn.
- **Measured over seeds 1-10** (table in the README).
  - `pdf` gives 2.4 % more total length (longer on every map) and 156 sinks instead of 155.
  - With the same visiting orders it adds 1.8 %, and 2 of 155 tours then exceed B.
  - The heading search avoids poses where the rule does badly: on 1504 of 1509 final legs the rule's word was also the shortest, and the fallback applied on 61. The cost shows up as less favourable headings.
  - On random poses the rule looks much worse, picking the shortest word only 8 % of the time below 2R and 39 % from 2R to 4R. Do not quote those figures as the planner's cost.
  - The conflict differences between modes are too small to rank them.
- **Fixed `conflict_game.detour_traj`.** The curvature check covered only the detour and one neighbouring frame on each side. A sink parked at a sensor just before a detour hid its incoming direction, which let a 1.607/R turn past the 1.15/R limit. It now checks the whole resulting trajectory. Default K, T_max and L_total are unchanged on seeds 1-10.
- **Tests.** Added three `test_pdf_rule_*` tests and `test_end_to_end_with_pdf_rule`; all 23 pass.
- **README.** Added the comparison section and corrected its description of `dubins_path.py`.
- **Created this file.** All of this day's changes were uncommitted when it was written; check `git status`.

### Earlier (from git history)

- 2026-10-30: new implementation added.
- 2026-11-02: Antigravity code moved to its own folder.
- 2026-11-04: `simple_resolver.py` added as a non-game alternative.
