# Inter-vehicle conflict resolution for pre-planned trajectories: game-theoretic and timing-based methods (novelty assessment)

Prepared 2026-11-06. Scope: the draft's conflict game (GO / WAIT / DETOUR), its simple rule (turns, longest tour first), and its repair step.

**How to read the evidence tags.** Each cited claim carries one tag saying how far it was checked.

- **[FT]** I opened the paper's full text and read the passage.
- **[AB]** I read the abstract (publisher record, arXiv or OpenAlex), not the full text.
- **[SR]** The claim comes only from a search-result excerpt or a secondary source. The paper's existence is confirmed, but I did not open it.
- **[META]** Only the bibliographic record was checked (Crossref). Any statement about its content is from memory and is marked "unverified".

Every DOI in these notes was resolved against Crossref on 2026-11-06. At least six DOIs I first recalled from memory pointed to the wrong paper or to nothing, and were replaced after a title search, so the DOIs in these notes are the corrected ones. The full list of papers is in the last section.

## Overall: how novel is the game, and what should the paper claim?

### Takeaway
No paper found combines the draft's exact elements, but each element on its own has clear precedent. The exact potential property is a routine consequence of symmetric pairwise coupling. The safest claim is a new combination plus a measured comparison on pre-planned Dubins sink routes, not a new class of game.

### Cited Findings
- Symmetric pairwise coupling is already a stated sufficient condition for a dynamic potential game. If `C_ca^ij(x_i, x_j) = C_ca^ji(x_j, x_i)`, the potential is the sum of private costs plus each pair's coupling counted once, and equilibria come from "a single optimal control problem". This is the same structure as the draft's potential. — [Williams, Chen, Mehr 2023](https://arxiv.org/abs/2303.04842) [FT], crediting [Kavuncu, Yaraneri, Mehr 2021](https://doi.org/10.15607/RSS.2021.XVII.084) [SR]
- The closest theoretical precedent is Urban Driving Games. Preferences are lexicographic with "the shared objective of not colliding" first, communal games are lexicographic ordinal potential games, "socially efficient equilibria can be found by solving a single (lexicographic) optimal control problem", and iterated best response has convergence guarantees. — [Zanardi et al. 2021](https://doi.org/10.1109/LRA.2021.3068657) [AB]
- Finite trajectory sets with prioritised metrics and pure equilibria are also covered: "autonomous vehicles select from a finite set of trajectories". — [Zanardi et al. 2022, Posetal Games](https://doi.org/10.1109/LRA.2021.3135030) [AB]
- Mixed discrete decisions, a potential game and best-response algorithms inside open- and closed-loop policies are treated for selfish highway vehicles. — [Fabiani, Grammatico 2020](https://doi.org/10.1109/TITS.2019.2901505) [AB]
- Whole-trajectory, open-loop, non-cooperative games with robots improving in turn are treated with convergence to Nash equilibria and a price of stability of one for scalar costs. — [Zhu, Otte, Chaudhari, Frazzoli 2014](https://arxiv.org/abs/1402.2708) [FT]
- Time-ordered conflict handling exists without any game: NASA's Autoresolver works down a conflict list ordered by "predicted time to first loss of separation", defers conflicts beyond a threshold, and rejects trial trajectories that create secondary conflicts. — [Erzberger, Lauderdale, Chu 2010](https://icas.org/ICAS_ARCHIVE/ICAS2010/PAPERS/197.PDF) [FT]
- A repair loop that replans only the colliding agents is the published MAPF-LNS2 method. — [Li et al. 2022](https://doi.org/10.1609/aaai.v36i9.21266) [SR]
- Measurements in this repo affect how strongly the comparison can be stated:
  - Resolver alone, the game leaves 13.0 s against 103.5 s for the simple rule. With two fixes to the simple rule (exact closest-distance test, accepting the lowest-risk stop) it leaves 31.0 s, and the game without DETOUR leaves 15.5 s.
  - The 10 s look-ahead was "patched in memory", is not in the code and "has not been through the tests".
  - Of 18 maps where both end conflict-free, the simple rule has lower T_max on 10 and the game on 1.

  — `Dubins_TSP_New_Implementation_claude/README.md` (what-if table and "Which is better") [FT]

### Inferences
- **What is not novel on its own:** exact potential from shared cost; dominance of safety by a large weight; best response on finite action sets; earliest-conflict-first processing; a time-in-conflict measure; replanning the conflicting agents; a look-ahead window.
- **What may be new:**
  - the specific sequence of stage games, each anchored at the earliest unsafe interval, over discrete repair manoeuvres on already-built Dubins mTSP routes;
  - seconds of exposure as the shared currency;
  - the measured effect of truncating that exposure inside the cost.
- **Closest prior methods, in my ranking:**
  1. Zanardi 2021 and 2022, for the theory.
  2. Fabiani and Grammatico 2020, for discrete best response in a potential game.
  3. Zhu et al. 2014, for open-loop whole trajectories.
  4. Chandra et al. (N-player Chicken at doorways), for "who yields".
  5. Feurgard et al. 2026, for Dubins candidates and a pairwise separation table, solved by ILP and not by a game.
  6. Erzberger's Autoresolver, for time order.
  7. Hu et al. 2024, for a game-theoretic reading of priority order.
- **What reviewers will probably ask**, most likely first:
  1. Why use a game with a central planner holding full knowledge?
  2. Why not minimise the potential directly?
  3. Why not a wait-only MILP or a coordination diagram?
  4. Why not a CBS-family baseline?
  5. Is the simple rule a fair baseline, given that its known flaws are fixable?
  6. Why is eq. 20 only reported and not guaranteed?
  7. Is open-loop execution robust to delay?
  8. Is a vehicle that stops still a Dubins vehicle, since the model assumes constant forward speed?
  9. Is the text consistent with the code? The draft describes a CorridorGame with Left / Right / Detour, while the code plays GO / WAIT / DETOUR over whole trajectories.
- **The game is largely a timing method in practice.** DETOUR was chosen in 11 of 613 decisions and removing it moves the result only from 13.0 s to 15.5 s. Reviewers will therefore classify it with velocity tuning.
- **Statistical strength of the results:**
  - 22 of 30 versus 20 of 30 cannot be significant. Even if all differences favour the game (2 to 0), an exact paired test gives p = 0.5.
  - "Better on 7 maps, worse on 2" for the 10 s variant gives p of about 0.18 on a two-sided sign test.
  - The 10 s value also appears to have been chosen on the same maps, so it needs held-out maps. The exposure gap is the stronger evidence.

### Gaps
- I could not open the full text of Zanardi 2021 or Fabiani and Grammatico 2020, the two most important prior papers. Both must be read before any novelty claim is submitted. In particular, check whether Zanardi's iterated best response assumptions fit discrete manoeuvres.
- No search can prove absence. "Not found" here means not found with the queries listed in the last section.

## What is the foundation: path-velocity decomposition, prioritized planning, and optimal or near-optimal MAPF?

### Takeaway
The simple rule is prioritized planning with velocity tuning on fixed paths, and both ideas date from the 1980s. "Longest first" is an established benchmark heuristic rather than a design contribution. CBS variants offer optimality or completeness on discretised models, and no Dubins-specific CBS variant was found.

### Cited Findings
- **Path-velocity decomposition.** Plan a path around static obstacles, then plan velocity along it around moving obstacles, solved as a graph search in path-time space. — [Kant, Zucker 1986](https://doi.org/10.1177/027836498600500304), content via [a summary page](https://bengravell.github.io/knowledge-base/papers/1986_kant_toward_efficient_trajectory_planning_the/) [SR]
- It was extended to car-like robots with bounded velocity and acceleration among moving obstacles. — [Fraichard, Laugier, IJCAI 1993](https://ijcai.org/proceedings/1993/221) [SR]
- **Prioritized planning.** The idea was "first articulated by" Erdmann and Lozano-Pérez, per Čáp et al. — [Čáp et al. 2015](https://arxiv.org/abs/1409.2399) [FT] on [Erdmann, Lozano-Pérez 1987](https://doi.org/10.1007/BF01840371) [META]
- **Priority heuristic.** The "Longest First" benchmark prioritises "the robot that has the longest remaining path to its goal" and is credited to van den Berg and Overmars. — [Wu, Bhattacharya, Prorok 2020](https://ar5iv.arxiv.org/html/1908.02361) [FT, via a fetched summary]. The original abstract says it studies "the assignment of priorities to the robots". — [van den Berg, Overmars 2005](https://doi.org/10.1109/IROS.2005.1545306) [AB]. I did not read the original's wording on the heuristic.
- **Completeness of prioritized planning.**
  - Theorem 2: if every robot has a path avoiding the start regions of lower-priority robots and the goal regions of higher-priority robots, a sequential conflict-free solution can be constructed.
  - The robot's fall-back is "to wait at its start position".
  - Corollary 3: Revised Prioritized Planning then succeeds if the single-robot planner is complete.
  - A valid infrastructure is a workspace and endpoints where any two endpoints connect by a path with r-clearance from the workspace boundary and 2r-clearance from every other endpoint, with each endpoint used by at most one robot.
  - In the experiments, RPP-based algorithms show "full-instance coverage" on infrastructure tasks but a lower success rate elsewhere.

  — [Čáp, Novák, Kleiner, Selecký 2015](https://arxiv.org/abs/1409.2399) [FT]
- Prioritized planning with an arbitrary order is incomplete in general, and complete on "well-formed" instances. — [Ma et al. 2019](https://arxiv.org/pdf/1812.06356) [SR]
- **Conflict-Based Search.** A two-level algorithm for optimal multi-agent path finding. — [Sharon et al. 2015](https://doi.org/10.1016/j.artint.2014.11.006) [SR]. Described as "complete and optimal" in [Li et al. 2021](https://arxiv.org/abs/2005.07371) [FT].
- **CCBS.** Combines Safe Interval Path Planning with CBS and guarantees minimal sum of costs in continuous time. SMT-CCBS guarantees minimal makespan. — [Andreychuk et al. 2022](https://doi.org/10.1016/j.artint.2022.103662) [SR]
- **MC-CBS.** Multi-Constraint CBS addresses large agents occupying several grid cells, with runtime gains of up to three orders of magnitude over CBS. It concerns agent size on grids, not kinematics. — [Li et al. 2019](https://doi.org/10.1609/aaai.v33i01.33017627) [SR]
- **K-CBS.** Described as probabilistically complete, with any sampling-based tree planner at the low level. — [Kottinger, Almagor, Lahijanian 2022](https://doi.org/10.1109/IROS47612.2022.9982018) [SR]
- **CL-CBS.** For car-like robots, with Spatiotemporal Hybrid-State A* at the low level. — [Wen, Liu, Li 2022](https://doi.org/10.1016/j.robot.2021.103997) [SR]
- **db-CBS.** Combines CBS with discontinuity-bounded A* for dynamics constraints. — [Moldagalieva et al. 2024](https://doi.org/10.1109/icra57147.2024.10610999) [SR]. A 2026 follow-on, [db-LaCAM](https://doi.org/10.1609/icaps.v36i1.42894), exists [META].
- **Repair.** MAPF-LNS2 starts from colliding paths and repeatedly replans a group of colliding agents. It is compared with prioritized planning with random restarts, EECBS and PPS. — [Li et al. 2022](https://doi.org/10.1609/aaai.v36i9.21266) [SR]
- **TSP plus conflicts.** CBSS interleaves CBS conflict resolution with multiple-TSP sequencing, handling up to 20 agents and 50 targets in the conference version. This is the nearest published problem to the draft's Steps 1-12. — [Ren, Rathinam, Choset 2023](https://doi.org/10.1109/TRO.2023.3266993) [SR]

### Inferences
- The draft's rule fits the definition of prioritized planning restricted to one fixed path per robot. That restriction removes the completeness route of Čáp et al., because their result needs a complete single-robot trajectory planner.
- The repair step is an instance of the MAPF-LNS2 idea, with "keep the best round". It should be cited as such, not presented as new.

### Gaps
- I did not read the original wording in van den Berg and Overmars 2005, Kant and Zucker 1986, or Erdmann and Lozano-Pérez 1987. Only the attributions above are confirmed.

## What exists on potential games for conflict resolution, and which is closest to GO/WAIT/DETOUR?

### Takeaway
Potential games are well used in autonomous driving and interactive robotics, mostly over continuous controls or lane decisions. I found none for aircraft or UAV conflict resolution with discrete repair manoeuvres. None is named "corridor game", and the nearest corridor work is decentralised and reactive.

### Cited Findings
- **Theory.** Potential games were defined by [Monderer, Shapley 1996](https://doi.org/10.1006/game.1996.0044) [META]. In a finite generalised ordinal potential game every improvement path is finite, so a pure Nash equilibrium exists and improving moves find one. — [Wikipedia, "Potential game"](https://en.wikipedia.org/wiki/Potential_game) [FT, secondary]
- **Cooperative control.** Consensus is modelled as a potential game, with learning for state-dependent action sets, and applied to consensus with nonconvex obstructions, sensor deployment and coverage. — [Marden, Arslan, Shamma 2009](https://doi.org/10.1109/TSMCB.2009.2017273) [SR]
- **Equivalence with local search.** "When formulated as noncooperative games, DCOPs form a subset of the class of potential games". The authors note that similar work was published independently across literatures, and the families they unify include maximum-gain messaging. — [Chapman, Rogers, Jennings, Leslie 2011](https://doi.org/10.1017/S0269888911000178) [SR]
- **Autonomous driving.**
  - Generalized mixed-integer potential game on highways. — [Fabiani, Grammatico 2020](https://doi.org/10.1109/TITS.2019.2901505) [AB]
  - Lexicographic communal games. — [Zanardi et al. 2021](https://doi.org/10.1109/LRA.2021.3068657) [AB]
  - Factorising dynamic games into smaller subgames using the spatio-temporal resources players need. — [Zanardi et al. 2022](https://doi.org/10.1109/iros47612.2022.9981416) [SR]
- **Interactive robotics.** Potential iLQR and its distributed form; "imagined" potential games with symmetric collision terms. — [Kavuncu et al. 2021](https://doi.org/10.15607/RSS.2021.XVII.084) [SR], [Williams et al. 2023](https://arxiv.org/abs/2303.04842) [FT], [Sun et al. 2023](https://arxiv.org/abs/2310.01614) [AB]
- **Corridor-like scenes.**
  - Deadlock in constrained spaces reduced to a "modified N-player Chicken game", with the equilibrium encoded as a control barrier function. The baseline is random perturbation, and the metrics are cost and path deviation. — [Chandra et al., project page](https://amrl.cs.utexas.edu/snupi.html) [FT], [Chandra et al. 2025](https://doi.org/10.1007/s10514-025-10194-8) [META]
  - Taxonomy of doorways, hallways and corridor intersections. — [Chandra, Singh, Luo, Sycara](https://arxiv.org/abs/2508.13459) [FT, sections]
  - SocialMAPF, which uses mechanism design during MAPF conflict resolution in narrow hallways. — [Chandra et al. 2023](https://doi.org/10.1109/lra.2023.3265169) [AB]
  - Yield-or-go as a chicken game is standard in driving surveys. — [Scenario-based game theory survey, arXiv 2509.05777](https://arxiv.org/abs/2509.05777) [SR]
- **Air traffic games use other concepts.**
  - Satisficing game theory. — [Archibald et al. 2008](https://doi.org/10.1109/tsmcc.2008.919162) [AB]
  - Sequential bargaining over which aircraft manoeuvres. — [Pritchett, Genton 2018](https://wcl.cs.rpi.edu/pilots/library/papers/collision/game_theory_negotiated_decentralized_conflict_resolution.pdf) [SR, title page opened]
  - A cooperative game with a nucleolus-style solution. — [Symmetry 2022](https://doi.org/10.3390/sym14081517) [SR]
  - A receding-horizon differential Nash scheme for drones. — [Hoogendoorn et al. 2023](https://arxiv.org/abs/2308.01069) [FT]
- **Priority as a game.** Computing the "socially optimal order of play" in an N-player Stackelberg trajectory game with Branch and Play, "an efficient and exact algorithm". It extends sequential trajectory planning. Baselines include first-come-first-served and Nash solutions. — [Hu et al. 2024](https://doi.org/10.15607/rss.2024.xx.116) [FT, abstract and contributions]
- **Early game language in multi-robot planning.** Coordination along fixed independent paths, with independent performance measures and optimality "consistent with concepts from multiobjective optimization and game theory". — [LaValle, Hutchinson 1998](https://doi.org/10.1109/70.736775) [AB]

### Inferences
- The draft's `play` in `conflict_game.py` selects, at each step, the player and strategy with the largest gain. This looks like the maximum-gain local search that Chapman et al. place inside the potential-game family. A reviewer from the DCOP community may read it that way.
- The draft's game is not a generalised game as long as the budget and detour feasibility depend only on a sink's own strategy. Rejecting over-budget strategies only shrinks that player's set, so the exact potential survives. If a strategy's feasibility ever depended on another sink's choice, Fabiani's "generalized" setting would apply.
- Starting from all-GO selects one equilibrium among several. [Zanardi et al. 2023](https://arxiv.org/abs/2210.13064) [FT] describe a "plurality of different equilibria posing the issue of equilibrium selection", and report different runs producing different orders of right of way.

### Gaps
- I found no potential-game paper with GO, WAIT and DETOUR-style strategies, and no game with Left / Right / Detour over corridors.
- A targeted search for potential games in air traffic returned no exact potential formulation. That search is limited, and journals such as JGCD were not directly searchable.

## Do papers use exposure time as a shared penalty, best-response resolution in time order, or a large collision weight?

### Takeaway
Each ingredient exists separately. Time-exposed safety measures exist as road-safety metrics, shared proximity costs are standard, and lexicographic collision priority is published. I found no paper that uses seconds of exposure as the shared cost in a game, or best response conflict by conflict in time order.

### Cited Findings
- **Exposure as a metric.** Time Exposed TTC (TET) is the integral over a scenario of an indicator that TTC is at or below a threshold. — [criticality-metrics documentation](https://criticality-metrics.readthedocs.io/en/latest/_sources/time-scale/TET.rst.txt) [FT, secondary], citing [Minderhoud, Bovy 2001](https://doi.org/10.1016/S0001-4575(00)00019-1) [META, existence; SR, content]
- **Shared proximity cost in a game.** In driving games, costs are `J_i = J_i^prox + J_i^per`, where the proximity cost is "collision at the limit", must be integrable over the trajectory and must increase monotonically as distance decreases. — [Zanardi et al. 2023](https://arxiv.org/abs/2210.13064) [FT]
- **Shared collision term.** Describing Urban Driving Games, this paper says "a collision cost is a shared cost which enforces no collision", alongside a personal cost, and shows its revised game is still a potential game. — [Miller, Mitra 2022](https://doi.org/10.1109/cdc51059.2022.9992957) [FT, passage]
- **Time in conflict in aviation.** Strategic 4D deconfliction minimises "interactions between trajectories" over a day of European traffic, but I could not confirm whether that interaction measure is time-integrated. — [Chaimatanan, Delahaye, Mongeau 2014](https://doi.org/10.1109/mci.2014.2350951) [SR]
- A genetic-algorithm line of work is framed around delay. I could not confirm a conflict-duration fitness. — [Medioni, Durand, Alliot 1995](https://hal-enac.archives-ouvertes.fr/hal-01004091) [SR]
- **Time-ordered resolution, no game.**
  - The Autoresolver processes conflicts by time to first loss. — [Erzberger et al. 2010](https://icas.org/ICAS_ARCHIVE/ICAS2010/PAPERS/197.PDF) [FT]
  - Stability under sequential conflict resolution is shown for the aircraft flow geometries that paper considers. — [Hand, Mao, Feron 2011](https://doi.org/10.1109/acc.2011.5991516) [AB]
  - For more than two intersecting flows, simulations indicate stability "may not be guaranteed under simple conflict avoidance rules". — [Mao dissertation](https://core.ac.uk/download/16521534.pdf) [SR]
- **Best response in order.** Robots "sequentially update the planned paths" by better response in rounds, but over the whole trajectory, not per conflict. — [Zhu et al. 2014](https://arxiv.org/abs/1402.2708) [FT]. Iterated best response is common in racing games, and only approximates equilibria in general. — [Wang et al. 2021](https://doi.org/10.1109/TRO.2020.3047521) [META], [Wang et al. 2020](https://msl.stanford.edu/papers/wang_game_2020.pdf) [SR]
- **Lexicographic priority.** Collision first. — [Zanardi et al. 2021](https://doi.org/10.1109/LRA.2021.3068657) [AB]. Weighted single objectives "can easily become ill-conditioned", and in that paper's experiments the weighted baseline "fails to consistently match" and has high variance. The same paper notes that iterated best response applies only where it is guaranteed to converge. — [Lee, Peters, Fridovich-Keil 2025](https://arxiv.org/abs/2410.21447) [FT, via a fetched summary with quotations]
- **Weights equal to lexicographic order.** Relevant classical results on equivalent weights exist: [Sherali 1982](https://doi.org/10.1016/0377-2217(82)90202-8) and [Sherali, Soyster 1983](https://doi.org/10.1007/bf00934527) [META; relevance from memory, unverified].

### Inferences
- The draft's exposure is the distance-threshold analogue of TET. It is also a step-function case of Zanardi's proximity cost. The proximity cost must be monotonic in distance, and a step function meets this only in the weak sense. Their price-of-anarchy results might therefore apply, but this needs checking against their definitions. Citing TET would help place the metric.
- The ill-conditioning objection is about continuous solvers. It should not apply to enumeration on finite tables, where the draft's `alpha = (1 + sum of cost ranges) / dt` is an exact dominance choice. The paper can state this directly.
- **A property the paper could prove, derived from reading `conflict_game.py` and not tested:**
  - Exposure counts in whole multiples of `dt`.
  - `play` starts at all-GO, and its potential includes the risk against non-players.
  - Every move lowers the potential, and alpha exceeds the total cost range divided by `dt`.
  - So, as written, no game can raise the total remaining exposure from that frame on.

  This is stronger than "best response stops" and is the kind of monotonic claim a reviewer would value. The 10 s variant loses it, because only exposure inside the window is then monotonic.

### Gaps
- I could not open the Chaimatanan paper (blocked) or read how "interaction" is defined. If it is a time integral, it is the closest aviation precedent for exposure.
- The fitness used by Durand and Alliot's genetic algorithm was not confirmed.

## Is game theory accepted when a central planner controls all agents, and what is said about efficiency?

### Takeaway
I found no paper that directly criticises the practice. However, the authors I read justify games by decentralisation, self-interest, scale, or modelling of others. With full central knowledge, those reasons fall away, leaving computational convenience, and that reason is weak for small stage games with tabulated costs.

### Cited Findings
- **Stated reasons for games.**
  - "With the expected large number of drones, we argue that centralized planning of all drone trajectories is not feasible nor practical." — [Hoogendoorn et al. 2023](https://arxiv.org/abs/2308.01069) [FT]
  - "Selfish vehicles" and a distributed framework. — [Fabiani, Grammatico 2020](https://doi.org/10.1109/TITS.2019.2901505) [AB]
  - "Decentralized and cooperative" conflict avoidance. — [Archibald et al. 2008](https://doi.org/10.1109/tsmcc.2008.919162) [AB]
  - Game theory is "relevant in settings with multiple interacting controllers", including team games with different information. — [Marden, Shamma 2018](https://doi.org/10.1146/annurev-control-060117-105102) [AB]
  - Game-theoretic tools as a design choice for distributed resource allocation. — [Marden, Wierman 2013](https://econpapers.repec.org/RePEc:inm:oropre:v:61:y:2013:i:1:p:155-168) [SR]
- **Efficiency as the remaining reason.** "Although Nash equilibrium may not be socially optimal, game theoretic approaches remain useful when the computational efficiency dominates." The paper contrasts its decentralised iNash-trajectory with a centralised iOptimalControl that finds a social optimum. With collisions as hard feasibility, it shows that every social optimum is an equilibrium, so the price of stability is one. — [Zhu et al. 2014](https://arxiv.org/abs/1402.2708) [FT]. A table reporting its price of anarchy as "Unknown" was seen only in [a search excerpt](https://www.researchgate.net/publication/260166634_Game_theoretic_controller_synthesis_for_multi-robot_motion_planning_Part_I_Trajectory_based_algorithms) [SR].
- **Warning on safety.** "As safety becomes more important to agents, a large gap opens up between safety achieved by a socially optimal and Nash equilibrium outcomes; in other words, Nash equilibria exhibit significantly more collisions than desirable by all agents." The stated reason is that each agent counts "the impact of collisions on themselves, and not on other agents who crash along with them". The case studies have two and three agents. — [Li, Vorobeychik, OptMAS 2018](https://www2.isye.gatech.edu/~fferdinando3/cfp/OPTMAS18/papers/paper_17.pdf) [FT, introduction and abstract]
- **Bounds.** Driving games are modelled as congestion games over spatio-temporal resources. Price-of-anarchy bounds are refined using "the relative importance between personal and joint costs". Potential structure "guarantees convergence of better-response schemes and, in some cases, social efficiency of global minima". The authors say that selecting the most efficient equilibrium is "often impractical". — [Zanardi et al. 2023](https://arxiv.org/abs/2210.13064) [FT]
- **Central view of MAPF.** Centralised methods such as CBS and M* "ensure global consistency and optimality (or bounded suboptimality), but their complexity grows rapidly with the number of agents". — [Chandra, Singh, Luo, Sycara](https://arxiv.org/abs/2508.13459) [FT]
- **Self-interested MAPF.** Taxation and combinatorial auctions are used. — [Bnaya et al. 2013](https://doi.org/10.1609/socs.v4i1.18292) [SR], [Amir, Sharon, Stern 2015](https://doi.org/10.1609/aaai.v29i1.9427) [SR]
- **Foundations of the measures.** [Koutsoupias, Papadimitriou 1999](https://doi.org/10.1007/3-540-49116-3_38) and [Anshelevich et al. 2008](https://doi.org/10.1137/070680096) [META; roles as origins of worst-case equilibria and price of stability are from memory, unverified].

### Inferences
- A potential minimiser is the natural reviewer's alternative. Williams et al. and Zanardi et al. both reduce equilibrium finding to one optimisation problem. In the draft, any global minimum of the stage potential is a pure Nash equilibrium.
- The code already tabulates every pairwise exposure and counts `n_profiles`, so the exact minimum of each stage can be enumerated. A planner with full knowledge has no computational reason to stop at the first equilibrium, unless the profile counts are large.
- **What can be defended, and what cannot.** Any global minimum of the stage potential is a pure equilibrium, and with the draft's dominant alpha it also has the least total exposure in that stage. So the best equilibrium is optimal on exposure, which is a price of stability of one on exposure, in the spirit of Zhu et al.
- **Li and Vorobeychik's warning applies to the draft.** Each sink's cost counts its own exposure and not the other sink's, which is the structure they blame. A dominant alpha stops a sink trading exposure for time on a single move, but it does not remove coordination failures: an equilibrium can leave exposure that only a joint move would remove. The price of anarchy on exposure can therefore be unbounded in principle (positive exposure against an optimum of zero). This is why the gap should be measured.
- **Most honest framing.** Describe the game as central, open-loop local search whose cost has potential-game structure, which gives termination and a monotonic safety property. If the paper keeps game language, add the path to decentralised execution, and measure the gap. Reporting an empirical price of anarchy against a stage optimum and a whole-mission optimum would itself be a useful, reviewable contribution.

### Gaps
- I did not find a source that explicitly argues against game formulations under central control, or a head-to-head study of central trajectory optimisation against Nash trajectories in cooperative multi-robot planning.
- The Marden and Shamma review text was not opened, only its abstract.

## What is known about receding-horizon or truncated-cost game-based planning, and is the 10 s finding similar to prior reports?

### Takeaway
A shorter window helping quality has one strong precedent, in lifelong MAPF, and that paper says the effect differs from one-shot MAPF. Other work shows gains saturating or longer horizons being better. The draft's finding is plausible and has a matching explanation, but I found no game-based paper reporting that truncation reduces remaining conflicts.

### Cited Findings
- **Bounded-horizon collision resolution.** Rolling-Horizon Collision Resolution resolves collisions only in the first w timesteps:
  - "Sometimes RHCR with smaller time horizons achieves higher throughput than with larger time horizons."
  - "In lifelong MAPF, resolving all collisions in the entire time horizon is unnecessary, which is different from regular MAPF."
  - Small w changes throughput by less than 1 % in most cases and speeds up RHCR by up to a factor of 6.
  - PBS with w = infinity solves up to 700 agents, against at least 1,000 with w = 5.
  - "Too small a value for the time horizon may generate deadlocks."

  — [Li et al. 2021](https://arxiv.org/abs/2005.07371) [FT]
- **Earlier windowing.** WHCA* runs faster as the window shrinks but generates longer paths, as described by Li et al. — [Silver 2005](https://doi.org/10.1609/aiide.v1i1.18726) [AB]
- **Completeness caution.** All existing windowed MAPF solvers are said to lack theoretical completeness. — [WinC-MAPF, arXiv 2410.01798](https://arxiv.org/abs/2410.01798) [SR]
- **Operational precedent.** The Autoresolver defers conflicts beyond a threshold: 8 minutes en route and 20 minutes for arrivals, nominally tied to prediction accuracy. — [Erzberger et al. 2010](https://icas.org/ICAS_ARCHIVE/ICAS2010/PAPERS/197.PDF) [FT]
- **Game-based results pointing the other way.**
  - For a receding-horizon Nash scheme for drones, minimum distance rises with horizon T, and "the improvement reduces substantially when going from T = 5 to T = 10". The authors argue T = 5 is a good trade-off. — [Hoogendoorn et al. 2023](https://arxiv.org/abs/2308.01069) [FT]
  - For non-cooperative distributed MPC of manipulators, execution time falls with longer horizons, and "a prediction horizon length as large as possible is desired". — [Gafur et al.](https://arxiv.org/abs/2103.00583) [FT]
- **Theory.** Without terminal ingredients, numerical examples show "the closed-loop region of attraction around the steady-state GNE shrinks exponentially with the horizon length", and a linear end penalty helps. — [Hall, Dörfler, Faulwasser 2026](https://arxiv.org/abs/2605.12467) [AB]
- **Repo measurements.** With the exposure term limited to 10 s, risk left falls from 13.0 s to 6.5 s and conflict-free maps rise from 17 to 24, resolver alone, with mission time added 985.5 s against 971.5 s. Through full Steps 8-12 it reaches 27 of 30, better on 7 maps and worse on 2. The README's stated cause is that `exposure` "counts risk to the end of the mission, so a stop is charged for later conflicts that later games could solve". — `Dubins_TSP_New_Implementation_claude/README.md` [FT]

### Inferences
- The mechanism looks like RHCR's: charging far-future conflicts now is wasted effort when later decisions will change those conflicts. The draft's setting is open-loop and finite-mission, closer to "regular MAPF" than to lifelong MAPF. RHCR's statement that one-shot MAPF differs means this analogy has to be argued, not assumed.
- Truncation keeps the per-stage exact potential, because both players' shared term uses the same window. It costs the whole-mission monotonic property noted above. That fits the observation that a few maps get worse.
- A paper should report a sweep of window lengths on held-out maps, and watch for the deadlock and short-sightedness that RHCR warns of.

### Gaps
- I found no game-based paper reporting fewer remaining conflicts from a truncated cost. The search was narrow, and work in JGCD or T-ITS may be missed.
- Horizon values cannot be compared across papers, since speeds and separations differ.

## Which works address deconfliction for Dubins or curvature-constrained vehicles?

### Takeaway
A substantial line exists, mostly for fixed-wing aircraft that cannot stop. The closest method to the draft is a 2026 enumeration of Dubins paths with a pairwise separation table and an ILP. No paper was found on explicit velocity scheduling on fixed Dubins paths, or on Dubins TSP with game-based deconfliction.

### Cited Findings
- **Enumeration plus ILP.** Builds a set of Dubins paths per aircraft for a given travel time, "checks in parallel separation between each path pair", then uses an Integer Linear Program to determine "if a fleet-wide conflict-free solution exists". It repeats over travel times, with up to 20 aircraft, a 95 % success rate and 8 s average computation. Aircraft fly at constant speed, with synchronous arrival. — [Feurgard, Hattenberger, Durand, Lacroix 2026](https://arxiv.org/abs/2603.21789) [AB]
- **Optimal conflict resolution with curvature limits.** "A kinematic system with constant velocity and curvature bounds", a safety distance, necessary conditions for optimality, and a decentralised implementation. — [Bicchi, Pallottino 2000](https://doi.org/10.1109/6979.898228) [FT, abstract in PDF]
- **Mixed integer programming.** Conflict resolution for air traffic via MIP. — [Pallottino, Feron, Bicchi 2002](https://doi.org/10.1109/6979.994791) [META, SR]
- **Provably safe manoeuvres.** Hybrid-system synthesis with speed and heading changes, with the maximal safe set. — [Tomlin, Pappas, Sastry 1998](https://doi.org/10.1109/9.664154) [AB]
- **Two identical turning vehicles as a game.** Each vehicle has planar position, heading and fixed linear velocity, and the problem "is called the game of two identical cars". Solved by HJI reachability. — [Mitchell, Bayen, Tomlin 2005](https://doi.org/10.1109/TAC.2005.851439) [FT, passage], citing [Merz 1972](https://doi.org/10.1007/bf00932932) [META]
- **Sequential reachability.** Vehicles planned in priority order, with higher-priority vehicles as time-varying obstacles. — [Chen, Fisac, Sastry, Tomlin 2015](https://doi.org/10.1109/ecc.2015.7331044) [AB]
- **Dubins timing by length.**
  - Dubins sets for multiple UAVs and clothoid-arc variants for simultaneous arrival. — [Shanmugavel et al. 2005](https://doi.org/10.2514/6.2005-5827), [2010](https://doi.org/10.1016/j.conengprac.2009.02.010) [SR]
  - A "Dubins-detour-based time-coordination mechanism" mapping timing into path-length changes. — [Cooperative Online 3D Path Planning, Drones 2026](https://doi.org/10.3390/drones10040297) [AB]
- **Holding with loops.** Closed Dubins trajectories resolve a fixed-wing traffic jam, tested with up to 15 UAVs. — [Sadovsky, Zelinski, Gorospe 2026](https://doi.org/10.2514/1.D0525) [SR]. A first search excerpt named a different journal; Crossref gives Journal of Air Transportation 34:102-112.
- **Fixed-wing corridors, no game.** A main lane, circular loiter lane and speed guidance for reinsertion. — [Pradeep J et al. 2026](https://arxiv.org/abs/2605.13822) [AB]
- **Speed-only on fixed paths.** Assigning speeds with changes of direction excluded, proven NP-hard, with three methods. — [Alejo et al. 2013](https://doi.org/10.1007/s10846-012-9768-4) [FT, theorem statement]
- **Timing on specified paths.**
  - Start times only: "sufficient and necessary conditions", a MILP, up to 20 robots, and proof that "even when the robot trajectories are specified, minimum time coordination of multiple robots is NP-hard". — [Akella, Hutchinson 2002](https://doi.org/10.1109/robot.2002.1013428) [FT]
  - Continuous velocity profiles along specified paths via MINLP and MILP. — [Peng, Akella 2005](https://doi.org/10.1177/0278364905051974) [AB]
  - Time discretisation to a MILP, "resulting in a resolution-optimal solution". — [Altché, Qian, de La Fortelle](https://arxiv.org/abs/1603.04610) [FT]
- **Multi-UAV Dubins TSP.** Focus on obstacles, sequencing and turning limits. "Collision avoidance" is a keyword, and I could not confirm any inter-UAV treatment. — [Fu et al. 2024](https://doi.org/10.1109/tcyb.2023.3265926) [SR]
- **CBS and UAVs.** CBS and ECBS extended to pre-flight UAV conflict detection, with fixed-wing treatment described as in progress. — [Ho et al. 2019](https://www.ifaamas.org/Proceedings/aamas2019/pdfs/p131.pdf) [SR]. See also [Ho et al. 2022](https://doi.org/10.1109/tits.2020.3019397) [META] and [Liu et al. 2023](https://doi.org/10.3390/drones7030217) [SR].

### Inferences
- The draft's WAIT only makes sense for a vehicle that can stop. Strict Dubins vehicles have constant forward speed, and most aviation papers above use speed changes, holding loops or path stretching instead. The paper should say plainly that sinks are ground vehicles with bounded curvature and that stopping is allowed. Reviewers from aviation may otherwise object.
- Feurgard et al. can be read as the non-game version of the draft's structure: discrete options per vehicle, a pairwise separation table, and a joint solve. It is the comparison a Dubins-focused reviewer would raise first.

### Gaps
- I found no paper combining fixed Dubins paths with explicit speed or stop scheduling as a stated problem, and none combining Dubins TSP routes with a game-based resolver. Neither search reached paywalled AIAA full texts.

## What baselines and metrics are standard?

### Takeaway
Reviewers will expect several reference points: no coordination, prioritized planning with more than one priority rule, an optimal or bounded-suboptimal search, and ideally an exact timing MILP. They will also expect success rate, makespan, flowtime, runtime and separation measures.

### Cited Findings
- **MAPF vocabulary.** Common assumptions and objectives, with benchmarks. The most common objectives named are makespan and sum of costs. — [Stern et al. 2019](https://arxiv.org/abs/1906.08291) [FT, passage], published as [SoCS 2019](https://doi.org/10.1609/socs.v10i1.18510)
- **Prioritized-planning comparisons.** Compares Naive Surroundings, Coupled Surroundings, Longest First, Forwards Looking, two Path Prospect variants and Random, with makespan and flowtime as percent increases over the collision-free "ideal", and success rate. — [Wu et al. 2020](https://ar5iv.arxiv.org/html/1908.02361) [FT, via a fetched summary]. Coverage of solved instances and trajectory prolongation are reported in [Čáp et al. 2015](https://arxiv.org/abs/1409.2399) [FT].
- **Repair comparisons.** Prioritized planning with random restarts, EECBS and PPS, with success within a 5-minute limit. — [Li et al. 2022](https://doi.org/10.1609/aaai.v36i9.21266) [SR]
- **Game-based comparisons.** First-come-first-served, randomized and Nash iLQ, against an exact order of play. — [Hu et al. 2024](https://doi.org/10.15607/rss.2024.xx.116) [FT]. Random perturbation as a baseline, with cost and path deviation as metrics. — [Chandra et al.](https://amrl.cs.utexas.edu/snupi.html) [FT]. Number of observed equilibria and empirical price of anarchy are reported in [Zanardi et al. 2023](https://arxiv.org/abs/2210.13064) [FT].
- **Navigation metrics.** Average ΔV, average delay and path deviation are named as typical. Makespan ratio and success rate also appear. — [Chandra, Singh, Luo, Sycara](https://arxiv.org/abs/2508.13459) [FT]
- **Aviation.** Reviews group results into safety, stability and efficiency, in fast-time simulation on the open BlueSky platform. — [Ribeiro, Ellerbroek, Hoekstra 2020](https://doi.org/10.3390/aerospace7060079) [SR]. The Domino Effect Parameter compares runs with and without conflict resolution. — [UAB record](https://portalrecerca.uab.cat/ca/publications/surrounding-traffic-complexity-analysis-for-efficient-and-stable--2/) [SR]. Delay and secondary conflicts are central in [Erzberger et al. 2010](https://icas.org/ICAS_ARCHIVE/ICAS2010/PAPERS/197.PDF) [FT]. Also standard: [Kuchar, Yang 2000](https://doi.org/10.1109/6979.898217) [META].
- **Execution.** Plans can be post-processed for robust execution with an action dependency graph and re-ordered in a receding horizon. — [Hönig et al. 2019](https://doi.org/10.1109/lra.2019.2894217) [AB], [Berndt et al. 2024](https://doi.org/10.1109/tro.2023.3344051) [AB], [Su, Veerapaneni, Li 2024](https://doi.org/10.1609/aaai.v38i16.29706) [SR]

### Inferences
- A defensible set for this paper:
  - Baselines: plan unresolved; simple rule as written, with fixes, and with random priority restarts; wait-only MILP as an exact reference on smaller maps; a CBS-family or PBS baseline if a Dubins lattice is built.
  - Metrics: maps conflict-free with confidence intervals; exposure in seconds; pairs left; closest approach; T_max, L_total and added delay against the no-interaction lower bound; runtime; budget and curvature violations; and the gap to the stage and mission optimum.
  - Reporting: paired per-map statistics, held-out seeds, and a note that seeds 1-10 flatter both methods (9 of 10 conflict-free, against 13 of 20 and 11 of 20 on seeds 11-30, per the README).

### Gaps
- I did not find a metric standard specific to multi-sink wireless sensor networks in this search. That scope sits with adjacent reviews.

## Does a stronger method with guarantees exist?

### Takeaway
Yes, in several forms, though no method gives unconditional conflict freedom under a mission budget, because the budget can make the instance infeasible. For the draft's setting, the most relevant is an exact timing MILP on the fixed routes. It can find a conflict-free schedule within its model or show that none exists.

### Cited Findings
- **Exact on fixed trajectories or paths.**
  - Start-time MILP with necessary and sufficient conditions, 20 robots. — [Akella, Hutchinson 2002](https://doi.org/10.1109/robot.2002.1013428) [FT]
  - Kinodynamic bounds via MILP, giving upper and lower bounds. — [Peng, Akella 2005](https://doi.org/10.1177/0278364905051974) [AB], with that description from [Altché et al.](https://arxiv.org/abs/1603.04610) [FT]
  - Resolution-optimal MILP. — [Altché et al.](https://arxiv.org/abs/1603.04610) [FT]
- **Coordination diagrams.** "Resolution-complete", and "complete for a large class of inputs", handling up to 10 robots in worst cases and more than 100 in practical ones. — [Siméon, Leroy, Laumond 2002](https://doi.org/10.1109/70.988973) [AB]. Earlier: [O'Donnell, Lozano-Pérez 1989](https://doi.org/10.1109/robot.1989.100033) [META], [LaValle, Hutchinson 1998](https://doi.org/10.1109/70.736775) [AB].
- **Discrete candidates plus ILP.** A yes-or-no answer for a fleet-wide conflict-free selection. — [Feurgard et al. 2026](https://arxiv.org/abs/2603.21789) [AB]
- **Prioritized with a proof.** Theorem 2 and RPP in valid infrastructures. — [Čáp et al. 2015](https://arxiv.org/abs/1409.2399) [FT]
- **Exact priority order.** Branch and Play. — [Hu et al. 2024](https://doi.org/10.15607/rss.2024.xx.116) [FT]
- **Search with optimality or completeness.** CCBS ([Andreychuk et al. 2022](https://doi.org/10.1016/j.artint.2022.103662) [SR]), K-CBS ([Kottinger et al. 2022](https://doi.org/10.1109/IROS47612.2022.9982018) [SR]), and with TSP sequencing, CBSS ([Ren et al. 2023](https://doi.org/10.1109/TRO.2023.3266993) [SR]).
- **Formal safety.** HJI reachability for two turning vehicles and priority-ordered sequential planning. — [Mitchell et al. 2005](https://doi.org/10.1109/TAC.2005.851439) [FT], [Chen et al. 2015](https://doi.org/10.1109/ecc.2015.7331044) [AB]
- **Size of the draft's problem.** About 15.5 sinks per map on seeds 1-10 (155 to 156 sinks over 10 maps, from `CLAUDE.md`), and 506 conflicting pairs over 30 maps.

### Inferences
- **Trivial guarantee, and why it does not settle the matter.** All sinks start and end in the docking zone, which is exempt from eq. 20. Applying Čáp's fall-back logic, sinks could wait there until earlier sinks return, which is conflict-free by construction. Total time would be the sum of the tours, which violates budget B except on small instances. The hard problem is therefore "conflict-free within B", and this also means a guarantee cannot be claimed without a feasibility condition. A formal link to Čáp's theorem would need care, because the shared base breaks the "one robot per endpoint" assumption unless the docking exemption is used to treat it as separate endpoints.
- **Suggested exact baselines, in order of fit:**
  1. A wait-only MILP over the draft's frames, with budget as a linear limit on each sink's total wait. It gives the minimum exposure achievable by timing alone, and shows which maps are impossible by timing. The problem size looks within reported ranges, but [slides on Akella and Hutchinson's method](https://www.sambuz.com/doc/coordinating-the-motions-of-multiple-robots-with-ppt-presentation-929838) [SR] warn that runtime depends on the number of collision-zone pairs, and tours can create many. I have not measured this.
  2. A pairwise ILP over the game's own strategy sets across the mission, following Feurgard. It gives the price-of-anarchy reference for exactly the draft's manoeuvres.
  3. CBSS or CCBS on a Dubins lattice for small maps, as the reference an AI-planning reviewer would expect.
- A MILP or ILP is not guaranteed to solve at this scale in time, and the problem is NP-hard per Akella and Hutchinson and Alejo et al. The game keeps a clear role as a fast method, measured against the exact result.

### Gaps
- I did not find a published theorem for velocity-tuned prioritized planning restricted to one fixed path, with a budget. I searched for this only through the works above.
- The feasibility of these baselines on the draft's maps has not been tested.

## What was searched for and not found, and what are the limits of this search?

### Takeaway
Searches were broad but not exhaustive. The most important negative results are that no exact potential game for aircraft or UAV conflict resolution with discrete manoeuvres was found, and no published "corridor game" of the draft's form. These are absences in this search, not proof of absence.

### Cited Findings
**Searched for and not found:**
- An exact potential game for aircraft or UAV conflict resolution with best response over discrete manoeuvres (queries: "potential game conflict resolution multi-UAV deconfliction", "aircraft conflict resolution potential game exact potential best response"). The results were other game types and communications-focused UAV potential games.
- A discrete GO / WAIT / DETOUR-style game for robots or AGVs ("multi-robot AGV path conflict resolution game theory wait detour").
- A named corridor game with Left / Right / Detour, or a narrow-passage potential game ("narrow passage two vehicles yield or go", "social mini-games narrow corridor").
- Game-theoretic ground-delay or wait-and-reroute deconfliction in UAS traffic management. The closest was non-game ground delay, [FAA-NASA vs. lane-based strategic deconfliction](https://users.cs.utah.edu/~tch/UTM/pub312.pdf) [SR].
- Seconds of exposure used as the shared cost in a game, and conflict-duration fitness in aviation genetic algorithms.
- A direct critique of game theory under a central planner, or a head-to-head comparison in cooperative multi-robot planning.
- A game-based result showing truncated cost leaves fewer conflicts.
- CBS specialised to Dubins kinematics, and fixed Dubins paths with explicit speed or stop scheduling.

**Limits of the search:**
- Tools were a US-only web search that returns summarised excerpts, page fetching, Crossref and OpenAlex lookups, and arXiv's API. Google Scholar and Semantic Scholar could not be queried directly.
- IEEE Xplore, ACM, ResearchGate, AIAA and core.ac.uk full texts were mostly unavailable (403 errors or paywalls), and the TU Delft repository refused connections. As a result, many IEEE, Elsevier and AIAA papers were checked only by abstract or metadata, as the tags show.
- Open-access and arXiv material is therefore over-represented. Coverage of JGCD, T-ITS and Automatica is limited. Work published very recently, including in 2026, may be missed.
- Excerpts returned by search are themselves summaries, so [SR] claims should be checked before citing.

**Verified paper list.** Tag = deepest level reached. Years are publication years; where Crossref shows a different date, it is noted.

| Authors | Year | Title (short) | Venue | DOI or URL | Tag |
|---|---|---|---|---|---|
| Merz | 1972 | Game of two identical cars | JOTA 9:324-343 | 10.1007/bf00932932 | META |
| Kant, Zucker | 1986 | Path-velocity decomposition | IJRR 5:72-89 | 10.1177/027836498600500304 | SR |
| Erdmann, Lozano-Pérez | 1987 | On multiple moving objects | Algorithmica 2:477-521 | 10.1007/BF01840371 | META |
| O'Donnell, Lozano-Pérez | 1989 | Deadlock-free coordination of two manipulators | ICRA | 10.1109/robot.1989.100033 | META |
| Fraichard, Laugier | 1993 | Dynamic trajectory planning, PVD and adjacent paths | IJCAI | ijcai.org/proceedings/1993/221 | SR |
| Monderer, Shapley | 1996 | Potential games | GEB 14:124-143 | 10.1006/game.1996.0044 | META |
| LaValle, Hutchinson | 1998 | Optimal motion planning, independent goals | IEEE T-RA 14:912-925 | 10.1109/70.736775 | AB |
| Tomlin, Pappas, Sastry | 1998 | Conflict resolution for ATM | IEEE TAC 43:509-521 | 10.1109/9.664154 | AB |
| Bicchi, Pallottino | 2000 | Optimal cooperative conflict resolution | IEEE T-ITS 1:221-231 | 10.1109/6979.898228 | FT |
| Kuchar, Yang | 2000 | Review of CD&R methods | IEEE T-ITS 1:179-189 | 10.1109/6979.898217 | META |
| Minderhoud, Bovy | 2001 | Extended TTC measures | Accid. Anal. Prev. 33:89-97 | 10.1016/S0001-4575(00)00019-1 | SR |
| Mao, Feron, Bilimoria | 2001 | Intersecting flows, decentralized rules | IEEE T-ITS 2:101-109 | 10.1109/6979.928721 | AB |
| Akella, Hutchinson | 2002 | Specified trajectories | ICRA 624-631 | 10.1109/robot.2002.1013428 | FT |
| Pallottino, Feron, Bicchi | 2002 | MIP for ATM conflicts | IEEE T-ITS 3:3-11 | 10.1109/6979.994791 | META |
| Siméon, Leroy, Laumond | 2002 | Path coordination, resolution-complete | IEEE T-RA 18:42-49 | 10.1109/70.988973 | AB |
| Peng, Akella | 2005 | Kinodynamic, specified paths | IJRR 24:295-310 | 10.1177/0278364905051974 | AB |
| Silver | 2005 | Cooperative Pathfinding | AIIDE 1:117-122 | 10.1609/aiide.v1i1.18726 | AB |
| van den Berg, Overmars | 2005 | Prioritized motion planning | IROS 430-435 | 10.1109/IROS.2005.1545306 | AB |
| Mitchell, Bayen, Tomlin | 2005 | Time-dependent HJ reachable sets | IEEE TAC 50:947-957 | 10.1109/TAC.2005.851439 | FT |
| Shanmugavel et al. | 2005, 2010 | Dubins sets; clothoid arcs | AIAA GNC; CEP 18:1084-1092 | 10.2514/6.2005-5827; 10.1016/j.conengprac.2009.02.010 | SR |
| Archibald et al. | 2008 | Satisficing conflict resolution | IEEE T-SMC-C 38:510-521 | 10.1109/tsmcc.2008.919162 | AB |
| Marden, Arslan, Shamma | 2009 | Cooperative control and potential games | IEEE T-SMC-B 39:1393-1407 | 10.1109/TSMCB.2009.2017273 | SR |
| Erzberger, Lauderdale, Chu | 2010; 2011 | Automated conflict resolution | ICAS; Proc. IMechE G 226:930-949 | icas.org/ICAS_ARCHIVE/ICAS2010/PAPERS/197.PDF; 10.1177/0954410011417347 | FT |
| Hand, Mao, Feron | 2011 | Sequential conflict resolution | ACC 2168-2173 | 10.1109/acc.2011.5991516 | AB |
| Chapman et al. | 2011 | Best-response DCOP framework | KER 26:411-444 | 10.1017/S0269888911000178 | SR |
| Bnaya et al. | 2013 (Crossref 2021) | MAPF for self-interested agents | SoCS | 10.1609/socs.v4i1.18292 | SR |
| Alejo et al. | 2013 (online 2012) | Velocity assignment problem | J. Intell. Robot. Syst. 69:331-346 | 10.1007/s10846-012-9768-4 | FT |
| Marden, Wierman | 2013 | Distributed welfare games | Oper. Res. 61:155-168 | econpapers.repec.org/RePEc:inm:oropre:v:61:y:2013:i:1:p:155-168 | SR |
| Zhu, Otte, Chaudhari, Frazzoli | 2014 | Game-theoretic controller synthesis I | ICRA 1646-1651 | 10.1109/icra.2014.6907072 | FT |
| Chaimatanan, Delahaye, Mongeau | 2014 | Hybrid metaheuristic, 4D trajectories | IEEE CIM 9:46-61 | 10.1109/mci.2014.2350951 | SR |
| Sharon et al. | 2015 | Conflict-based search | AIJ 219:40-66 | 10.1016/j.artint.2014.11.006 | SR |
| Čáp et al. | 2015 | Prioritized planning algorithms | IEEE T-ASE 12:835-849 | 10.1109/TASE.2015.2445780 | FT (arXiv) |
| Amir, Sharon, Stern | 2015 | MAPF as combinatorial auction | AAAI | 10.1609/aaai.v29i1.9427 | SR |
| Chen, Fisac, Sastry, Tomlin | 2015 | Safe sequential path planning | ECC 3304-3309 | 10.1109/ecc.2015.7331044 | AB |
| Altché, Qian, de La Fortelle | c. 2016 | Time-optimal coordination along paths | arXiv; IROS venue inferred, unconfirmed | arxiv.org/abs/1603.04610 | FT |
| Pritchett, Genton | 2018 | Negotiated decentralized conflict resolution | IEEE T-ITS 19(1) | see URL above | SR |
| Marden, Shamma | 2018 | Game theory and control | Annu. Rev. CRAS 1:105-134 | 10.1146/annurev-control-060117-105102 | AB |
| Li, Vorobeychik | 2018 | Path planning games | OptMAS workshop | URL above | FT |
| Li et al. | 2019 | MC-CBS, large agents | AAAI 33:7627-7634 | 10.1609/aaai.v33i01.33017627 | SR |
| Ma et al. | 2019 | Searching with consistent prioritization | AAAI 33:7643-7650 | 10.1609/aaai.v33i01.33017643 | SR |
| Stern et al. | 2019 (Crossref 2021) | MAPF definitions | SoCS 10:151-158 | 10.1609/socs.v10i1.18510 | FT (arXiv) |
| Hönig et al. | 2019 | Persistent, robust execution | RA-L 4:1125-1131 | 10.1109/lra.2019.2894217 | AB |
| Ho et al. | 2019; 2022 | MAPF for UTM | AAMAS; IEEE T-ITS 23:997-1008 | ifaamas.org/Proceedings/aamas2019/pdfs/p131.pdf; 10.1109/tits.2020.3019397 | SR |
| Fabiani, Grammatico | 2020 | Mixed-integer potential game | IEEE T-ITS 21:1064-1073 | 10.1109/TITS.2019.2901505 | AB |
| Ribeiro, Ellerbroek, Hoekstra | 2020 | Review of CR methods | Aerospace 7:79 | 10.3390/aerospace7060079 | SR |
| Wu, Bhattacharya, Prorok | 2020 | Prioritization by path prospects | ICRA 9809-9815 | 10.1109/icra40945.2020.9196813 | FT |
| Zanardi et al. | 2021 | Urban Driving Games | RA-L 6:4978-4985 | 10.1109/LRA.2021.3068657 | AB |
| Kavuncu, Yaraneri, Mehr | 2021 | Potential iLQR | RSS XVII | 10.15607/RSS.2021.XVII.084 | SR |
| Li et al. | 2021 | RHCR, lifelong MAPF | AAAI 35:11272-11281 | 10.1609/aaai.v35i13.17344 | FT |
| Wang et al. | 2021 | Game-theoretic planning, self-driving | IEEE T-RO 37:1313-1325 | 10.1109/TRO.2020.3047521 | META |
| Zanardi et al. | 2022 | Posetal Games | RA-L 7:1292-1299 | 10.1109/LRA.2021.3135030 | AB |
| Zanardi et al. | 2022 | Factorization of dynamic games | IROS 13159-13166 | 10.1109/iros47612.2022.9981416 | SR |
| Andreychuk et al. | 2022 | MAPF with continuous time | AIJ 305:103662 | 10.1016/j.artint.2022.103662 | SR |
| Wen, Liu, Li | 2022 | CL-MAPF | RAS 150:103997 | 10.1016/j.robot.2021.103997 | SR |
| Kottinger, Almagor, Lahijanian | 2022 | CBS, kinodynamic constraints | IROS 13494-13499 | 10.1109/IROS47612.2022.9982018 | SR |
| Li et al. | 2022 | MAPF-LNS2 | AAAI 36:10256-10265 | 10.1609/aaai.v36i9.21266 | SR |
| Miller, Mitra | 2022 | Differential games, lexicographic preferences | CDC 5751-5756 | 10.1109/cdc51059.2022.9992957 | FT |
| Williams, Chen, Mehr | 2023 | Distributed Potential iLQR | ICRA | 10.1109/ICRA48891.2023.10161176 | FT |
| Zanardi et al. | 2023 | How Bad is Selfish Driving? | RA-L 8:2293-2300 | 10.1109/lra.2023.3251845 | FT |
| Ren, Rathinam, Choset | 2023 | CBSS | IEEE T-RO 39:2669-2683 | 10.1109/TRO.2023.3266993 | SR |
| Chandra et al. | 2023 | SocialMAPF | RA-L 8:3214-3221 | 10.1109/lra.2023.3265169 | AB |
| Hoogendoorn et al. | 2023 | Game-theoretical multi-drone CR | arXiv; later SSRN versions; journal unconfirmed | arxiv.org/abs/2308.01069 | FT |
| Sun et al. | 2023 | Imagined potential games | arXiv | arxiv.org/abs/2310.01614 | AB |
| Liu, Su, Wu, Guo | 2023 | Multi-conflict-based, UAV | Drones 7:217 | 10.3390/drones7030217 | SR |
| Hu et al. | 2024 | Who Plays First? | RSS XX | 10.15607/rss.2024.xx.116 | FT |
| Moldagalieva et al. | 2024 | db-CBS | ICRA 14569-14575 | 10.1109/icra57147.2024.10610999 | SR |
| Su, Veerapaneni, Li | 2024 | Bidirectional TPG | AAAI 38:17559-17566 | 10.1609/aaai.v38i16.29706 | SR |
| Berndt et al. | 2024 | Receding horizon re-ordering | IEEE T-RO 40:1356-1372 | 10.1109/tro.2023.3344051 | AB |
| Fu et al. | 2024 | Hierarchical multi-UAV Dubins TSP | IEEE T-Cyb 54:123-135 | 10.1109/tcyb.2023.3265926 | SR |
| Lee, Peters, Fridovich-Keil | 2025 | Games of ordered preference | RA-L 10:7182-7189 | 10.1109/lra.2025.3575324 | FT |
| Chandra et al. | 2025 | Deadlock-free social mini-games | Auton. Robots 49 | 10.1007/s10514-025-10194-8 | META, plus FT project page |
| Chandra, Singh, Luo, Sycara | 2025-26 | SMG taxonomy | arXiv; Springer version listed | arxiv.org/abs/2508.13459 | FT |
| Feurgard et al. | 2026 | Wind-aware synchronous fleet | arXiv | arxiv.org/abs/2603.21789 | AB |
| Hall, Dörfler, Faulwasser | 2026 | Receding horizon games | arXiv | arxiv.org/abs/2605.12467 | AB |
| Sadovsky, Zelinski, Gorospe | 2026 | Dubins paths, traffic jam | J. Air Transp. 34:102-112 | 10.2514/1.D0525 | SR |
| Pradeep J et al. | 2026 | Loiter reinsertion, corridors | arXiv | arxiv.org/abs/2605.13822 | AB |
| Drones paper | 2026 | Cooperative online 3D planning | Drones 10:297 | 10.3390/drones10040297 | AB |
| Gafur et al. | 2022 (arXiv) | Non-cooperative multi-robot collision avoidance | arXiv | arxiv.org/abs/2103.00583 | FT |
| Sherali; Sherali, Soyster | 1982; 1983 | Equivalent lexicographic weights | EJOR 11:367-379; JOTA 39:173-186 | 10.1016/0377-2217(82)90202-8; 10.1007/bf00934527 | META |
| Koutsoupias, Papadimitriou | 1999 | Worst-case equilibria | STACS, LNCS | 10.1007/3-540-49116-3_38 | META |
| Anshelevich et al. | 2008 | Price of stability, network design | SICOMP 38:1602-1623 | 10.1137/070680096 | META |

### Inferences
- Before submission, the [SR] and [META] entries that carry weight in the argument should be read in full. Priority order: Zanardi 2021, Fabiani and Grammatico 2020, Kavuncu 2021, MAPF-LNS2, Peng and Akella 2005, Chaimatanan 2014, Sadovsky 2026.
- Items worth tracing further, not cited above: [Rosenthal 1973](https://doi.org/10.1007/BF01737559) and [Ui 2000](https://doi.org/10.1006/game.1999.0729) [META], for the wider theory of exact potentials.

### Gaps
- The Altché et al. venue is inferred from figure filenames only ("iros2016"), and the journal status of Hoogendoorn et al. is unconfirmed.
- Multi-sink wireless sensor network papers were not searched here, as that scope belongs to adjacent reviews.
