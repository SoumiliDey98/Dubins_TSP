# Closest prior art to the Dubins multi-sink pipeline: an adversarial search

Written 2026-11-06. The aim was to disprove the draft's novelty. No paper doing the whole pipeline was found, but **this search was cut short** (see the last section), and a failure to find a paper does not show that it is unpublished.

**Follow-up on 2026-11-09:** the priority list at the end was worked through in [follow_up_2026-11-09.md](follow_up_2026-11-09.md). Statements below that it changed were corrected and are marked "(2026-11-09)".

Evidence labels, used on every paper:

- **[FT]**: I read the relevant passages of the full text.
- **[FT*]**: the full text was read through the fetch tool's summariser, so re-check any wording before quoting it.
- **[ABS]**: I read the abstract, on the source page or in a metadata API record.
- **[SR]**: seen only in a search-result summary. It exists, but its content is second-hand.
- **[TITLE]**: title and bibliographic data only. Anything about its method is inferred from the title.
- **[UNVERIFIED]**: from memory, not confirmed in this session.
- **[REPO]** (added 2026-11-09): read in the authors' public code repository README, not in the paper.

## Which published papers are closest to the combined pipeline, ranked?

### Takeaway
No paper found combines all six parts. The closest overall is Huang & Savkin 2017, in the same domain (Dubins mobile sinks, WSN, obstacles, several sinks). The closest recent papers are Fu et al. 2026 (T-ITS) and Wang et al. 2026. Wang et al. already appears to do Dubins-based, obstacle-aware spectral partitioning at abstract level. None of these has a game or any stated resolution of conflicts between vehicles on their routes.

### Cited Findings

**Ranked against the whole pipeline.** "Lacks" means I saw no sign of it at the evidence level given, not that the paper certainly omits it.

| # | Paper | Evidence | Has | Lacks or differs |
|---|---|---|---|---|
| 1 | Huang & Savkin 2017, *Computer Communications*, [DOI](https://doi.org/10.1016/j.comcom.2017.07.010); method read in [arXiv report 1712.02436, ch. 4](https://arxiv.org/abs/1712.02436), which has the identical title | [FT] of the report chapter; journal version not opened | Mobile sinks modelled as a Dubins car in a WSN; obstacles; tangent-graph roadmap; ATSP ordering; dynamic-programming path search; contact time at sensors; k sinks minimising the longest path (k-SVPP) | k is given, not minimised; no time budget. Partition is a split of one long tour, not spectral. A "viable path" is defined as collision-free "with sensor nodes/base station and obstacles", with no clause for other sinks. Tested on n = 10-50, k = 3 |
| 2 | Fu, Sun, Yao, Wu, Wu 2026, *IEEE T-ITS*, [DOI](https://doi.org/10.1109/tits.2025.3645094) | [ABS] | "Data collection Dubins TSP" for heterogeneous multi-UAV IoT; 3-D complex obstacles; multi-height clustering; ant colony system; probabilistic roadmap for obstacle distance; time-synchronous Dubins recovery | No game. Abstract states no en-route conflict resolution. Clustering is not spectral. Pareto, not lexicographic. Fleet-size choice not stated |
| 3 | Wang, Shen, Ma 2026, IRCE, [DOI](https://doi.org/10.1109/irce70780.2026.11739077) | [ABS] | Multi-USV; Dubins-based feasible trajectories; **a node graph using travel time instead of Euclidean distance, with obstacle avoidance; target assignment as graph partitioning by spectral clustering**, balancing load and compactness; simulated annealing for order; no-sail zones | No game. No budget-driven recursion stated. Reconnaissance, not data collection. Has ocean currents. Affinity details not read |
| 4 | Fu, Sun, Liu, Yao, Wu 2024, *IEEE T-Cybernetics*, [DOI](https://doi.org/10.1109/tcyb.2023.3265926) | [ABS] | Hierarchical multi-UAV Dubins TSP; 3-D complex obstacles; clustering to cluster centres; adaptive roadmap; ant colony sequencing; Dubins smoothing after the order is fixed | Not data collection; no game; clustering objective is distance to cluster centres (whether geodesic is not stated) |
| 5 | Cai, Zhang, Zheng 2017, *Sensors*, [DOI](https://doi.org/10.3390/s17071607) | [FT] | Multi-AUV MTSP; GA using **Euclidean fitness, then mapped to Dubins curves**; tour-balance criteria; a post-hoc collision check, and "another sub-optimal paths can be generated with GA if there is a collision" | No obstacles or inter-AUV avoidance ("in our future work"). 10-50 targets, 2-5 AUVs |
| 6 | Li, Gan, Pang, Zhu 2024, *Sensors*, [DOI](https://doi.org/10.3390/s24196345) | [ABS], part [FT] | Workload-balanced SOM assignment; Dubins paths; static obstacles; **re-assignment when planning fails** | No inter-vehicle conflict resolution found in a keyword scan; not spectral or GA |
| 7 | Gottlieb, Manathara, Shima 2018, *J. Intell. Robot. Syst.* 90, [DOI](https://doi.org/10.1007/s10846-017-0684-5); [catalogue record](https://bvbvf1.bib-bvb.de/vufind/PrimoRecord/cdi_proquest_journals_2039707471) | [SR] | Dubins and Reeds-Shepp vehicles; polygonal obstacles; visibility-graph distances; GA. The draft already cites it | One vehicle |
| 8 | Edison & Shima 2011, *Computers & OR* 38(1), [record](https://cris.iucc.ac.il/en/publications/integrated-task-assignment-and-path-optimization-for-cooperating-/) | [SR] | Multi-UAV; Dubins; **finite set of visitation angles turning the problem into a graph**; GA | No obstacles or conflict game shown in the summary |
| 9 | Wichmann & Korkmaz 2015, *Computer Communications*, [DOI](https://doi.org/10.1016/j.comcom.2015.06.001); [partial abstract](https://www.aminer.org/pub/573695c46e3b12023e4de2fc/smooth-path-construction-and-adjustment-for-multiple-mobile-sinks-in-wireless-sensor) | [SR] | TSP-based smooth paths with contact-time adjustment, for multiple mobile sinks. Huang's report (its ref. 48) says it follows the Dubins TSP idea with "a turning circle model", and that robots "may collide with sensor nodes" ([report](https://arxiv.org/abs/1712.02436)) | No sign of obstacles or inter-sink conflict |
| 10 | Wu, Zhang, Le, Lu 2025, *Scientific Reports*, [DOI](https://doi.org/10.1038/s41598-025-09753-x) | [FT] | Heterogeneous UAVs; 3-D Dubins; GA coding approach angles; **a UAV waits by flying a closed holding loop at minimum turning radius**; "deadlock" | Its deadlock means cycles in task timing, not physical conflict. Keyword scan found no static obstacles or separation between UAVs. Minimises total length |
| 11 | Nayak & Rathinam 2023, *Sensors*, [DOI](https://doi.org/10.3390/s23146432) | [FT] | MinMax Dubins mTSP; **h = 8 candidate headings per city**; MILP, VNS, learned policy | No obstacles ("Future work can address..."); no conflict handling found |
| 12 | Cho & Choi 2018, [arXiv](https://arxiv.org/abs/1808.02720) | [ABS] | Heterogeneous Dubins vehicles; memetic algorithm; objective mixes total and longest tour | Remote tasks; abstract names no obstacles or collisions |
| 13 | Liu, Lu, Xie, Ji, Zheng 2025, [arXiv](https://arxiv.org/abs/2506.02365); listed as [*Drones* 10(2)](https://doi.org/10.3390/drones10020132) | [FT*] | K-means clustering; Dubins distance costs; several UAVs | "obstacles and hazardous areas are currently not taken into account"; collisions assumed away by altitude layering |
| (2026-11-09) | Kumar & Mukherjee 2026, COMSNETS, [DOI](https://doi.org/10.1109/comsnets67989.2026.11418194) | [ABS] | WSN mobile sinks; **minimises the number of sinks** under a per-sink length limit L; tours avoid obstacles; approximation algorithm with analysis; static and mobile sensors | No kinematics, dwell or conflicts between sinks in the abstract. Not ranked against the others, because only the abstract was read |
| (2026-11-09) | Xu et al. 2023, *IEEE T-ASE* 21(4), [DOI](https://doi.org/10.1109/tase.2023.3336076); [code](https://github.com/wuuya1/LRGO) | [ABS], [REPO] | Non-holonomic fleet; distributed task assignment; distributed motion planning that handles deadlocks, with ORCA among its planners; dense environments; 50 vehicles and 203 targets | Not data collection; reactive avoidance rather than resolving conflicts on planned timed tours; no budget or fleet minimisation stated |

Further Dubins routing work, less close:

- [Macharet et al. 2012](https://doi.org/10.1109/cec.2012.6256477) [ABS]: one nonholonomic data-collecting node in a WSN; metaheuristic; heading heuristic.
- [Sundar et al. 2019](https://arxiv.org/abs/1912.04353) [ABS]: team orienteering with fixed-wing drones, exact branch-and-price.
- [Zahrádka et al. 2019](https://doi.org/10.1007/978-3-030-14984-0_27) [TITLE]: UAV teams, Dubins model, budget constraint.
- [Gao et al. 2024](https://doi.org/10.1109/icceic64099.2024.10775515) [ABS]: GA for the Dubins mTSP with neighbourhoods.
- [Karapetyan et al. 2018](https://arxiv.org/abs/1808.02552) [ABS]: multi-robot Dubins coverage with k-TSP and clustering, tested on real boats.
- [Cai & Zhang 2016](https://doi.org/10.1049/el.2016.3836) [TITLE] and [2018](https://doi.org/10.3390/s18072105) [FT]: 3-D Dubins for a mobile sink in underwater networks.
- [Kumar & Mukherjee 2023](https://doi.org/10.1016/j.adhoc.2023.103239) [TITLE], cited by the draft: vehicle-aided collection from *mobile* sensors with obstacle avoidance. I could not read its abstract, so I do not know whether it models curvature.

**Ranked against the game (f) alone.** None of these is about Dubins routing.

1. [Jordán et al. 2021](https://ideas.repec.org/a/spr/grdene/v30y2021i1d10.1007_s10726-020-09703-0.html), *Group Decision and Negotiation* 30(1) [ABS, fetch tool]: FENOCOP is a two-level game. One level picks a Nash equilibrium among plan combinations, and the other adds delays so plans do not conflict. This is the closest structure to "fixed plans, resolve conflicts with WAIT", but it is about abstract multi-agent plans.
2. [Imran et al. 2026](https://arxiv.org/abs/2604.13708) [ABS]: a potential game that "leverages homotopy classes as structured strategy sets". Strategies are topologically distinct routes, which is the closest analogue to DETOUR.
3. [Liu et al. 2022](https://arxiv.org/abs/2201.06157) [ABS]: finite and continuous potential games for autonomous driving, with existence of pure Nash equilibria, convergence, and best-response dynamics.
4. [Meng, Huang, Ma 2024](https://arxiv.org/abs/2409.16190) [ABS]: mixed-integer potential game on a way-point graph for road vehicles.
5. [Mai et al. 2025](https://arxiv.org/abs/2511.12160) [ABS]: dynamic potential game with iterated best response and finite-step convergence to an epsilon-Nash equilibrium.
6. [Thabet et al.](https://doi.org/10.1109/access.2024.3515006), *IEEE Access* [ABS]: UAV base-station mobility "formulated as an exact potential game", with a collision-avoidance mechanism. The year is given as 2024 by OpenAlex and 2025 by Semantic Scholar. The goal is interference and coverage, not routing.
7. [Li & Vorobeychik, "Path Planning Games"](https://www2.isye.gatech.edu/~fferdinando3/cfp/OPTMAS18/papers/paper_17.pdf) [FT], apparently OptMAS 2018 from its URL: best-response dynamics over paths. It finds "safety is often significantly compromised compared to a cooperative solution".
8. Others:
   - [S. Li et al. 2022](https://arxiv.org/abs/2212.04122) [ABS, fetch tool]: MDP congestion game for air traffic.
   - [Hoogendoorn et al. 2023](https://arxiv.org/abs/2308.01069) [ABS, fetch tool]: differential game for drones.
   - [Alghamdi & Park 2025](https://arxiv.org/abs/2504.20947) [ABS, fetch tool]: a "multi-robot navigation game" in narrow corridors, using opinion dynamics.
   - [Ren et al. 2025](https://doi.org/10.1007/978-981-96-0789-1_2) [TITLE]: potential-game matching in homicidal-chauffeur pursuit games, the only "potential game" + Dubins abstract hit, and a different problem.

**Ranked against the partition (b) alone.**

1. [Wang et al. 2026](https://doi.org/10.1109/irce70780.2026.11739077) [ABS], as above.
2. [Karunakar et al. 2026](https://doi.org/10.1109/iccsc67078.2026.11468635) [ABS]: spectral clustering plus RRT for an obstacle-aware mobile sink. Spectral clustering chooses rendezvous points, with one sink and no curvature stated.
3. [Niranjan et al. 2026](https://doi.org/10.1109/iceccc70334.2026.11633036) [ABS]: spectral clustering, RRT*, ACO and DRL for obstacle-aware sink trajectories, compared against EDMSP and RRTMT.
4. [Fei et al. 2022](https://doi.org/10.1109/bigdia56350.2022.9874072) [ABS]: spectral task grouping for multi-UAV delivery, with flight-distance constraints.
5. [Faigl et al. 2012](https://imr.ciirc.cvut.cz/uploads/Papers-pdfs/Faigl12iros.pdf) [SR]: cluster-first route-second MTSP, arguing that real distances are geodesic because of obstacles.
6. [Karmakar 2026](https://doi.org/10.21203/rs.3.rs-8604869/v1) [TITLE], preprint: GA with spectral graph cuts for the TSP.
7. An uncited lead: [Aravind et al. 2026](https://doi.org/10.3390/s26206444) [FT] summarises an earlier sink paper using "spectral clustering... and Dijkstra's algorithm" for partitioning and sink trajectories. The reference number was lost in extraction, so the paper is unidentified.

### Inferences
- Huang & Savkin must be cited and compared against. Its report also states a criticism that applies to the draft: "many existing approaches set the sensor nodes' locations as target positions for the robots... which leads to the collisions with the sensor nodes". As I read the draft and code, sinks do drive to the sensor positions.
- The draft's reference list has five entries and contains none of ranks 1-4. Section 0.9 also names "NPA" as the traditional method with no citation. A reviewer would read this as a missing literature review.

### Gaps
- I read Fu 2026, Wang 2026 and Fu 2024 only as abstracts. Their full texts could contain inter-vehicle handling, or a geodesic or radius-scaled affinity, that would change this ranking. Wang 2026 is the most urgent to read.

## Is there any paper combining two or more of: Dubins multi-vehicle routing, obstacle-aware geodesic clustering, game-based deconfliction, WSN mobile sinks?

### Takeaway
Three of the six pairs are already published, and so is one triple. No paper was found for any pair or triple that includes the game alongside Dubins routing or WSN sinks. All such "not found" entries have the search limits listed at the end.

### Cited Findings

| Combination | Status | Evidence |
|---|---|---|
| Dubins routing + WSN sinks | **Exists** | [Huang & Savkin 2017](https://doi.org/10.1016/j.comcom.2017.07.010), [Fu et al. 2026](https://doi.org/10.1109/tits.2025.3645094) (IoT), [Wichmann & Korkmaz 2015](https://doi.org/10.1016/j.comcom.2015.06.001) |
| Dubins routing + obstacle-aware clustering | **Exists** | [Wang et al. 2026](https://doi.org/10.1109/irce70780.2026.11739077) is the strongest; [Fu et al. 2024](https://doi.org/10.1109/tcyb.2023.3265926) is partial |
| Obstacle-aware clustering + WSN sinks | **Exists** (spectral, not stated as geodesic) | [Karunakar et al. 2026](https://doi.org/10.1109/iccsc67078.2026.11468635), [Niranjan et al. 2026](https://doi.org/10.1109/iceccc70334.2026.11633036) |
| Dubins routing + clustering + WSN (triple) | **Exists in substance** | [Fu et al. 2026](https://doi.org/10.1109/tits.2025.3645094) (clustering plus obstacle-avoiding roadmap, not geodesic spectral) |
| Dubins routing + game deconfliction | Not found | Nearest Dubins work coordinates by timing without a game: [Fu 2026](https://doi.org/10.1109/tits.2025.3645094), [Wu 2025](https://doi.org/10.1038/s41598-025-09753-x). Dubins game hits are pursuit-evasion ([Ruiz](https://arxiv.org/abs/2406.08637) [TITLE], [Ren 2025](https://doi.org/10.1007/978-981-96-0789-1_2)) |
| Clustering + game deconfliction | Not found | Nothing surfaced |
| Game deconfliction + WSN sinks | Not found | [Raj et al. 2020](https://doi.org/10.1007/s11276-020-02254-x) [SR] uses game theory with ACO for a mobile sink. (2026-11-09) The game selects rendezvous points and ACO selects the route, so it does not resolve conflicts between sinks. A Europe PMC full-text hit, a [topology-control paper](https://doi.org/10.1016/j.mex.2023.102423) [TITLE], looks irrelevant from its title |
| Any triple with the game | Not found | Nothing surfaced |
| All four, or the full pipeline | Not found | Nothing surfaced |

**Component overlap beyond the four named pillars:**

- (a) Feasible graph and geodesic cost: tangent graph in [Huang](https://arxiv.org/abs/1712.02436); visibility-graph distances in [Gottlieb 2018](https://bvbvf1.bib-bvb.de/vufind/PrimoRecord/cdi_proquest_journals_2039707471) [SR].
- (c) GA with a straight-line surrogate: Euclidean fitness then Dubins in [Cai 2017](https://doi.org/10.3390/s17071607). A surrogate model for evaluating Dubins tours appears in [Drchal et al. 2020](https://arxiv.org/abs/2002.00811) [TITLE].
- (d) Discrete headings: [Edison & Shima 2011](https://cris.iucc.ac.il/en/publications/integrated-task-assignment-and-path-optimization-for-cooperating-/) [SR]; [Nayak & Rathinam 2023](https://doi.org/10.3390/s23146432) [FT]; DP in [Huang](https://arxiv.org/abs/1712.02436) [FT].
- (e) Objectives: longest path in [Huang](https://arxiv.org/abs/1712.02436) and [Nayak](https://doi.org/10.3390/s23146432); total plus longest in [Cho & Choi](https://arxiv.org/abs/1808.02720); Pareto in [Fu 2026](https://doi.org/10.1109/tits.2025.3645094). I found no paper using exactly (K, T_max, L_total) lexicographically in a Dubins setting.
- Repair: regenerate on collision in [Cai 2017](https://doi.org/10.3390/s17071607) (one sentence); re-assign on planning failure in [Li 2024](https://doi.org/10.3390/s24196345).

### Inferences
- The strongest open pair is Dubins routing + game deconfliction. The novelty is in the combination, because each half is well established on its own.
- Fleet-size-first objectives are, from memory, standard in vehicle routing with time windows **[UNVERIFIED]**, so the lexicographic objective is unlikely to count as a contribution.

### Gaps
- (2026-11-09) Raj et al. 2020 is resolved only at search-summary level: the publisher page still redirects to a login.

## What did the direct phrase searches return?

### Takeaway
The draft's coined terms ("radially unimodal", "curvature-aware partitioning", "CorridorGame") have no relevant hits in the indexes tried. The underlying ideas do have neighbours, though, so a coined name protects nothing.

### Cited Findings
- **"radially unimodal"**: 0 hits on the arXiv API and Europe PMC. One hit on OpenAlex, [an image-deblurring paper](https://doi.org/10.1007/978-3-642-38267-3_8) [TITLE], which is unrelated.
- **"curvature-aware" + partitioning**: 13 OpenAlex and 3 arXiv hits, all unrelated (federated learning, meshes, for example [this one](https://doi.org/10.26599/air.2026.9150004)). 0 for "curvature-aware" clustering vehicle.
- **"corridor game" / "CorridorGame"**: 9 OpenAlex hits, all unrelated, such as [the China-Pakistan Economic Corridor](https://doi.org/10.1017/s0305741025000189). Nearest robotics work: [Alghamdi & Park 2025](https://arxiv.org/abs/2504.20947); [Chandra et al. 2023](https://iros2023-madgames.f1tenth.org/papers/chandra.pdf) [SR]. The only exact use I saw is this project's own [public repository](https://github.com/SoumiliDey98/Dubins_TSP), whose file list includes `theory_claude/`.
- **"potential game" + Dubins**: 1 OpenAlex hit ([Ren 2025](https://doi.org/10.1007/978-981-96-0789-1_2)), 0 on the arXiv API and in Europe PMC full text.
- **"spectral clustering" + Dubins**: 1 OpenAlex hit, which is [Wang et al. 2026](https://doi.org/10.1109/irce70780.2026.11739077). This is the most damaging single result. 0 on the arXiv API and Europe PMC.
- **Dubins + "mobile sink(s)"**: 1 OpenAlex hit ([Cai & Zhang 2016](https://doi.org/10.1049/el.2016.3836)), 1 in Europe PMC ([Cai & Zhang 2018](https://doi.org/10.3390/s18072105)). The web search found [Huang's report](https://arxiv.org/abs/1712.02436), which these indexes had missed.
- **Dubins + "multiple traveling salesman"**: 15 OpenAlex hits, including [Ernest & Cohen 2012](https://doi.org/10.2514/6.2012-2562) [TITLE], [Zhu et al. 2022](https://doi.org/10.1109/rcae56054.2022.9995954) [TITLE], Gao 2024 and Nayak 2023. With "obstacles": 8.
- **"radial" tours as a concept**: my one web search for pyramidal, depot-radial or petal tours returned only generic GA papers. From memory, **pyramidal tours** (Gilmore, Lawler, Shmoys 1985), **bitonic tours**, and the **sweep and petal** heuristics (Gillett & Miller 1974; Foster & Ryan 1976) are the obvious neighbours **[all UNVERIFIED]**.

### Inferences
- A radially unimodal tour looks to me like a pyramidal tour with cities ranked by distance from the base. If that holds, a reviewer may say the seed structure is known. The defensible new part would then be using it as a proxy for Dubins turning cost, with the pi * R per-peak penalty. Check this against the pyramidal-tour definition before writing it.

### Gaps
- The neighbours above were not verified, because the search budget ran out. They should be confirmed before they are cited.

## What do surveys from 2020 to 2026 say, and do they name this as a gap?

### Takeaway
No survey seen names this combination. They state general gaps (integration, kinematics, obstacles, multi-vehicle coordination) that support the motivation but not a claim that this exact gap is recognised. The strongest verbatim statements I read come from primary papers' future-work sections, not surveys.

### Cited Findings

**Read in the source:**

- [Arshid et al. 2025](https://doi.org/10.3390/s25185877), *Sensors* [FT]: "A comprehensive framework is needed in the future that integrates all aspects, such as multi-agent coordination, real-time adaptation, explainability, and safety into a unified structure." On Dubins: "this model has limitations; it can only handle straight, constant-radius turns, making it less suitable for dynamic replanning."
- [Nayak & Rathinam 2023](https://doi.org/10.3390/s23146432) [FT], not a survey: "due to the absence of pre-existing baselines for the Dubins MTSP, it was not possible to determine the optimality gap"; "Future work can address factors related to planning paths in the presence of obstacles".
- [Cai et al. 2017](https://doi.org/10.3390/s17071607) [FT], not a survey: "in our future work, we will study how to avoid multiple underwater obstacles and inter-AUV collision".
- [Liu et al. 2025](https://arxiv.org/abs/2506.02365) [FT*], not a survey: "Future work will focus on resolving potential conflicts caused by obstacles".
- [Huang's report](https://arxiv.org/abs/1712.02436) [FT], 2017: "the dynamic constraint of robots is rarely taken into account in recent algorithms". This is older than the requested window.
- [Lin et al. 2026](https://arxiv.org/abs/2610.05832) [ABS], not a survey: interactions "are routinely replaced by a potential game, i.e. by a common objective". This is evidence **against** novelty for the game framing.

**Seen only through search-result summaries. Check the wording before quoting any of these:**

- [Systematic review on multi-robot task allocation](https://dl.acm.org/doi/full/10.1145/3700591), ACM: lists "nonconvex obstacle avoidance", and says collision-free path planning for heterogeneous teams "is still an open challenge". The page returned 403.
- [Review of task assignment and path planning](https://www.sciencedirect.com/science/article/abs/pii/S1568494625018113), 2025: "still an open-ended research area"; most prior work was "restricted to either task allocation or path planning". The page was not opened.
- [Survey on UAV-based data gathering in WSNs](https://www.researchgate.net/publication/394595355_A_Comprehensive_Survey_on_UAV-based_Data_Gathering_Techniques_in_Wireless_Sensor_Networks), 2025: open challenges "including trajectory planning, data collection and obstacle avoidance strategies". The page was not opened.
- [UAV-assisted data collection survey](https://www.mdpi.com/2079-9292/10/21/2603), *Electronics* 2021: coordination among UAVs poses challenges. The page returned 403.
- [Multiple mobile robot task and motion planning survey](https://dl.acm.org/doi/10.1145/3564696), ACM: defines trajectory planning as feasible timed paths accounting for collisions and kinematics. The summary's open questions do not name this combination.
- Listed but not read for gaps: [anti-collision survey, Wei et al.](https://arxiv.org/abs/2109.12832) [ABS]; [orienteering survey, Shen et al. 2025](https://arxiv.org/abs/2512.16865) [ABS], which includes Dubins OP in its taxonomy; [UAV task assignment survey](https://www.sciencedirect.com/science/article/pii/S221420962200016X) [SR]; [WSN mobile-sink path review](https://www.sciencedirect.com/science/article/abs/pii/S1574013721000526) [SR]; [Temene et al. 2022](https://doi.org/10.1016/j.adhoc.2021.102726), the draft's own reference, not opened, with its DOI taken from the draft.

### Inferences
- A gap sentence in the paper can honestly say that surveys ask for integration of allocation and path planning, and that Dubins routing papers defer obstacles and inter-vehicle conflicts to future work. It should not say that surveys identify a Dubins + spectral + game gap.

### Gaps
- (2026-11-09) Macharet & Campos, ["A survey on routing problems and robotic systems", *Robotica* 2018](https://doi.org/10.1017/s0263574718000735), exists [TITLE]. A newer [seventy-year review of Dubins path planning (*Industrial Robot* 2026)](https://doi.org/10.1108/ir-07-2025-0265) [ABS] says that "obstacle constraints and real-time responsiveness remain active challenges". Neither was read in full.

## Could a very recent paper (2023 to 2026) scoop the draft?

### Takeaway
Nothing found scoops the whole pipeline, but scoop risk is real and rising in two areas: Dubins multi-UAV data collection with obstacles, and spectral partitioning with Dubins or obstacle-aware costs. Parts (a), (b) and (d) are already partly taken by 2024-2026 papers.

### Cited Findings
- **Highest-risk line:** the Fu / Sun / Yao / Wu papers moved from [hierarchical Dubins TSP with obstacles (2024)](https://doi.org/10.1109/tcyb.2023.3265926) to [data-collection Dubins TSP with obstacles (2026)](https://doi.org/10.1109/tits.2025.3645094). Semantic Scholar shows 7 citations of the 2026 paper, which I did not get to list.
- **Spectral + Dubins or obstacles, all 2026:** [Wang et al.](https://doi.org/10.1109/irce70780.2026.11739077), [Karunakar et al.](https://doi.org/10.1109/iccsc67078.2026.11468635), [Niranjan et al.](https://doi.org/10.1109/iceccc70334.2026.11633036).
- **Cooperative Dubins planning:** [Drones 10(4):297 (2026)](https://doi.org/10.3390/drones10040297) [SR], "Cooperative-3D-Quick-Dubins-RRT*", with multi-UAV obstacle and inter-agent avoidance but no routing shown. [Wu et al. 2025](https://doi.org/10.1038/s41598-025-09753-x) [FT]. [Li et al. 2024](https://doi.org/10.3390/s24196345).
- **Potential games for motion, 2022-2026, active and growing:** [Imran 2026](https://arxiv.org/abs/2604.13708), [Mai 2025](https://arxiv.org/abs/2511.12160), [Meng 2024](https://arxiv.org/abs/2409.16190), [Williams 2023](https://arxiv.org/abs/2303.04842), [Bhatt 2022](https://arxiv.org/abs/2206.08963). My local filter of 467 arXiv "potential game" records found no Dubins or multi-vehicle tour-planning paper, judged from the titles of the entries that mention vehicles, robots or trajectories and from a sample of abstracts.
- **Kinodynamic multi-robot alternatives to a game:** [db-CBS](https://arxiv.org/pdf/2309.16445) [SR], [db-LaCAM 2025](https://arxiv.org/pdf/2512.06796) [SR], [CBS as a protocol 2025](https://arxiv.org/abs/2510.00425) [SR], [AOC-CBS 2026](https://arxiv.org/pdf/2608.08175) [SR]. These address collision-free kinodynamic planning. The [single-robot KRRF 2025](https://arxiv.org/abs/2505.06126) [ABS] addresses unknown visit order with kinodynamics.
- **Adjacent recent titles:** [Pseudo-trajectory homotopy for UMV data collection, IoT-J 2025](https://doi.org/10.1109/jiot.2025.3596338) [TITLE], which shares a Jinyu Fu author name; [Multi-AAV moving-target planning, T-SMC 2026](https://doi.org/10.1109/tsmc.2026.3678144) [TITLE]; [Dubins moving-target TSP with obstacles](https://www.researchgate.net/publication/397926930_Lazy_Anytime_Planning_for_the_Dubins_Moving_Target_Traveling_Salesman_Problem_with_Obstacles) [SR].

### Inferences
- If the Fu group adds en-route conflict resolution, the application-level overlap would be large. This is my speculation from their progression, not something they have stated.
- The window for a "spectral + Dubins" claim has probably closed. Any partition claim now has to be narrower.

### Gaps
- IEEE conference and journal content from 2026 is reachable mostly through metadata indexes, so very recent paywalled work may have been missed.

## Which venues publish this, and what do accepted papers show in their results?

### Takeaway
The closest work appears in T-ITS, T-Cybernetics, *Computer Communications*, *Ad Hoc Networks*, *Sensors*, *Drones* and *Computers & OR*. In the papers I could read, results sections report means and percentage gaps with few or no significance tests, so paired statistics and ablations would help the draft stand out. The draft's instance size is already at or above most of what I read, though this is a small sample.

### Cited Findings

| Paper and venue | Baselines | Instance size | Statistics |
|---|---|---|---|
| [Huang's report ch. 4](https://arxiv.org/abs/1712.02436) (*Computer Communications* version not opened) [FT] | Viable k-SPLITOUR; multihop shortest-path routing for energy | n = 10-50; k = 3 | 20 independent instances per setting; means; k-SVPP 5.5 % better on average; about 95 % energy saving against multihop |
| [Nayak & Rathinam](https://doi.org/10.3390/s23146432), *Sensors* [FT] | CPLEX MILP, VNS variants, learned policy | Modified TSPLIB, 16-176 cities; m = 3; h = 8 | Best MILP gap 14.09 % on a 22-city instance; no feasible MILP solution at 52 cities or more; averages over instances |
| [Cai et al. 2017](https://doi.org/10.3390/s17071607), *Sensors* [FT] | Single-AUV 3-D Dubins TSP; alternating algorithm excluded | 10-50 targets; 2-5 AUVs; field 20 x 20 x 10 turning radii | GA settings given; no significance test seen |
| [Liu et al. 2025](https://arxiv.org/abs/2506.02365), arXiv, listed in *Drones* [FT*] | Simulated annealing as benchmark; greedy, Hungarian, auction; own version without clustering | Mostly 15-25 tasks; 4 UAVs | 50 simulations; means and gaps only; no variance or tests |
| [Wu et al. 2025](https://doi.org/10.1038/s41598-025-09753-x), *Sci. Rep.* [FT] | Basic GA; decoupled method (+29.83 % length) | 5 UAVs in the parameter table | Parameter sensitivity; a keyword scan found no significance test |
| [Aravind et al. 2026](https://doi.org/10.3390/s26206444), *Sensors* [FT] | Several learned and routing baselines; ablation | Not extracted | Five independent runs with named seeds |
| [Karunakar 2026](https://doi.org/10.1109/iccsc67078.2026.11468635), [Niranjan 2026](https://doi.org/10.1109/iceccc70334.2026.11633036), conferences [ABS] | EDMSP, RRTMT ([Boyineni et al. 2023](https://doi.org/10.1007/s12652-023-04717-3) [SR]) | Not stated in abstract | Percentage improvements only |
| [Fu 2024](https://doi.org/10.1109/tcyb.2023.3265926), [Fu 2026](https://doi.org/10.1109/tits.2025.3645094) [ABS] | Not stated in abstract | Not stated | "Simulation experiments", Pareto solutions |

Other venues seen with related work, with no editorial policy checked: [*IEEE Access*](https://doi.org/10.1109/access.2021.3102317), [*Wireless Networks*](https://doi.org/10.1007/s11276-020-02254-x), [*IEEE RA-L*](https://doi.org/10.1109/lra.2022.3194688), [*J. Intell. Robot. Syst.*](https://doi.org/10.1007/s10846-017-0684-5), [*IEEE IoT Journal*](https://doi.org/10.1109/jiot.2025.3596338).

**The draft's current position**, from this repository's [README](../../Dubins_TSP_New_Implementation_claude/README.md):

- 93-109 sensors and 14-16 sinks per map, on seeds 1-10.
- Resolver comparison on seeds 1-30: Wilcoxon p < 0.001 for the resolver alone, and p = 0.06 with the repair step.

### Inferences
What the draft should show to be credible. This is my judgement, not a venue rule.

1. **Ablations for every claimed part:** unimodal seeding off, pi * R penalty off, Euclidean instead of geodesic affinity, sigma not tied to R, game versus simple rule, repair off. A part that makes no measurable difference should not be claimed.
2. **Comparison with published methods, reimplemented:** k-SVPP (Huang), travel-time spectral (Wang), K-means + Dubins (Liu), and VNS MinMax (Nayak) for routing. Priority planning and a CBS-style kinodynamic planner for conflicts.
3. **A public benchmark:** Nayak's modified TSPLIB is the obvious one, plus a small MILP or lower-bound check, since all current maps come from one generator.
4. **Reporting:** all maps, not just seeds 1-10, since the README says these "flatter both methods". Paired tests, effect sizes and runtimes.
5. **Reproducible numbers:** the README says the seeds 1-30 comparison came from "scripts that are not in the repository". Those scripts should be added before the numbers are published.

### Gaps
- I did not read author guidelines, acceptance rates or reviewer expectations for any venue. Three of the closest papers (Huang journal version, Fu 2024, Fu 2026) were not read in full, so their results columns are incomplete.

## Which combination claims would survive review?

### Takeaway
"First" claims about the Dubins, WSN, obstacle and spectral combinations would not survive. Two narrower claims could: game-based deconfliction layered on Dubins routing, and budget-triggered recursive spectral bisection with radius-tied affinity as the way of choosing fleet size. Both need ablation evidence, and two modelling problems in the draft should be fixed first.

### Cited Findings

| Claim | Verdict | Why |
|---|---|---|
| First planner for Dubins mobile sinks among obstacles | **Fails** | [Huang & Savkin 2017](https://doi.org/10.1016/j.comcom.2017.07.010) |
| First obstacle-aware spectral partition for Dubins vehicles | **Fails at abstract level** | [Wang et al. 2026](https://doi.org/10.1109/irce70780.2026.11739077) |
| First spectral clustering for obstacle-aware mobile sinks | **Fails** | [Karunakar 2026](https://doi.org/10.1109/iccsc67078.2026.11468635), [Niranjan 2026](https://doi.org/10.1109/iceccc70334.2026.11633036) |
| GA plus discrete-heading optimisation as a contribution | **Fails** | [Edison & Shima 2011](https://cris.iucc.ac.il/en/publications/integrated-task-assignment-and-path-optimization-for-cooperating-/), [Nayak 2023](https://doi.org/10.3390/s23146432) |
| Exact potential game, best response reaches a pure Nash equilibrium, as a result | **Fails as a result** | Shown for vehicles in [Liu 2022](https://arxiv.org/abs/2201.06157). From memory, the general theorem is Monderer & Shapley 1996 **[UNVERIFIED]** |
| Lexicographic (K, T_max, L_total) as a contribution | **Weak** | No exact match found, but it is a modelling choice, not a method |
| Repair by re-optimising the sinks involved | **Weak** | Simple forms in [Cai 2017](https://doi.org/10.3390/s17071607), [Li 2024](https://doi.org/10.3390/s24196345). (2026-11-09) The general form is MAPF-LNS2, confirmed in [full text](https://ojs.aaai.org/index.php/AAAI/article/view/21266); cite it |
| Radially unimodal seed plus pi * R per-peak surrogate | **Possible, narrow** | No hit for the phrase, but likely neighbours exist (pyramidal tours **[UNVERIFIED]**). Needs ablation |
| Budget-triggered recursive spectral bisection with radius-tied affinity, as fleet sizing | **Possible, as a method only** | Not found. At risk from [Wang 2026](https://doi.org/10.1109/irce70780.2026.11739077) full text. Travel-time costs may already capture turning. (2026-11-09) Minimising the number of sinks under a per-sink length limit with obstacles is itself published in [Kumar & Mukherjee 2026](https://doi.org/10.1109/comsnets67989.2026.11418194) [ABS], with an approximation guarantee, so the bisection needs a comparison with it |
| Discrete GO/WAIT/DETOUR potential game on timed Dubins tours, with route repair | **Possible, best distinct claim, but narrower (2026-11-09)** | Not found in any index tried. But [Picard 2022](https://ifaamas.csc.liv.ac.uk/Proceedings/aamas2022/pdfs/p1065.pdf) [FT] already has discrete per-conflict actions (postpone, elevate, skip) on pre-planned UAV trajectories, with conflicts weighted first. What remains is the Dubins setting, exposure time as an exact potential, and the coupling to routing. See the objections below |
| The whole integrated pipeline, "to the best of our knowledge" | **Likely survives as worded** | Not found, but integration alone is weak novelty. Support it with a feature table against ranks 1-5 |

**Objections a reviewer can raise, with evidence:**

1. **"This is not really a game."** All trajectories are held by one central planner, with no self-interested agents. An exact potential game has a shared objective, and [Lin et al. 2026](https://arxiv.org/abs/2610.05832) says such interactions "are routinely replaced by a potential game, i.e. by a common objective". [Bhatt et al. 2022](https://arxiv.org/abs/2206.08963) solve the equilibrium as a single optimal control problem. So best response here can be described as coordinate descent on a shared cost.
2. **"A Dubins vehicle cannot wait."** Section 0.1 of the draft says each sink is "limited to forward motion at a constant velocity", yet Section 0.10 defines WAIT as "stop at frame s", and the code does the same ([conflict_game.py](../../Dubins_TSP_New_Implementation_claude/conflict_game.py)). `trajectory.py` also adds a dwell at sensors. This rules out fixed-wing UAVs, one of the suggested equivalent settings. [Wu et al. 2025](https://doi.org/10.1038/s41598-025-09753-x) models waiting as a holding loop flown at minimum turning radius, and a reviewer may ask why the draft does not.
3. **"Collision-free is not achieved."** The draft's problem statement says "ensuring collision-free motion". The [README](../../Dubins_TSP_New_Implementation_claude/README.md) reports 22 of 30 maps with no conflict, and a worst closest approach of 0.28 m against d_safe = 3 m. Reachability and CBS-style work, such as [Mai 2025](https://arxiv.org/abs/2511.12160) and [db-CBS](https://arxiv.org/pdf/2309.16445), aims at safety explicitly.
4. **"The game barely beats a simple rule."** The same [README](../../Dubins_TSP_New_Implementation_claude/README.md) shows game against simple rule on seeds 1-30 with the repair step: 8 versus 12 pairs left, and on risk left the game wins 8 maps to 2 with 20 ties (p = 0.06). On the 18 maps where both end clear, the simple rule has lower T_max on 10 and the game on 1. [Li & Vorobeychik](https://www2.isye.gatech.edu/~fferdinando3/cfp/OPTMAS18/papers/paper_17.pdf) also report that safety is "often significantly compromised compared to a cooperative solution". Whether that transfers to the draft's centrally designed costs has not been tested.
5. **"Why visit exact sensor positions?"** Comparable work uses neighbourhoods or visiting circles ([Huang](https://arxiv.org/abs/1712.02436), [Macharet 2012](https://doi.org/10.1109/cec.2012.6256477)), and Huang calls exact targets a collision risk with the sensors.
6. **"The Fiedler split is not needed."** Spectral partitioning has to beat simple alternatives such as tour splitting ([Huang](https://arxiv.org/abs/1712.02436)) and K-means ([Liu 2025](https://arxiv.org/abs/2506.02365)) on the draft's own metrics.

### Inferences
- A safe framing: "We integrate X, Y and Z; to our knowledge no prior work combines Dubins multi-vehicle routing with game-based deconfliction", backed by a feature table. Avoid "first" for anything involving spectral partitioning or Dubins mobile sinks.
- Objection 2 is a modelling error that a reviewer can find without any literature search, so it is the most urgent to fix. The options are holding loops, or restating the vehicle as a stop-capable ground robot. That is a decision for the author, and I changed nothing.
- The game can only be defended on the separation constraint, which the README itself says. The honest framing is a measured trade-off, not a dominant method.

### Gaps
- All verdicts depend on the unread full texts and the unfinished citation trails, listed in the last section.

## Are there prior publications by Soumili Dey?

### Takeaway
I found no publications I could confidently attribute to this author, so that part is dropped. One clearly related public listing does exist: the project's own public GitHub repository, which matters for anonymity at double-blind venues.

### Cited Findings
- One web search for the name with WSN and mobile-sink terms returned no result naming this author. This is a negative result, so it has no source to link. The Semantic Scholar author lookup never ran, because I stopped that job.
- The repository [SoumiliDey98/Dubins_TSP](https://github.com/SoumiliDey98/Dubins_TSP) is labelled public, with 35 commits, and its file list includes `Dubins_TSP.pdf` and `theory_claude/`. It surfaced in an ordinary web search for Dubins multi-agent planning. Local `git log` shows the first commit on 2026-10-07, titled "Dubins MTSP with collision avoidance in a WSN", and `origin` points to this repository, so it is the same project.

### Inferences
- This is not a publication, but it is a dated public disclosure of the method and results under the author's name. Check double-blind rules at the target venue, and take advice before any patent filing.

### Gaps
- One name search is weak evidence, and Google Scholar, ORCID and IEEE author pages were not checked.

## What are the limits of this search, and what was queried?

### Takeaway
The search was cut short, and its "not found" results are moderate evidence at best. Citation trails were **not completed**, and neither Google Scholar nor IEEE Xplore was searched directly.

### Cited Findings

**What stopped the search:**

- **Web search budget:** the turn's shared limit of 200 searches ran out. 22 of my queries ran and 5 were refused. I then stopped searching by every route, including scholarly APIs from the shell, as the limit notice required.
- **Stopped job:** I stopped a background Semantic Scholar job that was running 17 keyword queries, 7 citation trails and the author lookup. It had written nothing, so **none of those results exist**.
- **OpenAlex:** my bulk download of Dubins records used up the free daily allowance shared by this network address, with a retry-after of 44439 s (about 12 hours). Any other agent on this address will get the same refusal until then. The download produced nothing usable.
- **Citation trails:** the listings for Gottlieb 2018 and Kumar & Mukherjee 2023 were **not obtained**. Only counts are known: Kumar & Mukherjee 4 (OpenAlex) or 5 (Semantic Scholar). Other counts from Semantic Scholar: Huang & Savkin 57, Macharet 2012 26, Fu 2026 7.
- **Refused queries, which could be run in a follow-up:**
  - A survey of UAV-based data gathering.
  - Macharet & Campos and other Dubins routing surveys.
  - Collision-constrained Dubins mTSP with delays.
  - Spectral clustering with geodesic or visibility-graph affinity.
  - Inter-sink collision avoidance by game.

**Weaknesses in the sources:**

- **Blocked sites:** direct fetches were refused (403 or a login redirect) for ScienceDirect, ResearchGate, ACM, MDPI, Nature and Springer. IEEE Xplore returned an empty page, and dblp blocked access. Much of the paywalled evidence is therefore abstract-level.
- **The arXiv API is demonstrably incomplete here.** `all:"mobile sink"` returned 25 records in total, and my local filter did not surface [Huang's report](https://arxiv.org/abs/1712.02436), although its abstract mentions mobile sinks, a non-holonomic model and collision-free paths. Many multi-term queries returned 0 where related papers exist. **Treat arXiv zero counts as weak evidence.**
- **Narrow matching:** OpenAlex title-and-abstract matching with quoted phrases is narrow, so its zero results are also weak.

**Queries that ran:**

- **arXiv API (33 queries)**, Dubins crossed with: mobile sink (0), sensor network (0), data collection (1), potential game (0), game + multi-vehicle + collision (0), spectral clustering (0), geodesic + partition (0), multiple TS (1), multi-agent TS (0), multiple vehicles + obstacles + routing (0), lexicographic + vehicles + makespan (0), conflict-based search (0), MAPF + task assignment (0), task assignment + collision-free (0), AGV (0), Nash + TS (0). Other queries, all 0 except where noted: "radially unimodal"; "radially monotone" tour; "curvature-aware" partitioning (3, unrelated); "pyramidal tour" vehicle; Fiedler multi-robot; spectral clustering vehicle routing; spectral + geodesic + multi-robot; geodesic + clustering + TSP; potential game with multi-robot, UAV, mobile sink or warehouse; game + mobile sink; multiple mobile sinks + obstacles; multi-UAV + fixed-wing + data collection; kinematically constrained + TS + conflict; wait + detour + best response.
- **arXiv bulk downloads, filtered locally:** all:dubins (245 records), "mobile sink(s)" (25), "potential game(s)" (467), turning radius or fixed-wing (412), spectral clustering or Fiedler (newest 300 of 3027), geodesic + multi (11), TS + conflict + multi (13).
- **OpenAlex (44 queries, title and abstract):**
  - Dubins with "mobile sink" (1), "mobile sinks" (1), "wireless sensor network" (3), "data collection" multiple (9), "data gathering" (2), "potential game" (1), "spectral clustering" (1), geodesic partition (0), "multiple traveling salesman" (15), same + obstacles (8), mTSP (8).
  - Multi-vehicle routing + collision (0), vehicle routing + collision avoidance (0), "conflict resolution" + TS (0), deconfliction + routing (0), task assignment + collision-free + GA + obstacles (0), lexicographic (0), fixed-wing data collection (0 and 0), visibility graph + TS (0).
  - Non-holonomic mobile sink (0 and 1), curvature constraint (0), multiple mobile sinks + obstacles (1), + collision (7), mobile sinks + game theory + trajectory (1), mobile sink + potential game (0).
  - Potential game + multi-UAV (0), + multi-robot (0), exact potential game + collision + vehicles (1).
  - Spectral clustering + routing + obstacles (0), + multi-UAV task allocation (1), Fiedler + multi-robot (1), Fiedler + TS (1), spectral bisection + routing (0).
  - "radially unimodal" (1), "curvature-aware" partitioning (13), clustering (0), geodesic + multi-robot (0), geodesic + k-means + UAV (0), obstacle-aware + mobile sink (6 and 2), visibility graph + mobile sink (0), "corridor game" (9).
- **Europe PMC (25 full-text queries):** Dubins with mobile sink (0 and 1), WSN + collision (2), potential game (0), game theory + TS (0), game-theoretic + task allocation (0), spectral clustering (0), Fiedler (0), geodesic + clustering (0), visibility graph + clustering (1), data collection + GA + multi-UAV (6), data collection + number of UAVs (6), minimum number + budget (0), time window + conflict (3). Potential game with mobile sink (1), path planning + Nash (1), deconfliction (0), multi-robot trajectory (4). Spectral clustering + mobile sink (2), + obstacles (1), geodesic + spectral (0), geodesic + task allocation (0), "radially unimodal" (0), multiple mobile sinks + game (5), + collision (14).
- **Web search (22 queries):**
  - Dubins + mobile sink.
  - Multiple sinks + curvature.
  - Dubins mTSP + obstacles.
  - Potential game + Dubins.
  - Spectral + Dubins.
  - CorridorGame.
  - Fu 2026 by title (paper not returned).
  - Three survey queries.
  - Potential game + multi-UAV.
  - Game + multi-AGV.
  - Geodesic clustering.
  - The author's name.
  - Dubins + game + fixed-wing.
  - Fixed-wing data collection.
  - Edison & Shima.
  - Pyramidal tours (no useful return).
  - Gottlieb by title.
  - Kumar & Mukherjee by title (abstract not returned).
  - Dubins + kinodynamic MAPF.
  - Wichmann & Korkmaz by title.

### Inferences
- Absence from this search does not show that a paper is unpublished. The "not found" verdicts could be overturned by a Google Scholar pass, IEEE Xplore full-text search, or citation trails.
- Before relying on these verdicts, in priority order. Status on 2026-11-09 is in brackets; details are in [follow_up_2026-11-09.md](follow_up_2026-11-09.md):
  1. Read Wang et al. 2026 in full. (Not done. It is CC BY on IEEE Xplore, which refuses automated downloads.)
  2. Read Fu et al. 2026 in full, and list its 7 citing papers. (Full text closed. OpenAlex lists 11 citing works; none is close.)
  3. List the 57 papers citing Huang & Savkin 2017. (OpenAlex lists 62; all screened. Kumar & Mukherjee 2026 is the one close paper.)
  4. Check what the game does in Raj et al. 2020. (Done at [SR] level: it picks rendezvous points.)
  5. Run the refused queries. (Done. New close papers: Picard 2022 and Xu et al. 2023.)

### Gaps
- Not searched directly at all: Google Scholar, IEEE Xplore, ScienceDirect, SpringerLink, ACM Digital Library. Chinese-language literature, theses and patents were not covered, apart from one [US patent on joint multi-UAV assignment and path planning](https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/10140875) [SR] whose page returned 403.
