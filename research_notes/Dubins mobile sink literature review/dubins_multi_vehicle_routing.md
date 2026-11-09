# Dubins TSP and its multi-vehicle variants: what is published, and how close it is to the draft's formulation

Compiled 2026-11-06. The draft's formulation, for reference: several Dubins vehicles leave one base, every static sensor is served exactly once with a stationary dwell tau_i, static disjoint polygonal obstacles need clearance, T_k = L_k/v + sum(tau_i) <= B per vehicle, pairwise separation >= d_safe at all t, objective lexmin (K, T_max, L_total), headings chosen from a discrete set by dynamic programming.

**Follow-up on 2026-11-09:** see [follow_up_2026-11-09.md](follow_up_2026-11-09.md). It adds a 2026 Dubins survey and Xu et al. 2023, and lists the papers citing Nayak & Rathinam 2023 and Gottlieb 2018 (Q8). Statements it changed are marked "(2026-11-09)".

**Evidence tags used on every paper.** These say what was actually read, because publisher sites mostly refused access.

- **[T]** I read the relevant part of the paper's text (open PDF or HTML). This is usually sections plus keyword checks, not a cover-to-cover reading.
- **[T*]** Open text, but read through the fetch tool's summariser, so not line by line.
- **[A]** I read the abstract, from a bibliographic API or repository record. Some abstracts were cut off at about 1100 characters; those are marked "cut off".
- **[S]** Only a search-result snippet or search summary. Treat as a lead, not a finding.
- **[R]** Existence confirmed by a bibliographic record or another paper's reference list. Content not read; anything said about content is inferred from the title or from another paper's description.

## Q1. Single-vehicle Dubins TSP: foundations and state of the art

### Takeaway
The single-vehicle problem is mature. It is NP-hard, and no general exact solver exists. The decoupled pipeline (order first, then discrete headings solved by a layered shortest path or dynamic program) is standard and well studied, so the draft's heading DP is established practice rather than a contribution.

### Cited Findings

**Foundations**
- Dubins gave the shortest bounded-curvature curve between two poses. [R] — [Dubins 1957, Am. J. Math. 79(3)](https://doi.org/10.2307/2372560)
- Reeds and Shepp extended this to a car that can reverse; at most 68 candidate paths are sufficient, and at most 2 cusps. [A] — [Reeds & Shepp 1990, Pacific J. Math.](https://doi.org/10.2140/pjm.1990.145.367)
- Savla, Frazzoli and Bullo define the DTSP for a vehicle on "planar paths of bounded curvature, without reversing direction". Worst-case tour length grows linearly in n, with a constant-factor algorithm for worst-case point sets. For uniformly random targets the expected length is of order at least n^(2/3), and an algorithm achieves order n^(2/3) with probability one. [A] — [Savla et al. 2008, IEEE TAC 53(6), 1378-1391](https://doi.org/10.1109/TAC.2008.925814)
- The n^(2/3) stochastic constant-factor algorithm appeared earlier as a preprint that states the vehicle moves "at constant speed". [T] — [Savla et al. 2006, arXiv cs/0603010](https://arxiv.org/abs/cs/0603010)

**Alternating algorithm and bounds**
- The Alternating Algorithm fixes the Euclidean TSP order, sets odd-numbered headings to give a straight segment to the next even-numbered point, and joins the rest with optimal Dubins paths. Its cost is bounded in terms of L_TSP and kappa * ceil(n/2) * pi * rho with kappa < 2.658. The PDF text I extracted lost the operator between the two terms; the additive form L_TSP + kappa*ceil(n/2)*pi*rho is from my memory of Savla et al., so check it before quoting. [T, second-hand via Cho & Choi] — [Cho & Choi 2018, arXiv 1808.02720](https://arxiv.org/abs/1808.02720). The original is [Savla et al. 2005, ACC](https://www.ccdc.ucsb.edu/node/12240) [S].
- When all points are at least 4 rho apart, the optimal DTSP exceeds the optimal ETSP by no more than pi*n. The proof uses the alternating construction. [T] — [Kirszenblat, Ayala & Rubinstein 2021, arXiv 2104.05253](https://arxiv.org/abs/2104.05253)
- NP-hardness is proved, with lower bounds on the approximation ratios of earlier heuristics and new heading-discretisation algorithms. [A] — [Le Ny, Feron & Frazzoli 2012, IEEE TAC 57(1), 265-270](https://doi.org/10.1109/TAC.2011.2166311)
- Any algorithm that follows an ETSP-optimal order cannot beat an Omega(n) ratio when no minimum point spacing is assumed. Their non-ETSP algorithm has guarantee O(min{(1+rho/eps) log n, (1+rho/eps)^2}), where eps is the minimum spacing. [A] — [Le Ny, Frazzoli & Feron 2007, CDC](https://doi.org/10.1109/cdc.2007.4434503)
- A constant-factor approximation exists for the single and multiple vehicle cases, assuming targets are at least 2 rho apart. [A for the result; assumption from Manyam & Rathinam, T] — [Rathinam, Sengupta & Darbha 2007, IEEE T-ASE 4(1), 98-104](https://doi.org/10.1109/tase.2006.872110); [Manyam & Rathinam](https://arxiv.org/abs/1506.08752)
- The 2005 report behind it gives factors 4.64 (one vehicle) and 6.08 (many vehicles). [S: the PDF would not download, and the journal's final figures are unconfirmed] — [UC Berkeley report](https://escholarship.org/uc/item/2qr347j1)

**Heading discretisation and DP**
- Headings at k samples per location form a search graph, and the shortest path for a given sequence is found "in O(nk^3) by a forward search based on dynamic programming". The authors describe sampling-based solvers as state of the art for the fixed-sequence problem. [T] — [Drchal, Faigl & Váňa 2020, WiSM, arXiv 2002.00811](https://arxiv.org/abs/2002.00811)
- Headings are discretised, an auxiliary network is built, and a shortest path is computed through it; the same technique handles open or closed subtours. [A] — [Babel 2020, J. Heuristics](https://doi.org/10.1007/s10732-020-09440-2)
- A discretised formulation gets a performance bound as a function of discretisation level and number of targets; the numerical results suggest a low level is often enough. [A, cut off] — [Cohen, Epstein & Shima, IISE Transactions 49, 238-254 (online 2016)](https://doi.org/10.1080/0740817x.2016.1217101)
- A class of discretisation-based look-ahead algorithms is parameterised by discretisation level and horizon, with time linear in targets plus the cost of the initial order. [A] — [Cohen, Epstein, Isaiah, Kuzi & Shima 2017, IEEE T-ASE 14](https://doi.org/10.1109/tase.2016.2602385)
- A k-step look-ahead and a 2-opt adaptation treat the combinatorial and motion parts together. [S] — [Isaiah & Shima 2015, Automatica 53, 247-255](https://doi.org/10.1016/j.automatica.2014.12.041)
- The fixed-sequence problem is named the Dubins Touring Problem (DTP). Informed sampling reaches less than 0.1 % from the optimum. [A] — [Faigl, Váňa, Saska, Báča & Spurný 2017, ECMR](https://doi.org/10.1109/ecmr.2017.8098685)
- The Dubins Interval Problem gives lower bounds, and Max-DIP upper bounds drive a branch-and-bound that prunes heading intervals. It is valid under the D4 assumption (consecutive points at least 4 rho apart), so it is not yet general. Interval bounds use dynamic programming. [T] — [Váňa & Faigl 2022, ACM SAC, DOI 10.1145/3477314.3507350](https://comrob.fel.cvut.cz/papers/sac22maxdip.pdf)
- Under D4 the fixed-sequence problem reduces to a family of convex problems over polyhedra. [A] — [Goaoc, Kim & Lazard 2013, SIAM J. Comput.](https://doi.org/10.1137/100816079). Váňa & Faigl count 2^(2n-2) of them. [T]

**Heuristics and neighbourhood variants**
- Three-point receding horizon, with order from an approximate straight-line TSP. [A] — [Ma & Castañón 2006, CDC](https://doi.org/10.1109/cdc.2006.376928)
- Improved Alternating Algorithm plus C-GRASP, reporting 28 % average length reduction against the classic technique. [A] — [Macharet, Neto, da Camara Neto & Campos 2011, ICRA](https://doi.org/10.1109/icra.2011.5980239)
- Orientation assignment heuristic. [R] — [Macharet & Campos 2014, IBERAMIA](https://doi.org/10.1007/978-3-319-12027-0_37)
- Routing survey with Dubins keywords. [A] — [Macharet & Campos 2018, Robotica](https://doi.org/10.1017/s0263574718000735)
- (2026-11-09) A dedicated Dubins survey: "Dubins path planning has evolved from simple analytical models to complex multirobot and dynamic scenarios. While exact solutions exist for basic tasks, the scalability, obstacle constraints and real-time responsiveness remain active challenges." [A] — [Zhou et al. 2026, "A seventy-year review of Dubins robot path planning", Industrial Robot 53(5):897-919](https://doi.org/10.1108/ir-07-2025-0265)
- GA. [R] — [Yu & Hung 2012, ISIE](https://doi.org/10.1109/isie.2012.6237270)
- DTSP with neighbourhoods by simple local iteration. [A] — [Váňa & Faigl 2015, IROS](https://doi.org/10.1109/iros.2015.7353945)
- Learning-based DTSPN. [A] — [Shin et al. 2024, arXiv 2404.16721](https://arxiv.org/abs/2404.16721)

**Optimality status**
- "Currently, there is no algorithm that can find an optimal solution to the DTSP", and the ETSP bound is often far below feasible costs; the paper gives the first systematic tight lower bounds. [A] — [Manyam & Rathinam 2018, JDSMC](https://doi.org/10.1115/1.4039099). Related: [Manyam et al. 2017, JIRS](https://doi.org/10.1007/s10846-016-0459-4) [R]; [Váňa & Faigl 2018, RSS](https://doi.org/10.15607/rss.2018.xiv.035) [A]; [Váňa & Faigl 2020, Auton. Robots 44(7)](https://doi.org/10.1007/s10514-020-09932-x) [R].

### Inferences
- A paper cannot claim novelty for choosing headings from a discrete set by DP. It should cite Le Ny 2012, Cohen 2016/2017, Faigl 2017 and Babel 2020 as prior art.
- Le Ny's Omega(n) result is a worst-case warning for any decoupled pipeline that orders points first without Dubins costs. It does not say such pipelines work badly at the sensor spacings used in practice, and it needs no minimum-spacing assumption to hold. A reviewer may still raise it.
- The task named "Cohen/Epstein/Shima": the three-author paper is the IISE one, and the T-ASE paper has five authors. Cite each correctly.

### Gaps
- I did not read the full text of Le Ny 2012, Savla 2008, Isaiah & Shima, or either Cohen paper. Publisher pages refused access, so the results above rest on abstracts and on other papers' descriptions.
- I found no published exact algorithm for the general DTSP (free order, no spacing assumption), but the searches were not exhaustive. The most recent explicit statement I read is from 2018, and WiSM (2020) says optimal solutions to its instances are unknown; Váňa & Faigl 2022 still restrict to D4 for the fixed-sequence problem.

## Q2. Multi-vehicle Dubins TSP and Dubins mTSP

### Takeaway
A small literature exists, with fixed fleets and min-sum or min-max objectives, and without obstacles or separation in the papers I could check. **Gottlieb, Manathara & Shima 2018, which the draft cites, is single-vehicle with an ordered target sequence, so it must not be described as multi-vehicle or as a routing paper.** Papers combining obstacles and inter-vehicle separation do seem to exist in applied UAV-swarm venues, but I could not read them; they are the main threat to a novelty claim.

### Cited Findings

**Theory and exact methods**
- n vehicles, each target visited once, tours return to start, heading free, objective is the sum of distances; a constant-factor approximation. Differs from the draft: fleet fixed, min-sum, 2 rho spacing, and no budget, dwell, obstacles or separation in the abstract. [A] — [Rathinam, Sengupta & Darbha 2007](https://doi.org/10.1109/tase.2006.872110)
- Multi-depot, functionally heterogeneous Dubins (aerial) and Reeds-Shepp (ground) vehicles with minimum total length, solved by branch-and-cut: 100 targets and 5 vehicles in 300 s on average. Heading angles at vertices are prescribed, which changes the problem, and the manuscript has no occurrence of "obstacle" or "collision". Its Reeds-Shepp vehicle also moves at "constant velocity". [T] — [Sundar & Rathinam 2017, JIRS 88, 513-526](https://doi.org/10.1007/s10846-016-0458-5); [accepted manuscript at OSTI](https://www.osti.gov/servlets/purl/1345942)
- Lower bounds for the multi-depot min-sum version, with about 39 % improvement on 4 vehicles and 40 targets. [A] — [Manyam, Rathinam & Darbha 2015, JDSMC](https://doi.org/10.1115/1.4030354)

**Min-max and mixed objectives**
- Dubins MinMax TSP: a single depot, fixed m (m = 3 in experiments), h = 8 headings, MILP, VNS heuristics and reinforcement learning. Obstacles are deferred ("Future work can address factors related to planning paths in the presence of obstacles"); separation and service time are not mentioned. The paper says no prior Dubins MTSP baselines exist. [T*] — [Nayak & Rathinam 2023, Sensors 23(14), 6432](https://doi.org/10.3390/s23146432); [PMC](https://pmc.ncbi.nlm.nih.gov/articles/PMC10383109/)
- Dubins mTSP with min-max objective via two discretisation-based MILPs. The authors state that no mathematical programming work for this problem had been found. [A] — [Zhu, Dai, Gao & Zhou 2022, RCAE, 242-248](https://doi.org/10.1109/rcae56054.2022.9995954). This conflicts with [Cho et al.](https://arxiv.org/abs/1808.02720), who gave a MILP earlier, and Nayak's "no baselines" claim (as the summariser reports it) conflicts with both. "First" claims in this subfield have been unreliable.
- Heterogeneous Dubins vehicles at multiple depots, minimising a weighted sum of total tour cost and largest tour cost, by MILP and a memetic algorithm. No occurrence of "obstacle" or "collision" in the arXiv text; "service times" appears only in a reference title. [T keyword check] — [Cho, Jang & Choi 2020, Int. J. Systems Science 51(4), 608-630](https://doi.org/10.1080/00207721.2020.1737263); [arXiv 1808.02720](https://arxiv.org/abs/1808.02720) (the arXiv version lists Cho & Choi only)
- Multi-UAV Dubins routing under steady wind, called a "variable-speed Dubins path vehicle routing problem"; objective is the maximum completion time, solved by GA, with experiments on two UAVs. No obstacles. Separation is assumed away: UAVs are assumed to have a collision-avoidance function. [T*] — [Luo, Liang, Zhu, Hu & Wang 2018, PLoS ONE](https://doi.org/10.1371/journal.pone.0194690)
- Other: multi-depot polygon-visiting Dubins mTSP [R] — [Ernest & Cohen 2012](https://doi.org/10.2514/6.2012-2562); GA for Dubins mTSP with neighbourhoods [A] — [Gao et al. 2024, ICCEIC](https://doi.org/10.1109/icceic64099.2024.10775515); heterogeneous min-max multi-depot TSP, kinematics not in the abstract [A] — [Kumar, Rathinam, Darbha & Bihl 2024, arXiv 2410.23449](https://arxiv.org/abs/2410.23449).
- Team version with budgets: fixed m Dubins vehicles maximise collected score, path length below L_max, discretised headings (2, 4, 6), branch-and-price. Differs from the draft: selection rather than cover, no obstacles or collisions in the text. [T] — [Sundar, Sanjeevi & Montez, arXiv 1912.04353](https://arxiv.org/abs/1912.04353). Also [Zhu et al. 2020, Applied Sciences](https://doi.org/10.3390/app10041388) [A].

**Obstacles together with several vehicles (the nearest neighbours)**
- Hierarchical multi-UAV DTSP in a 3-D complex obstacle environment: multilayer projection clustering, an adaptive window PRM for avoidance paths, an ant colony system with homotopy for sequencing, and Dubins paths for the turning radius. The part of the abstract I read does not state pairwise separation, a budget, or a fleet-size objective. [A, cut off] — [Fu, Sun, Liu, Yao & Wu 2023, IEEE T-Cybernetics](https://doi.org/10.1109/tcyb.2023.3265926)
- Distributed heterogeneous swarm allocation (SECBBA) with threat zones and Dubins curves tied into allocation. A search snippet reports collision handling by "altitude adjustment priority, supplemented by loiter-and-wait". The conflict-free phrase in the abstract refers to task allocation deadlocks, so separation needs checking. [A cut off, plus S] — [Yu et al. 2026, Aerospace 13, 601](https://doi.org/10.3390/aerospace13070601)
- A GA with Dubins curves is reported to avoid zones of electromagnetic interference and "guarantee inter-UAV collisions avoidance". [S only; bibliographic details not verified] — [ResearchGate 326627819](https://www.researchgate.net/publication/326627819_A_Fixed_Wing_UAV_Path_Planning_Algorithm_Based_On_Genetic_Algorithm_and_Dubins_Curve_Theory)
- Gale-Shapley plus GA for swarm allocation, with DP path refinement. Whether Dubins, obstacles or separation are involved is unconfirmed. [A cut off, S] — [Xu et al. 2026, Applied Sciences 16, 4428](https://doi.org/10.3390/app16094428)

**Separation without routing**
- Fleet transition by enumerating Dubins candidates at a given travel time, pairwise separation checks, and an ILP; about 95 % success with up to 20 aircraft. No target ordering in the abstract, and timing is handled by searching over travel times, ideally "maintaining cruise speed", rather than by waiting. [A] — [Feurgard, Hattenberger, Durand & Lacroix 2026, arXiv 2603.21789](https://arxiv.org/abs/2603.21789)
- Simultaneous-arrival Dubins paths with minimum separation conditions. [S] — [Shanmugavel et al. 2005, AIAA GNC](https://doi.org/10.2514/6.2005-5827)
- Dubins coverage by a robot team. [A] — [Karapetyan et al. 2018, arXiv 1808.02552](https://arxiv.org/abs/1808.02552)

**Routing with collisions, but not Dubins (MCPF, from multi-agent path finding)**
- Target assignment, ordering and collision-free paths solved together, with optimal or bounded-suboptimal guarantees. [A] — [Ren, Rathinam & Choset 2022, CBSS, RSS](https://doi.org/10.15607/rss.2022.xviii.058); [MS*, arXiv 2103.09979](https://arxiv.org/abs/2103.09979)
- Min-max version. [A] — [Ren, Nandy, Rathinam & Choset, DMS*, arXiv 2312.06314](https://arxiv.org/abs/2312.06314)
- With heterogeneous task durations at targets, either by post-processing (no optimality guarantee) or during planning (optimal), up to 20 agents and 50 targets. [A] — [Zhang, Wu, Wang & Ren 2023/2024, arXiv 2311.15330](https://arxiv.org/abs/2311.15330)
- A search for car-like or curvature-limited versions of MCPF returned none. [S]

**Car-like MAPF (kinematics and collisions, no routing)**
- Goals are assigned in advance; conflict search over a body conflict tree with spatiotemporal hybrid-state A*; a public 3000-instance benchmark. [A] — [Wen, Liu & Li 2022, Robotics & Autonomous Systems 150](https://doi.org/10.1016/j.robot.2021.103997)

### Inferences
- The closest published combination on routing plus collisions plus durations is CBSS with durations (Zhang et al.), but without curvature limits or budget, and with fixed agents. The closest on Dubins plus obstacles plus several vehicles is Fu et al. 2023, without a shown separation guarantee, budget or fleet-size objective.
- "First to combine obstacles and inter-vehicle separation for multiple Dubins vehicles" is **not a safe claim**: applied UAV papers (Yu 2026, the ResearchGate paper) appear to touch both. The defensible claim is about the full conjunction, stated in Q7.
- Zhu 2022 and Nayak 2023 show that "no prior work" sentences have been wrong in this area, so a novelty sentence should state the searched scope.
- (2026-11-09) A non-holonomic fleet that visits targets, with distributed task assignment and distributed collision avoidance that handles deadlocks, is published: [Xu, Kang, Yang, Wu, Liu, Cao, Liu 2023, IEEE T-ASE 21(4):7027-7039](https://doi.org/10.1109/tase.2023.3336076) [A], with [Python code](https://github.com/wuuya1/LRGO) naming ORCA among its planners. It avoids collisions reactively, not by resolving conflicts on planned timed tours, and has no budget or fleet minimisation in the abstract. It is a different paper from Xu et al. 2026 above.

### Gaps
- I could not read Fu 2023, Yu 2026, Xu 2026 or the ResearchGate paper in full. Someone with library access should check these four first, along with Ernest & Cohen 2012.
- I did not establish whether Rathinam 2007's multi-vehicle factor changed in the journal version.

## Q3. Dubins routing with obstacles: visibility graphs and geodesic costs

### Takeaway
Obstacle-aware Dubins routing is published, mostly for a single vehicle. Visibility graphs appear as a node set and a cost estimate, not as the routing metric of a multi-vehicle tour. Other papers use discrete routing models or roadmaps.

### Cited Findings
- **Gottlieb et al.** A single vehicle with a minimum turn radius visits an *ordered sequence* of targets among polygonal obstacles. The search tree has obstacle vertices, initial pose and targets as nodes; arcs are relaxed Dubins (aerial) or Reeds-Shepp (ground) paths with no terminal angle constraint. Methods: GA, a greedy heuristic that estimates remaining path by visibility-graph distance, and an exhaustive algorithm. Evaluated by sample runs and Monte Carlo. The heuristic does well for small turning radii; the GA balances load and quality. Differs from the draft: one vehicle, fixed order (no assignment or TSP), no dwell, no separation. [A, via the Technion record's summary] — [Gottlieb, Manathara & Shima, JIRS 90(3-4), 515-536, published 2018](https://doi.org/10.1007/s10846-017-0684-5); [Technion CRIS record](https://cris.iucc.ac.il/en/publications/multi-target-motion-planning-amidst-obstacles-for-autonomous-aeri/)
- **Babel 2017.** Minimum-length tour through landmarks avoiding obstacles for one kinematically constrained vehicle. Heuristics insert landmarks and optimise overflight directions for each order. "Path finding between landmarks is done by a discrete routing model", with arbitrary flight directions and turn angles. Tested on random scenarios with obstacles of varying size and number. This is the closest single-vehicle formulation, but not a visibility-graph or geodesic-distance method by its description. [A, via EconPapers] — [Babel 2017, EJOR 262(1), 335-346](https://doi.org/10.1016/j.ejor.2017.03.067); [EconPapers](https://econpapers.repec.org/RePEc:eee:ejores:v:262:y:2017:i:1:p:335-346)
- **Fu et al. 2023.** PRM-based roadmap (AWPRM) with a straight-line flight test, and Dubins-based curved paths after sequencing. [A, cut off] — [T-Cybernetics](https://doi.org/10.1109/tcyb.2023.3265926)
- **Hague et al.** 3D Dubins airplane, buildings as 2.5D extruded polygons, occlusion-free visibility volumes, 3D DTSPN with sampled configurations, and lower-bound edge costs to speed sequencing. One vehicle. [A] — [Hague, Willis, Maity & Wolek 2023, AIAA SciTech](https://doi.org/10.2514/6.2023-0108); [2026, J. Aerospace Inf. Systems](https://doi.org/10.2514/1.i011541)
- **Moving-target TSP with obstacles** defines a moving-target visibility graph on a standard visibility graph with a complete algorithm; the agent's kinematic model was not confirmed from what I read. A 2025 follow-up handles kinematic constraints by asymptotically optimal sampling plus a generalised TSP. [T partial; A] — [Bhat et al. 2024, arXiv 2409.09852](https://arxiv.org/abs/2409.09852); [Bhat et al. 2025, arXiv 2509.08743](https://arxiv.org/abs/2509.08743)
- Goaoc et al. note that the fixed-sequence problem arises in path planning for point car-like robots among polygonal obstacles. [A] — [SIAM J. Comput. 2013](https://doi.org/10.1137/100816079)
- Minimal Exposure Dubins OP considers a known sensor field as a threat, not as obstacles. [A] — [Macharet, Neto & Shishika 2020, arXiv 2010.11932](https://arxiv.org/abs/2010.11932)
- Obstacles often prevent a Dubins path between two landmarks, and the fix described is to replace Dubins paths with curvature-constrained obstacle-avoiding paths inside existing DTSP heuristics. [S, summary of Babel's snippet] — [Babel 2017 record](https://www.sciencedirect.com/science/article/abs/pii/S0377221717302989)

### Inferences
- Geodesic (visibility-graph) cost for clustering and ordering, followed by Dubins legs inflated around corners, looks less common than discrete-model or PRM approaches, but this is an impression from a small sample, not a finding. Gottlieb's use of visibility-graph distance as a cost-to-go is the nearest precedent and is a heuristic estimate.
- Accuracy warning for the draft itself: per the repository's notes, the GA fitness is straight-line length plus a turn term, with only the radial distance geodesic, while `partition.workload` uses geodesic legs. A sentence saying "routing on geodesic costs" would overstate the GA's cost function. Source: [CLAUDE.md, "Three length measures"](/workspaces/Dubins_TSP/CLAUDE.md).

### Gaps
- I did not read the full text of Gottlieb or Babel 2017. Exactly how Gottlieb handles a path that does not exist, and how Babel's routing model relates to visibility graphs, are unconfirmed.
- A curvature-constrained path among obstacles has its own literature, including Agarwal et al. seen in GMDM's reference list. I did not survey it.

## Q4. Lexicographic objectives and fleet minimisation under a per-vehicle budget

### Takeaway
No Dubins paper I found minimises fleet size first under a mission-time budget, or uses a lexicographic (K, makespan, total length) objective. The classical analogue is well established: distance-constrained VRP has minimum vehicles under a length bound, and fleet-first lexicographic objectives are standard in VRPTW. Each ingredient is known; the Dubins version appears unpublished.

### Cited Findings
- **Closest metric formulation.** DVRP: given a metric, depot and bound D, find a minimum-cardinality set of tours covering all vertices with each tour at most D. NP-complete even on a weighted star. 2-approximation on tree metrics, no factor better than 1.5 unless P = NP, and (O(log 1/eps), 1+eps) bicriteria on general metrics. Differs from the draft: Dubins costs are asymmetric and depend on headings, no obstacles, no dwell, no separation, no second or third objective. [A] — [Nagarajan & Ravi 2012, Networks](https://doi.org/10.1002/net.20435)
- Earlier exact methods for identical vehicles with a distance upper bound. [A] — [Laporte, Desrochers & Nobert 1984, Networks 14](https://doi.org/10.1002/net.3230140113)
- Approximation algorithms for minimum and min-max vehicle routing. Title only; the content is expected to cover minimum vehicle count under a route bound, which I did not confirm. [R] — [Arkin, Hassin & Levin 2006, J. Algorithms 59, 1-18](https://doi.org/10.1016/j.jalgor.2005.01.007)
- Tree-metric improvements: ratio about 1.691 + eps via bounded-space online bin packing. [S] — [Dufay, Mathieu & Zhou 2023, STACS](https://drops.dagstuhl.de/entities/document/10.4230/LIPIcs.STACS.2023.27)
- **Fleet first, then cost.** The VRPTW algorithm "first minimizes the number of vehicles" with simulated annealing, then minimises travel cost by large neighbourhood search. [A] — [Bent & Van Hentenryck 2004, Transportation Science](https://doi.org/10.1287/trsc.1030.0049)
- **Fleet size as an outcome in UAV routing.** A minimum-time coverage MILP where "the number of UAVs used to cover the area is automatically selected", influenced by maximum flight time and setup time. Differs from the draft: the objective is time, not fleet first; it covers areas rather than visiting sensors; curvature handling and separation not confirmed. [A, cut off] — [Avellar, Pereira, Pimenta & Iscold 2015, Sensors](https://doi.org/10.3390/s151127783)
- A CTU thesis titled "Motion planning for team of Unmanned Aerial Vehicles with flight time constraint and Dubins vehicle model" appeared in search; the page came back empty, so its contents, including whether team size is minimised, are unknown. [S] — [CTU DSpace](https://dspace.cvut.cz/handle/10467/76495)
- A fleet-size statement exists for dynamic routing: upper and lower bounds on the optimal number of vehicles for time-constrained problems. [S; PDF would not load] — [Bullo, Frazzoli et al., "Dynamic Vehicle Routing for Robotic Systems" (full author list not read)](https://viterbi-web.usc.edu/~ksavla/papers/Bullo.Frazzoli.ea.DVR.pdf)
- **Budgets with Dubins, fleet fixed.** Dubins OP, one vehicle, maximises reward within a travel budget. [A] — [Pěnička, Faigl, Váňa & Saska 2017, RA-L](https://doi.org/10.1109/lra.2017.2666261). Team version with fixed m and L_max. [T] — [Sundar et al.](https://arxiv.org/abs/1912.04353)
- **Lexicographic in motion planning, but not routing.** Hierarchical resource costs in graph search over PRMs with a single robot. A search summary says it uses Dubins vehicles and a budget layer; the part of the abstract I read does not mention Dubins. [A, S] — [Shan & Englot 2019, arXiv 1909.02184](https://arxiv.org/abs/1909.02184)
- **Nearest on objective shape among Dubins papers.** A weighted sum of total and largest tour cost, which is scalarised, not lexicographic, and has fixed fleet. [A] — [Cho et al. 2020](https://doi.org/10.1080/00207721.2020.1737263)
- Shriwastav & Song compare loiter circle packings partly on the number of UAVs needed, for coverage rather than routing. [A] — [arXiv 2005.08153](https://arxiv.org/abs/2005.08153)

### Inferences
- The draft's nearest named problem is the **distance-constrained VRP with a Dubins (asymmetric, heading-dependent) metric and service times**. Reviewers from operations research will reach for this, so the paper should say so and cite Nagarajan & Ravi and Laporte et al.
- Lexicographic optimisation needs a guarantee or at least a lower bound on K. Per the repository's notes, K in the implementation comes from recursively bisecting a sensor group while T > B ([CLAUDE.md, "plan_routes"](/workspaces/Dubins_TSP/CLAUDE.md)). That produces a heuristic upper bound on K, so a claim of solving lexmin needs wording like "heuristic for". A simple lower bound (total dwell + a lower bound on total travel time, divided by B, rounded up) would let the paper report a gap. This is my suggestion, not taken from a source.

### Gaps
- I found no source that explicitly proves or discusses hardness for the Dubins form of DVRP. It seems likely to follow from Dubins TSP hardness (B-feasibility with K = 1 is a decision form of it), but this is my reasoning and is not published as far as I found.
- Fully lexicographic triples (K, T_max, L_total) were searched only by keyword, and synonyms such as "hierarchical objective" or "fleet-first" were not exhausted.

## Q5. Dwell, service time and stopping: is "constant speed" violated?

### Takeaway
Yes, under the DTSP and control-theory definition, which is constant forward speed and no hovering; papers there avoid stopping by loitering or by serving on the move. However, LaValle's widely used definition of the Dubins car has forward speed in {0, 1}, including stopped, so the draft's model has a published name. The honest description is "a forward-only bounded-curvature car that may stop", using Dubins paths as its geometry.

### Cited Findings

**The strict definition: cannot stop**
- "Since the vehicles cannot hover, we assume that they fly along static closed curves called loitering curves." [A] — [Savla, Bullo & Frazzoli 2007, CDC](https://doi.org/10.1109/cdc.2007.4435017)
- Multi-UAV dynamic routing with "constant forward speed along paths of bounded curvature"; targets need only be visited. [A] — [Enright, Savla, Frazzoli & Bullo 2009, JGCD 32, 1152-1166](https://doi.org/10.2514/1.41616)
- The vehicle goes forward at constant speed v with |u| <= 1. [T] — [Váňa & Faigl 2022](https://comrob.fel.cvut.cz/papers/sac22maxdip.pdf)
- All vehicles travel at constant speed. [T] — [Sundar et al.](https://arxiv.org/abs/1912.04353). Also [Tang & Özgüner 2005, T-RO](https://doi.org/10.1109/tro.2005.847567) [A], "a point mass moving at a constant speed with a bounded turning radius".

**The permissive definition: can stop**
- "Reeds-Shepp Car: ... us in {-1, 0, 1}. This model intuitively makes us correspond to three discrete gears: reverse, park, or forward." "Dubins Car: Remove the reverse speed ... to obtain us in {0, 1}". [T, direct text] — [LaValle, Planning Algorithms, Ch. 13](http://lavalle.pl/planning/ch13.pdf)

**How dwell is modelled for curvature-limited vehicles**
- **Dwell in motion.** Inspection tours have segments observing each target continuously for "a predetermined dwell time", with "dwell times enforced along straight segments". This is the only paper I found with "dwell" in a Dubins tour. One vehicle, no stop. [A] — [Hague et al. 2026, JAIS](https://doi.org/10.2514/1.i011541)
- **Service as a constrained pass.** Each target needs a collecting manoeuvre, constrained to straight segments, motivated by dropping objects or heading-sensitive sensing. Whether the vehicle stops is not stated in what I read; the text implies a pass. [A] — [Váňa & Faigl 2016, Acta Polytechnica CTU Proc. 6](https://doi.org/10.14311/app.2016.6.0034)
- **Wait by loitering.** Fixed-wing swarm using altitude change first and loiter-and-wait as fallback. [S] — [Yu et al. 2026](https://doi.org/10.3390/aerospace13070601)
- **Wait by a longer path.** Choosing longer Dubins paths at a shared travel time, keeping cruise speed. [A] — [Feurgard et al. 2026](https://arxiv.org/abs/2603.21789)
- **Data collection, no dwell stated.** WSN mobile node as a Dubins vehicle that visits waypoints in overlapping RF footprints, without a dwell time in the abstract I read. [A, cut off] — [Macharet, Neto, da Camara Neto & Campos 2012, CEC](https://doi.org/10.1109/cec.2012.6256477)
- **Durations on graphs.** Task durations in collision-free routing, without curvature limits as far as the abstract shows. [A] — [Zhang et al. 2023/2024](https://arxiv.org/abs/2311.15330)

**Stop-capable or variable-speed vehicles**
- TSP for the Reeds-Shepp car and the differential-drive robot. [R] — [Enright & Frazzoli 2006, CDC, 3058-3064](https://doi.org/10.1109/cdc.2006.377339)
- TSP for a double integrator with bounded velocity and bounded control. [A] — [Savla, Bullo & Frazzoli 2009, IEEE TAC](https://doi.org/10.1109/tac.2008.2007856)
- Multirotor trajectory-based TSP with discretised velocity and direction at waypoints, computing time-optimal trajectories for edge costs; compared with a state-of-the-art Dubins-style approach, and the snippet says Dubins length divided by max speed gives travel time. Whether zero velocity is in the set is unconfirmed. [A, S] — [Meyer & Glock 2022, arXiv 2211.16820](https://arxiv.org/abs/2211.16820)
- GMDM permits speed changes along six Dubins types, with V = [vmin, vmax] and vmin, vmax positive, so it does not stop (experiments use vmin = 0.3 m/s); slower speed tightens turns and lowers obstacle risk. A short TSP study has 3 and 5 points. [T] — [Wilson, Gupta & Wettergren 2025, IEEE T-RO 41, 2861-2878](https://doi.org/10.1109/tro.2025.3554436); [arXiv 2402.01122](https://arxiv.org/abs/2402.01122)
- Earlier variable-speed work, seen in GMDM's reference list: Wolek, Cliff & Woolsey 2016, JGCD 39(10), 2374-2390; Kučerová, Váňa & Faigl 2020, ACM SAC; Kučerová, Váňa & Faigl 2021, IROS, 4714-4719. [R, via bibliography] — [GMDM references](https://arxiv.org/abs/2402.01122)
- A global planner on a DTSP plus NMPC for a curvature-limited agricultural ground robot, showing ground-vehicle DTSP use. Stopping is not mentioned in the abstract. [A] — [Ghorab & Lorenzen 2025, arXiv 2507.23350](https://arxiv.org/abs/2507.23350)

### Inferences
These are my reasoning from the sources, not statements in them.

- The tension is partly about terminology. Dubins' geometric result concerns curves, so a shortest leg stays shortest under any forward speed profile that keeps the curvature bound. Constant speed is what makes length proportional to time, and with speed in {0, v} the formula T_k = L_k/v + sum(tau_i) is still exact.
- Stopping does **not** remove the heading problem for a car-like vehicle, because the curvature bound still applies at rest and heading must be continuous across a stop. So the heading DP remains necessary. If the vehicle could turn in place (differential drive or multirotor), headings would decouple and the Dubins machinery would be unnecessary, which a reviewer may raise.
- The model excludes fixed-wing aircraft, the main motivation of most DTSP papers. The draft should name its platform as a car-like ground vehicle, or else replace stops with loiter or a straight-pass model and redo the budget.
- GMDM's motivation (slower means tighter turns) applies to speed-limited turn rates. For a steering-limited car R is geometric, so that benefit does not apply.
- Instantaneous stop and start ignores acceleration. Every stop has a time cost that the formula omits, and the double integrator and multirotor papers above show how others model this. Sensitivity could be stated in the paper.
- Waiting as the conflict resolver depends on stopping. It is the classical path-velocity decomposition ([Kant & Zucker 1986, IJRR](https://doi.org/10.1177/027836498600500304) [R]; [Peng & Akella 2005, IJRR](https://doi.org/10.1177/0278364905051974) [R]), and it is unavailable for a true constant-speed vehicle. So the stop assumption supports more than the dwell, and should be stated as a modelling choice with that consequence.
- A stopped vehicle is also a temporary obstacle for other vehicles. The combination of dwell and separation therefore has real effect on feasibility, and I found no Dubins paper treating this. It is a more defensible point of interest than the dwell term in the budget, which is simply additive.

### Gaps
- I could not confirm the content of Enright & Frazzoli 2006, or whether Meyer & Glock's velocity set includes zero.
- The word "dwell" returned only Hague 2026, but synonyms (on-site time, service, loiter, hold, sojourn) were checked only partly. The "mobile sink" literature may use "sojourn time"; it is outside this note's scope.

## Q6. Benchmarks, baselines and evaluation practice

### Takeaway
No standard benchmark exists even for the single-vehicle DTSP. Practice is random uniform instances over a density or radius grid, or TSPLIB coordinates with an assigned turning radius, with results compared against a small set of baselines and a lower bound where one is available. I found no public benchmark for multi-vehicle Dubins routing with obstacles and separation.

### Cited Findings

**Instance families**
- **Random uniform, density-based.** Side b = sqrt(n/d), 10 instances per pair, n in {25, 100, 500}, d in {0.1, 1, 10}. [T] — [WiSM](https://arxiv.org/abs/2002.00811). n = 10, d = 0.3, rho in {0.2, ..., 1.0}, 20 instances, median and 60 % non-parametric interval, order from LKH, angular resolution up to 256. [T] — [Váňa & Faigl 2022](https://comrob.fel.cvut.cz/papers/sac22maxdip.pdf)
- **Random uniform, fixed.** 25 instances of 20 targets in a 1000 x 1000 square with rho = 100, heading discretisations 4, 8, 16, 32, and Noon-Bean transformation to an ATSP. [T] — [Manyam & Rathinam](https://arxiv.org/abs/1506.08752). (My title search for the Noon-Bean paper returned a different paper, so its citation is not verified here.)
- **TSPLIB-derived.** bays29, eil51, eil76, eil101, with 3-5 vehicles and random headings and depots: 72 instances, radius 3*k*g/100. [T] — [Sundar & Rathinam 2017](https://www.osti.gov/servlets/purl/1345942). TSPLIB with h = 8 and rho = ceil(0.1 * max inter-city distance); the paper is inconsistent on size, 16-176 versus 16-137 cities. [T*] — [Nayak & Rathinam 2023](https://pmc.ncbi.nlm.nih.gov/articles/PMC10383109/). bays29 with circular neighbourhoods of 150 m. [T] — [Cho & Choi](https://arxiv.org/abs/1808.02720)
- **Orienteering libraries.** KU Leuven TOP library, up to 66 targets, 267 instances, 3 discretisation levels, 801 in total. [T] — [Sundar et al.](https://arxiv.org/abs/1912.04353). "Existing OP benchmarks". [A] — [Pěnička 2017](https://doi.org/10.1109/lra.2017.2666261)
- **Random obstacles.** Random scenarios varying obstacle size and number, for agile and less agile aircraft. [A] — [Babel 2017](https://econpapers.repec.org/RePEc:eee:ejores:v:262:y:2017:i:1:p:335-346). Sample runs and Monte Carlo. [A] — [Gottlieb](https://cris.iucc.ac.il/en/publications/multi-target-motion-planning-amidst-obstacles-for-autonomous-aeri/)
- **Learning sets.** Mendeley DTSP dataset with 10, 30, 50 targets, uniform and clustered, 16 headings, rho = 0.1. [S] — [Mendeley](https://data.mendeley.com/datasets/t3db2jz79z)
- **Car-like MAPF.** 3000 instances with released code. [A] — [Wen et al. 2022](https://doi.org/10.1016/j.robot.2021.103997)

**Baselines and metrics**
- Baselines: Alternating Algorithm, Local Iterative Optimization, Sampling-based Algorithm (GTSP to ATSP, solved by LKH), and a decoupled baseline of Concorde order plus IRIS headings. Optimal solutions are unknown, so results are relative. [T] — [WiSM](https://arxiv.org/abs/2002.00811)
- Gap against a lower bound from the interval problem. The gap is reported as 1.43 % in one displayed example. [T] — [Váňa & Faigl 2022](https://comrob.fel.cvut.cz/papers/sac22maxdip.pdf). A DIP-based lower bound is the source's key tool. [T] — [Manyam & Rathinam](https://arxiv.org/abs/1506.08752)
- MILP with time limit: CPLEX under 3 h gave feasible solutions only below 52 cities, none proven optimal, gaps 14.09 % to 46.46 %. [T*] — [Nayak & Rathinam 2023](https://pmc.ncbi.nlm.nih.gov/articles/PMC10383109/). By contrast, Sundar & Rathinam solve 100 targets and 5 vehicles to optimality, but with min-sum and prescribed headings. So exact baselines look out of reach for free-heading min-max problems of this size, which is my reading rather than a stated result.

### Inferences
My suggestions for the draft, based on the above:

- **Single-vehicle sanity check.** With K = 1 and no obstacles, compare path length against AA, a fine-resolution sampling baseline, and a DIP lower bound, on the WiSM density grid or on TSPLIB. This shows the draft's components match accepted results.
- **Fleet.** Report K against a lower bound on K, as no baseline exists for this. Also vary B, because K is a step function of it.
- **Separation.** Report the violation rate and closest approach distance over all seeds, not only feasible runs. The repository's own notes record conflicts left on some seeds ([CLAUDE.md, open item 2](/workspaces/Dubins_TSP/CLAUDE.md)). Literature on continuous collision checks for Dubins segments exists ([Schildbach 2023, IV](https://doi.org/10.1109/iv55152.2023.10186576) [R]); frame-based checks at a fixed dt should say what they miss.
- **Sample size.** 10-25 instances per configuration is the norm above, and medians with intervals are used, so ten seeds on one setting would look thin. Publishing instance files would fill a real gap.

### Gaps
- I did not extract the instance details for Babel 2017, Fu 2023 or Isaiah & Shima.
- Whether any shared obstacle-TSP instance set exists for curvature-limited vehicles remains unconfirmed; I found none.

## Q7. How close are the nearest formulations, and what remains unaddressed?

### Takeaway
No paper found has the draft's full conjunction. Each feature exists somewhere: the nearest papers match about 3 to 5 of the 7 table features (the one at 5, CBSS with durations, has no curvature limit), and the pair "K first" and "stationary dwell" is the rarest in the Dubins setting. Novelty lies in the combination and the model, not in any single component.

### Cited Findings

Y means the source says so. N means the source excludes it, or a text check found no mention. "?" means not stated in what I read, which is **not** evidence of absence. Tags are in the sections above.

| Work | Several vehicles | Free assign + order, cover all | Polygonal / static obstacles | Per-vehicle budget | Stationary dwell | Pairwise separation | K in objective |
|---|---|---|---|---|---|---|---|
| **Draft** | Y | Y | Y | Y (time, incl. dwell) | Y | Y | Y (lexmin first) |
| [Rathinam 2007](https://doi.org/10.1109/tase.2006.872110) | Y | Y | ? | ? | ? | ? | N (given) |
| [Sundar & Rathinam 2017](https://www.osti.gov/servlets/purl/1345942) | Y | Y (headings given) | N | ? | ? | N | N |
| [Cho et al. 2020](https://arxiv.org/abs/1808.02720) | Y | Y | N | ? | N | N | N (weighted total + max) |
| [Nayak & Rathinam 2023](https://pmc.ncbi.nlm.nih.gov/articles/PMC10383109/) | Y | Y | N | N | N | N | N (min-max, m fixed) |
| [Luo 2018](https://doi.org/10.1371/journal.pone.0194690) | Y | Y | N | ? | ? | N (assumed on board) | N (min-max) |
| [Fu 2023](https://doi.org/10.1109/tcyb.2023.3265926) | Y | Y | Y (3-D) | ? | ? | ? | ? |
| [Yu 2026](https://doi.org/10.3390/aerospace13070601) | Y | Y | threat zones | ? | ? | ? (altitude, loiter per snippet) | ? |
| [Sundar 2019 D-DTOP](https://arxiv.org/abs/1912.04353) | Y | N (selects a subset) | N | Y (length) | N | N | N |
| [Gottlieb 2018](https://cris.iucc.ac.il/en/publications/multi-target-motion-planning-amidst-obstacles-for-autonomous-aeri/) | N | N (ordered) | Y | ? | ? | N (one vehicle) | N |
| [Babel 2017](https://econpapers.repec.org/RePEc:eee:ejores:v:262:y:2017:i:1:p:335-346) | N | Y | Y | ? | ? | N (one vehicle) | N |
| [Hague 2026](https://doi.org/10.2514/1.i011541) | N | Y | Y (urban) | ? | N (dwell in motion) | N (one vehicle) | N |
| [Feurgard 2026](https://arxiv.org/abs/2603.21789) | Y | N (no routing stated) | ? | ? | N | Y | N |
| [CBSS with durations](https://arxiv.org/abs/2311.15330) (no curvature limit shown) | Y | Y | graph | ? | Y | Y | N |
| [CL-MAPF 2022](https://doi.org/10.1016/j.robot.2021.103997) (car-like, no routing) | Y | N | ? | ? | ? | Y | N |
| [Nagarajan & Ravi 2012](https://doi.org/10.1002/net.20435) (no kinematics) | Y | Y | N | Y (length) | N | N | Y (primary) |
| [Avellar 2015](https://doi.org/10.3390/s151127783) (coverage) | Y | coverage | ? | Y (flight time) | ? | ? | selected, not primary |

### Inferences

**Likely unaddressed, subject to the limits in Q8:**
1. The full conjunction above, for curvature-limited vehicles.
2. Lexicographic (K, T_max, L_total) with Dubins kinematics. The closest shapes are Cho's scalarisation, Nayak's fixed-m min-max and classical DVRP.
3. Stationary dwell inside a curvature-limited routing model, where a stopped vehicle also matters to other vehicles' separation.
4. A post-routing separation layer in continuous time for Dubins tours with dwell. The conflict-game component is outside this note's scope, but on the routing side I found no direct precedent.

**Not novel, and should be cited as prior art:**
- Heading discretisation with DP (Q1).
- Fleet-first lexicographic objectives (Bent & Van Hentenryck) and minimum-K under a route bound (Nagarajan & Ravi; Laporte et al.).
- Obstacle-aware Dubins tours (Babel; Gottlieb; Hague; Fu).
- Multi-vehicle Dubins routing (Rathinam; Cho; Nayak; Zhu).
- Collision-free multi-agent routing with durations on graphs (CBSS family).

**Suggested wording, once the unread papers are checked:** "To our knowledge, and within the searches described, we have not found a formulation that jointly treats fleet minimisation under a per-vehicle mission-time budget with stationary service, static polygonal obstacles and continuous inter-vehicle separation for forward-only curvature-limited vehicles." Avoid "first".

**Places where the method and the claim could diverge (from the repository notes, not the literature).** Conflicts are reported, not always removed, and Eq. 20 applies only while both sinks are on mission and not when both are inside the docking zone ([CLAUDE.md, Architecture and open item 2](/workspaces/Dubins_TSP/CLAUDE.md)). The formulation states separation at all t. A reader will expect the constraint to hold whenever a solution is claimed feasible. The exemption and the failure cases should be stated in the same place as the constraint.

### Gaps
- The matrix has many "?" entries because full texts were unavailable. These do not support any absence claim.
- I did not rate importance or journal quality systematically. Some nearest neighbours (Yu 2026, the ResearchGate paper, Gao 2024) are in lower-profile venues, but they count for a novelty claim all the same.

## Q8. What was searched for and not found, and the limits of that absence

### Takeaway
Nothing was found for the full conjunction or for the Dubins lexicographic form, but the evidence is modest. Most search channels were limited or blocked, and full texts were rarely available. Treat this as "not found in a limited search", not "does not exist".

### Cited Findings

| Looked for | How | Outcome |
|---|---|---|
| Dubins + fleet minimisation + time budget + lexicographic | Web search (extended), Crossref, arXiv `abs:Dubins AND abs:lexicographic` and `"minimum number"` | No match. The search summary said no paper combined them. Crossref returned unrelated Dubins papers. arXiv returned 0. |
| Multiple Dubins + obstacles + inter-vehicle separation in one framework | Web search (extended, twice), Crossref, arXiv `abs:Dubins AND abs:"inter-vehicle"` | Partial: Fu 2023, Yu 2026, an unverified ResearchGate paper. None confirmed to have all three. arXiv returned 0. |
| Dubins routing with service time or dwell | Web search, Crossref, arXiv `"service time"`, `"dwell"` | Only Hague 2026 (dwell in motion) and Váňa 2016 (collecting manoeuvre). arXiv returned 0. |
| Stationary stop in a Dubins tour, with constant-speed violation discussed | Web search (twice), loiter query | None discussing it. Loiter appears for fixed-wing (Savla 2007, Yu 2026). LaValle's {0, 1} was checked directly. |
| Reeds-Shepp TSP, multi-vehicle or with stops | Web search, Crossref, arXiv | Enright & Frazzoli 2006, Sundar & Rathinam 2017, Gottlieb 2018. No multi-vehicle R-S with obstacles and separation. arXiv returned 0. |
| Car-like or curvature-limited MCPF | Web search | None returned. CL-MAPF has goals assigned in advance. |
| Public benchmark for multi-vehicle Dubins with obstacles | Web search, reading instance sections | None found. |
| Exact algorithm for the general DTSP | OpenAlex and Semantic Scholar queries failed; Váňa 2022 and Manyam 2018 read | None confirmed; last explicit statement read is 2018. |
| Dubins + mobile sink | arXiv `abs:Dubins AND abs:"mobile sink"`; Crossref | arXiv returned 0. Macharet 2012 (mobile node for WSN) found through Crossref. The wider WSN literature is outside this note's scope. |

### Inferences
Limits on all of the above:

- **Search budget.** The session's web search limit was reached near the end, so a planned final query (the LaValle definition) was replaced by opening LaValle's chapter directly. No further web searches were possible. The web tool summarises results rather than listing them.
- **Discovery channels.** OpenAlex answered 39 title lookups (37 matched; GMDM and Noon-Bean did not), then returned HTTP 429 for all discovery queries. Semantic Scholar's search endpoint was too throttled to return results within the time limit, so its search contributed nothing. Crossref matches bibliographic text only, so abstracts are not searched. The arXiv API needs literal terms, so zero hits would miss "fixed-wing", "curvature-constrained", "nonholonomic" and similar wording.
- **Not queried directly:** Google Scholar, Scopus, Web of Science, IEEE Xplore search.
- **Access.** ScienceDirect, AIAA, MDPI (403), Springer (redirected to login) and PubMed (cookie wall) refused the fetch tool; IEEE and CTU DSpace pages came back empty. A library-proxy copy of a Springer PDF was blocked by a permission check; I did not retry it and used the authors' accepted manuscript at OSTI instead. Most full texts remain unread.
- **Language and venue.** Chinese-language and conference-only UAV work is likely under-sampled, and that is where a close match is most likely to be.
- **Terminology.** Synonyms (for example "mobile collector", "curvature-constrained VRP", "hierarchical objective", "sojourn") were not exhausted.

### Gaps
- Papers to obtain before any novelty statement is finalised, in order: Fu 2023; Yu 2026; the ResearchGate GA paper; the CTU thesis; Ernest & Cohen 2012; Avellar 2015; Gottlieb 2018; Babel 2017; Arkin et al. 2006; Enright & Frazzoli 2006.
- Run the Q8 queries on Google Scholar and Scopus, adding the synonyms above, and scan papers citing Nayak & Rathinam 2023, Fu 2023 and Gottlieb 2018 for work from 2024-2026. Citation chaining was not done here and is likely the quickest way to find a close match if one exists. (2026-11-09) Done for Nayak & Rathinam (12 citing works in OpenAlex) and Gottlieb (11): the new relevant ones are Xu et al. 2023, the 2026 Dubins survey, and two learned-routing papers. Fu 2023 was not chained.
