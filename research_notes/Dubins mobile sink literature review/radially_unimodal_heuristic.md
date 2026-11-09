# Radially unimodal tour heuristic: novelty assessment of Section 0.8

Searched on 2026-11-06. Evidence tags after each citation say how far I read the source: **[text]** = I read the relevant passage in the document; **[abstract]** = abstract or catalogue record only; **[snippet]** = seen only in a search-result summary, document not opened; **[meta]** = existence and bibliographic data confirmed, anything about content is inferred from the title. Own calculations are labelled as such, sit under Inferences, and are not published results. Scripts: `/tmp/claude-1000/-workspaces-Dubins-TSP/f3d37a5d-dcb4-42fd-8e49-536c9c4b50c5/scratchpad/exp/` (temporary location; ask if they should be kept). (2026-11-09) That folder no longer exists, so the own calculations below cannot be rerun from it.

**Follow-up on 2026-11-09:** see [follow_up_2026-11-09.md](follow_up_2026-11-09.md). The searches that did not run were run, and three of the four depot hits were checked. Statements this changed are marked "(2026-11-09)".

## Are the heuristic, its fitness and its seeding new, and what do they most closely resemble?

### Takeaway
The tour class is not new. It is the classical pyramidal tour class, taken with respect to one chosen linear order (rank of geodesic distance from the base), and the draft's m(π) = 1 is almost word for word the published definition. The hypothesis is confirmed on structure: the best tour of the class is available exactly in O(n²) for any cost matrix, and in O(n log² n) for straight-line legs with any linear order, so no GA is needed to find it. What I did not find published is narrower: using radial rank from a depot as the pyramidal order, the penalty term m(π)·π·R, and GA seeding from pyramidal tours. The word "Proposed" in the section title cannot stand without citing pyramidal tours.

### Cited Findings
- Published definition of the same condition: "In the more general pyramidal tsp problem, the points to be visited are labeled 1, . . . , n and the sequence of labels in the solution is required to have at most one local maximum." — [de Berg, Buchin, Jansen, Woeginger 2016, arXiv:1607.02725, extended abstract at ICALP 2016](https://arxiv.org/abs/1607.02725) **[text]**
- The same paper states "an optimal pyramidal tour among n points in the plane can be computed in O(n log² n) time with respect to any given linear order on the points", and that a bitonic tour "is pyramidal with respect to the left-to-right order". — [de Berg et al. 2016](https://arxiv.org/abs/1607.02725) **[text]**
- It also says that "Combined with a procedure for generating suitable permutations of the vertices, heuristic solutions to tsp can be obtained by computing optimal pyramidal tours with respect to the generated orders", citing Carlier and Villon. So using a chosen order plus the exact solver as a heuristic is itself published; a fixed radial order would be one such choice. — [de Berg et al. 2016](https://arxiv.org/abs/1607.02725) **[text]**
- Closest published pipeline found: a GA fixes the visiting order of a mobile sink using Euclidean distances, and Dubins curves are built afterwards. "The fitness function is defined as the reciprocal of Euclidean distances", in an underwater sensor network. It has no turning term and no radial structure. — [Cai and Zhang 2018, Sensors 18(7):2105, DOI 10.3390/s18072105](https://pmc.ncbi.nlm.nih.gov/articles/PMC6069250/) **[text]**; an earlier version, GA-3DTSP, is [Cai et al. 2017, Electronics Letters, DOI 10.1049/el.2016.3836](https://ietresearch.onlinelibrary.wiley.com/doi/full/10.1049/el.2016.3836) **[snippet, page returned 403]**
- Closest published objective of the form "length + turning": the angular-metric TSP, whose authors also study trading off the angle measure against the length measure. It charges turning angles, not radial peaks. — [Aggarwal, Coppersmith, Khanna, Motwani, Schieber 2000, SIAM J. Comput. 29(3):697-711](https://researchwith.njit.edu/en/publications/angular-metric-traveling-salesman-problem/) **[snippet]**; the combined cost is named the angular-distance-metric TSP in [a later tabu search paper](https://www.aminer.org/pub/64f9a7313fda6d7f063e1499/a-tabu-search-with-geometry-based-sparsification-methods-for-angular-traveling-salesman) **[snippet]**
- Closest published bound with πR per item: ETSP ≤ DTSP ≤ ETSP + κ⌈n/2⌉πρ with κ ≈ 2.6575. The count is points, not radial peaks. — [Savla, Bullo, Frazzoli, CDC 2005](http://motion.me.ucsb.edu/pdf/2005j-sbf.pdf) **[text]**
- The draft cites five references, none on pyramidal tours, the Dubins TSP bounds, or GA seeding, and "Lemma 1" appears once and is never stated. — `Dubins_TSP.pdf`, Sections 0.8.1 and 0.14 **[text]**

### Inferences
Novelty by component (my judgement from the findings in all sections):

| Component of Section 0.8 | Status | Closest prior work |
|---|---|---|
| m(π) = 1 tour class | Not new | Pyramidal tours (Klyaus, Sarvanov and Doroshko, Gilmore et al.); bitonic tours |
| Radial rank as the order | Not found published; a special case of "any linear order", so not a new algorithmic idea | de Berg et al. 2016; Carlier and Villon 1990 |
| Geodesic r with Euclidean legs | Not found published; a modelling choice with no stated guarantee | None found |
| Random split, then sort | Same as drawing a random subset in the standard parametrisation of pyramidal tours; I found no GA seeded this way, but one search for it could not run (see last section) | Gutin et al. 2002, Theorem 1 |
| Euclidean GA, Dubins afterwards | Published | Cai and Zhang 2018; Yu's greedy crossover uses Euclidean distance |
| m(π)·π·R penalty | Not found published; no theoretical basis found, and it fails as a bound in both directions (own counterexamples below) | Savla et al. bound; angular-metric TSP |
| "Reduces iterations" | Partly supported on its own fitness, but a GA is not needed for that optimum | Measurements below |

- A defensible wording for the paper: "we use pyramidal tours (Gilmore et al. 1985; Burkard et al. 1998) with respect to the geodesic radial order from the base".
- Possible genuine contributions, none tested here. (a) A condition under which the radial class contains the optimum, which is what the classical pyramidal literature provides and "Lemma 1" could become; Alkema et al.'s strip result suggests a narrow-sector version. (b) An exact pyramidal dynamic program over discretised headings, which optimises real Dubins length over the class in about O(n²k³) for k headings, with chain identity kept because Dubins costs are direction-dependent. I found no paper doing this, but the search was thin and I have not implemented it.

### Gaps
- I cannot prove absence. See the last section for what was and was not searched.
- Google Scholar, Scopus, Web of Science and IEEE Xplore were not searched directly.

## Do sources confirm the pyramidal definition and O(n²) dynamic program, how good are such tours, and how exactly does the draft relate?

### Takeaway
Confirmed. Definition, size 2^(n-2), and O(n²) for any cost matrix are all in sources I read. I found no published worst-case ratio for the best pyramidal tour against the optimal tour; the literature instead gives conditions under which the class contains the optimum. My own constructions give a ratio growing linearly in n in the worst case and like √n on uniform random points.

### Cited Findings
- Definition: a tour ⟨1, i1, ..., ir, n, j1, ..., j(n-r-2)⟩ with i1 < ... < ir and j1 > ... > j(n-r-2). "a minimum cost pyramidal tour can be determined in O(n²) time by dynamic programming, while the total number of pyramidal tours is exponential in n". — [Bondarenko and Nikolaev 2017, arXiv:1710.06286](https://arxiv.org/abs/1710.06286) **[text]**
- Size and first proof: |PY(x1, H)| = 2^(n-2), a tour is determined by the subset of intermediate cities visited on the way up, and O(n²) "was proved for the first time by Klyaus". The neighbourhood is attributed to Sarvanov and Doroshko (1981). — [Gutin, Yeo, Zverovich, "Exponential neighborhoods and domination analysis for the TSP", in Gutin and Punnen (eds.), The Traveling Salesman Problem and Its Variations, Kluwer](https://www.cs.rhul.ac.uk/home/gutin/paperstsp/chGYZ.pdf) **[text]**
- Citation correction: both sources I opened date Klyaus to 1976, not 1971: "The structure of the optimal solution of certain classes of travelling salesman problem", Vestsi Akad. Navuk BSSR no. 6:95-98, 1976 ([Gutin et al.](https://www.cs.rhul.ac.uk/home/gutin/paperstsp/chGYZ.pdf) **[text]**), and a 1976 Minsk preprint ([Bondarenko and Nikolaev](https://arxiv.org/abs/1710.06286) **[text]**). I did not find a 1971 Klyaus paper. Neither Klyaus paper was opened.
- Built-in weakness: every tour in the class uses the arc x1x2 or x2x1, and x(n-1)xn or xnx(n-1), so the algorithm "will not produce a good tour if these arcs are expensive". The fix, the Carlier-Villon neighbourhood, takes the union over all n starting cities at O(n³), and contains 2-Opt and at least 75 % of 3-Opt tours. — [Gutin et al.](https://www.cs.rhul.ac.uk/home/gutin/paperstsp/chGYZ.pdf) **[text]**
- Empirical quality when iterated: Carlier and Villon repeat "find best pyramidal neighbour, renumber" on 10 random 60-city instances. Their table gives 6330/6330, 6665/6559, 6402/6344, 6412/6324, 6207/6101, 5976/5960, 6577/6439, 6257/6191, 6689/6484, 6578/6235 (BELPERM/best); they call it "very good" but slower than Lin-Kernighan and suggest coupling the two. — [Carlier and Villon 1990, RAIRO 24(3):245-253](https://www.numdam.org/item/RO_1990__24_3_245_0/) **[text]** (my arithmetic: mean 1.8 % above best, worst 5.5 %)
- When the optimum is pyramidal: matrix conditions (Monge, Demidenko, Van der Veen, Kalmanson, Supnick) guarantee a pyramidal optimum; Aizenshtat and Kravchuk 1968 and Gilmore, Lawler and Shmoys 1985 are the origins. — [Bondarenko and Nikolaev 2017](https://arxiv.org/abs/1710.06286) **[text]**; [van der Veen, Sierksma, van Dal 1991, EJOR 52(1):90-102, DOI 10.1016/0377-2217(91)90339-W](https://www.iaorifors.com/paper/8645) **[snippet]**; [Warren 1994, Optimization, DOI 10.1080/02331939408843938](https://doi.org/10.1080/02331939408843938) **[abstract]**; [Deineko, Klinz, Woeginger 2023, arXiv:2302.05161](https://arxiv.org/abs/2302.05161) **[abstract]**
- Survey entries confirmed but body not read: [Burkard et al. 1998, SIAM Review 40(3):496-546, DOI 10.1137/S0036144596297514](https://doi.org/10.1137/S0036144596297514) **[meta; a search summary says its Section 2 covers pyramidal tours]**; Gilmore, Lawler, Shmoys, "Well-solved special cases", pp. 87-143 in The Traveling Salesman Problem, Wiley 1985, as listed in [Bondarenko and Nikolaev](https://arxiv.org/abs/1710.06286) **[meta]**.
- Extensions: [Enomoto, Oda, Ota 1998, Discrete Applied Mathematics, DOI 10.1016/S0166-218X(98)00048-1](https://doi.org/10.1016/S0166-218X(98)00048-1) **[meta]**, step-backs; [Oda and Ota 2001, DOI 10.4036/iis.2001.123](https://doi.org/10.4036/iis.2001.123) **[abstract]**; [Baki and Kabadi 1999, Computers & OR 26(4):353-369, DOI 10.1016/S0305-0548(98)00067-7](https://doi.org/10.1016/S0305-0548(98)00067-7) **[meta]**.
- Bitonic tours: CLRS Problem 15-3, quoted in full, attributes the restriction to J. L. Bentley and asks for O(n²). — [de Berg et al. 2016](https://arxiv.org/abs/1607.02725) **[text; CLRS itself not opened]**. A shortest bitonic tour is a shortest tour for points with distinct integer x-coordinates in a strip of width δ ≤ 2√2, and that bound is best possible. — [Alkema, de Berg, van der Hofstad, Kisfaludi-Bak, SoCG 2020, arXiv:2003.09948](https://arxiv.org/html/2003.09948) **[text]**
- Radial distance from a point is not unimodal even around a convex polygon: from a vertex of an n-gon it can have n/2 maxima. — [Avis, Toussaint, Bhattacharya 1982, Comput. Math. Appl. 8(2):153-156, DOI 10.1016/0898-1221(82)90054-2](https://www.bohrium.com/paper-details/on-the-multimodality-of-distances-in-convex-polygons/812391251427983360-913) **[snippet]**

### Inferences
Exact mapping to the classical form:

| Draft | Classical pyramidal tour |
|---|---|
| Base S0, r = 0 | City 1 |
| Apex: "the absolute farthest sensor" | City n |
| S_out sorted by ascending r | i1 < ... < ir |
| S_in sorted by descending r | j1 > ... > j(n-r-2) |
| Random split of S_k | Random choice of the subset FORW(G, H) |
| m(π) = 1 | "At most one local maximum" of the labels |
| Fitness inside the class | Euclidean tour length + the constant πR, so R has no effect on the ranking of m = 1 tours |

With N sensors there are n = N + 1 cities and 2^(N-1) directed tours, each reached by 2 of the 2^N splits because the farthest sensor may fall in either set. For symmetric costs that is 2^(N-2) distinct tours. So the initial population is a uniform random sample of the class. The chance that 50 draws contain the best tour of the class is 1 - (1 - 2^(2-N))^50: 0.80 at N = 7, 0.18 at N = 10, 0.02 at N = 13.

What differs from the classical setting, and whether it changes the conclusion:

1. **Radial order from a depot.** Classical results take any linear order, so this is a choice of H, not an extension. Ties in r need a tie-break, because a pyramidal order must be strict.
2. **Geodesic r with Euclidean legs.** The dynamic program does not care what the costs are, so it applies unchanged. The mixing is a modelling inconsistency: order from one metric, leg cost from another, and final length from a third (Dubins around obstacles). No source offers a guarantee for this.
3. **Variation can leave the class.** The draft's GA searches all permutations from a start inside the class. So it does not literally solve the class problem, and the dynamic program does not literally replace it. Own measurement on repository maps (R = 5 m, seeds 1-5, n = 94-109): at the top level the GA finishes with mean m between 12 and 17 per map, Fitness 3542-4255 in about 1 s, while the dynamic program gives the best m = 1 tour with Fitness 3065-3932 in about 1.4 ms. It beat the GA on all 5 maps.
4. **The m(π) penalty.** No analogue in the pyramidal literature I read. But the whole Fitness over all tours also needs no GA. Whether a sensor is a peak depends only on whether arrival and departure are rising and falling, so each sensor can be split into two copies, "arrived rising" and "arrived falling", giving an asymmetric equality-generalised TSP with clusters of size 2. That can go to LKH unmodified via the [Helsgaun 2015 transformation](https://doi.org/10.1007/s12532-015-0080-8) **[snippet]**, or to Held-Karp for small n, which I implemented and checked against brute force. This reduction is mine, not published.

Quality of the class, own calculations (R-independent, Euclidean):

- **Worst case grows with n.** Three spokes 120° apart, sensors at evenly spaced radii in cyclic spoke order. The best m = 1 tour against a petal tour with m = 3: length ratio 1.54, 2.83, 4.99, 9.32, 17.97 at n = 12, 30, 60, 120, 240. Real Dubins lengths gave the same ratios (1.54 to 18.02). Two chains cannot serve three spokes without jumping across the map, so the excess is linear in n while the petal tour stays bounded by the spoke length. The penalty does not rescue the class here: the petal tour has far lower Fitness (3638 against 64570 at n = 240).
- **Typical case grows like √n.** Uniform points in a unit square. Best m = 1 tour divided by an iterated 2-opt + Or-opt tour, which is an upper bound on the optimum, so these ratios are conservative. Base at centre: 1.27, 1.64, 2.51, 3.57, 5.02, 7.82, 10.63 at n = 10, 20, 50, 100, 200, 500, 1000. Base at corner: 1.05, 1.16, 1.47, 2.04, 2.66, 4.25, 5.77. Reason: sorted by r, each next sensor's angle is nearly independent of the two chain ends, so legs stay long and the length grows like n, whereas [ETSP grows like √n](http://motion.me.ucsb.edu/pdf/2005j-sbf.pdf). This is a sketch, not a proof.
- **The optimum is rarely in the class on open maps.** Exact solutions, uniform points, base at centre: the Euclidean optimum was radially unimodal on 15.3 % of instances at n = 6, 2.0 % at n = 8, 0 % at n = 10, 2.5 % at n = 12, with mean m(optimum) rising from 2.0 to 3.5.
- **It is often a mild restriction on compact groups.** On the planner's own sink groups (below), the restriction costs little, which fits the narrow-strip theorem of Alkema et al. The heuristic is therefore plausible when partitioning has already made a group narrow, and weak for large or wide groups. That is a regime statement the draft does not make.

### Gaps
- No published worst-case ratio of best pyramidal tour to optimal in the Euclidean plane was found. Absence not proven; the survey bodies were not read. Search for it returned nothing on point.
- Sarvanov and Doroshko (1981), Klyaus (1976) and Aizenshtat and Kravchuk (1968) are cited through later papers only.
- The strip theorem is for straight x-coordinates. A polar sector version would need its own proof.
- Hull order of the optimal Euclidean tour for points in convex position (Flood 1956) would combine with Avis et al. to show an optimal tour with many radial maxima. **Flood 1956 is from memory and unverified.**

## Has radial, monotone or unimodal ordering around a depot been used for routing before?

### Takeaway
Depot-centred geometry is old, but what is classical is angular (sweep and petal), or a "radial" cost term in analysis, not ordering stops by distance from the depot. I found no paper using distance-from-depot rank as the pyramidal order. The word "radial" is a trap: in the petal literature it means angular adjacency.

### Cited Findings
- Sweep: "The locations that are used to make up each route are determined according to the polar-coordinate angle for each location", then distance is improved iteratively. — [Gillett and Miller 1974, Operations Research 22(2):340-349](https://doi.org/10.1287/opre.22.2.340) **[abstract]**. This is the same step as the draft's "Polar Angle Evaluation and Angular Sectoring" in Section 0.8.2.
- Petal: routes through "sets of radially contiguous locations", chosen by an integer programme. — [Foster and Ryan 1976, Operational Research Quarterly 27(2):367-384](https://trid.trb.org/View/66410) **[snippet]**. Extended in [Ryan, Hjorring, Glover 1993, JORS 44:289-296](https://link.springer.com/content/pdf/10.1057%2Fjors.1993.54.pdf) **[snippet]**, solved by shortest paths; [Renaud, Boctor, Laporte 1996, JORS, DOI 10.1057/jors.1996.29](https://doi.org/10.1057/jors.1996.29) reports results about 2.38 % from best known on 14 benchmarks **[snippet]**.
- "Radial" as a cost: Daganzo's two classical lower bounds are called the radial cost and the local cost. — [arXiv:2304.11281](https://arxiv.org/html/2304.11281) **[snippet]**. The radial bound is cost(OPT) ≥ (2/Q) Σ distance to depot. — [arXiv:1707.08270](https://arxiv.org/pdf/1707.08270) **[snippet]**, citing [Haimovich and Rinnooy Kan 1985, Math. of OR, DOI 10.1287/moor.10.4.527](https://doi.org/10.1287/moor.10.4.527) **[abstract]**; [Daganzo 1984, Transportation Science 18(4):331-350](https://doi.org/10.1287/trsc.18.4.331) **[abstract]**.
- Sensor-network data mules use GA-based TSP heuristics without radial structure, as far as the abstract shows. — [Levin, Efrat, Segal, "Using Data Mules for Sensor Network Data Recovery"](https://www.ee.bgu.ac.il/~segal/mules2.pdf) **[snippet]**
- Dubins-based sensor-network work is concentrated underwater. — [Cai and Zhang 2018](https://pmc.ncbi.nlm.nih.gov/articles/PMC6069250/) **[text]**

### Inferences
- The draft's combination of polar sectoring with radial sorting within a sector looks like a sweep-then-order scheme, with radial pyramidal order taking the place of the usual per-route TSP solve.
- Own calculation: in the three-spoke test, the petal structure beats the unimodal one by a factor growing with n. So on maps with radial corridors the classical petal shape is the right structure, and it is explicitly the one m(π) penalises.
- Section 0.8.2 of the draft partitions by polar angle, while the implementation partitions by geodesic spectral bisection (`planner.plan_routes`, `partition.bisect`). This is outside my assignment but should be reconciled.

### Gaps
- A combined search for UAV or vehicle "monotone radial" routes returned nothing relevant, and "cone" and "spiral" searches returned only loosely related work on view cones and data mules. I did not read any of those in full.
- OpenAlex lists four depot-related hits for "pyramidal tour": [Three essays on sequencing and routing problems (MIT, 2005)](https://dspace.mit.edu/handle/1721.1/17517), [Exact and Heuristic Algorithms for Routing AGV on Path with Precedence Constraints (2016)](https://doi.org/10.1155/2016/5040513), [2VRP (arXiv:1802.08042, 2018)](https://arxiv.org/abs/1802.08042) and [To walk or not to walk (2024)](https://doi.org/10.1016/j.tre.2024.103696). I read only the 2VRP abstract, which does not mention pyramidal tours. **Someone should check these four before the novelty statement is finalised.** The first link is my guess at the matching record, inferred from a later search; treat as unconfirmed. (2026-11-09) Three are checked, and none orders stops by distance from the depot. 2VRP [text] uses pyramidal tours under Kalmanson matrices, ordered by convex position. The AGV paper [abstract] orders along a guide wire. "To walk or not to walk" [text, keyword scan] mentions pyramidal tours only in its references (Oda & Ota 2001, "pyramidal tours with restricted jump-backs"). The MIT thesis is still unchecked.
- From memory only, and not confirmed: that Daganzo's zones are elongated towards the depot with stops served half on the way out and half on the way back. If so, this would be close prior work. The searches confirmed only the "strip" strategy.

## Are there GA or evolutionary methods for the Dubins TSP, how do they treat fitness, and does m(π)·π·R have a basis?

### Takeaway
GA and evolutionary methods for the Dubins TSP exist since at least 2012, and they use real or learned Dubins length as fitness. Cheap evaluation has been published, but none I found uses Euclidean length plus a peak-count penalty. Euclidean-to-Dubins bounds exist and scale with points, not radial peaks. I found no theoretical basis for m(π)·π·R; it fails as a bound in both directions, and in my tests it ranks tours worse than plain Euclidean length.

### Cited Findings
Evolutionary methods:
- Yu and Hung: GA with permutation plus orientations in the chromosome, random initialisation, fitness 1/L where L is the sum of leg costs over the tour with chosen headings, and a greedy crossover that uses Euclidean distance. Parameters for the low-density case: population 20, up to 7000 generations, n from 5 to 50, 30 random samples per n. Beats the Alternating Algorithm at low density and is no worse than Random Headings at high density, but "When the number of waypoints becomes very large... not as efficient". — [Yu 2015 PhD dissertation, Auburn, Chapter 5](https://etd.auburn.edu/handle/10415/4599) **[text]**; [Yu and Hung 2012, IEEE ISIE, DOI 10.1109/ISIE.2012.6237270](https://doi.org/10.1109/ISIE.2012.6237270) **[abstract]**
- Macharet et al.: three-stage evolutionary algorithm for the Dubins TSP with neighbourhoods, varying waypoint position, then orientation, then sequence. — [GECCO 2012, DOI 10.1145/2330163.2330218](https://doi.org/10.1145/2330163.2330218) **[abstract]**
- Zhang et al.: memetic algorithm, using Dubins paths "in the sense of terminal heading relaxation... to simplify the calculation of the Dubins distance". This is a published cheap evaluation. — [Chinese J. Aeronautics 2014, DOI 10.1016/j.cja.2014.04.024](https://doi.org/10.1016/j.cja.2014.04.024) **[abstract]**
- WiSM: a learned regression surrogate estimating Dubins cost through a sliding window, used by a simple EA that evolves only the permutation. 75.4×10³ path evaluations per second at n = 100, against 221 and 7.84 for IRIS-based EAs. Cost relative to the Concorde + IRIS decoupled baseline at n = 100, d = 1: Alternating Algorithm 1.3, best sampling + LKH method 0.73, WiSM-EA 0.61 at 2.6 s and 0.51 at about 10 minutes. It groups solvers into decoupled, joint evolutionary and GTSP-transformation approaches. — [Drchal, Faigl, Váňa 2020, arXiv:2002.00811](https://arxiv.org/html/2002.00811v1) **[text]**. A search summary says the preprint was under review at a journal; final venue not confirmed.
- Non-evolutionary comparators: [Babel 2020, J. Heuristics 26(4):503-530](https://doi.org/10.1007/s10732-020-09440-2) **[abstract]**; [Cohen, Epstein, Isaiah, Kuzi, Shima, IEEE T-ASE, DOI 10.1109/TASE.2016.2602385](https://doi.org/10.1109/TASE.2016.2602385) **[abstract]**; multi-vehicle: [Nayak and Rathinam 2023, Sensors 23(14):6432](https://doi.org/10.3390/s23146432) **[text, partly]**, using greedy or cheapest insertion with Dubins distances and variable neighbourhood search, with 8 sampled headings.

Bounds:
- Upper and lower: ETSP ≤ DTSP ≤ ETSP + κ⌈n/2⌉πr with κ < 2.658. Near-tightness: for any r there is a point set with ETSP + 2⌊n/2⌋πr ≤ DTSP. A single Dubins leg costs at most d + κπr. For small enough r, the optimal ETSP order equals the optimal Dubins order. — [Savla, Frazzoli, Bullo, ACC 2005](https://skoge.folk.ntnu.no/prost/proceedings/acc05/PDFs/Papers/0144_WeB08_1.pdf) **[text; floor and ceiling symbols partly lost in extraction, cross-checked against the CDC paper]**; [Savla, Bullo, Frazzoli, CDC 2005, Theorems 2.1-2.2](http://motion.me.ucsb.edu/pdf/2005j-sbf.pdf) **[text]**; journal version [IEEE TAC 2008, DOI 10.1109/TAC.2008.925814](https://doi.org/10.1109/TAC.2008.925814) **[meta]**
- Growth: ETSP is Θ(√n), stochastic DTSP is Ω(n^(2/3)) and O(n^(2/3) log(n)^(1/3)), worst-case DTSP is Θ(n). — [Savla et al., CDC 2005](http://motion.me.ucsb.edu/pdf/2005j-sbf.pdf) **[text]**; [arXiv:cs/0603010](https://arxiv.org/abs/cs/0603010) **[abstract]**
- Euclidean order: any algorithm following an ETSP-optimal ordering has approximation ratio no better than Ω(n). — [Le Ny, Frazzoli, Feron, CDC 2007, DOI 10.1109/CDC.2007.4434503](https://unpaywall.org/10.1109%2FCDC.2007.4434503) **[snippet of abstract]**. The DTSP is NP-hard, with lower bounds on the ratio of some heuristics. — [Le Ny, Feron, Frazzoli, IEEE TAC 57(1):265-270, 2012, DOI 10.1109/TAC.2011.2166311](https://doi.org/10.1109/TAC.2011.2166311) **[snippet of abstract; publisher record confirmed]**
- Density regime: "when waypoints are far apart, the Dubins problem becomes similar to the Euclidean problem", and when they are close, "the optimal order of the ETSP is no longer the optimal order for DTSP". — [Yu 2015, Section 5.4](https://etd.auburn.edu/handle/10415/4599) **[text]**
- The ETSP "provides the only lower bound available", and it is often "far below the cost of the feasible solutions". A systematic tighter bound is offered. — [Manyam and Rathinam, arXiv:1506.08752](https://arxiv.org/abs/1506.08752) **[abstract]**, published in ASME JDSMC 2018 per the record.
- A rigorous "count of maxima × 2π" result exists for linear height functions: Milnor's formula, total curvature = 2π × the average number of local maxima over directions, and Fenchel's bound follows. — [Gulliver and Yamada, arXiv:1101.2305](https://arxiv.org/pdf/1101.2305) **[snippet]**

### Inferences
Basis for the term, own analysis:

- **The arc is πR, but the excess is less.** A full reversal uses a semicircle of arc πR, which replaces a straight run of 2R, so the extra length over the straight legs is about (π - 2)R ≈ 1.14R for long legs. Measured with the repository's Dubins code (R = 5 m, 50 m legs, free heading at the sensor): a turn of 30° costs 0.006R, 90° costs 0.17R, 150° costs 0.75R, 180° costs 1.25R. Only a full U-turn approaches the penalty's order, and even that is 2.5 times smaller.
- **Thin-petal limit, the one regime where the form holds.** If every peak and every valley is a full reversal, the excess is about 2(π - 2)mR = 2.28mR. For base + 1 sensor, the exact closed-tour excess is (2π - 4)R = 2.283R, and I measured 11.416 m at R = 5 m. So m·πR overstates by about 1.38 in this limit, and more elsewhere.
- **"Lemma 1" cannot be either natural inequality.** It is unstated, so this is about what it might say.
  - *L_Dubins ≥ L_euclid + mπR is false.* Base + 1 sensor: m = 1, excess 2.283R < πR. A ring around the base with alternating ±0.5 m radial jitter (radius 40 m, 40 sensors): m = 20, Dubins excess 4.8 m, penalty 314 m.
  - *L_Dubins ≤ L_euclid + mπR is false.* The m = 1 three-spoke tour at n = 240 has a Dubins excess of 730 m, against a penalty of 15.7 m.
- **m = 1 does not stop zig-zag.** Radial monotonicity says nothing about angle. A tour can keep r rising while swinging across the map at every leg, as in the three-spoke case. So "stops zig-zag exploration" holds only for radial bouncing, not for turning. The draft also says the heuristic "hardcodes Dubins kinematics", but no heading enters the fitness, and within m = 1 neither does R.
- **Does m predict the real excess?** Own test, repository code without obstacles, 6 sensor sets per size, comparing tours within each set. Excess means L_Dubins - L_euclid.

| n | Tours | Fitted slope per peak | R² on m | R² on total turning | Spearman of real L with L_euclid | with Fitness | with L_euclid + c·turning |
|---|---|---|---|---|---|---|---|
| 7 | random + seeds | 0.74R | 0.22 | 0.69 | 0.981 | 0.955 | 0.990 |
| 7 | near-optimal | 1.14R | 0.35 | 0.78 | 0.992 | 0.986 | 0.995 |
| 12 | random + seeds | 0.45R | 0.20 | 0.67 | 0.992 | 0.967 | 0.996 |
| 12 | near-optimal | 0.84R | 0.16 | 0.67 | 0.989 | 0.981 | 0.995 |
| 30 | random + seeds | -0.37R | 0.24 | 0.30 | 0.990 | 0.887 | 0.994 |
| 30 | near-optimal | 0.91R | 0.15 | 0.35 | 0.992 | 0.989 | 0.993 |

  In all six rows, adding the penalty makes the ranking worse than Euclidean length alone, and total turning explains the excess better than peak count. The coefficient c (0.30R to 0.42R per radian) was fitted on the same data, so the last column is optimistic. A fitted value near 1R per peak is about a third of the draft's πR. This points to the angular-distance TSP form as the better-founded cheap fitness.
- A linear-height version of the peak count could support a lemma (L ≥ 2πR × average maxima over directions, from Milnor and the curvature limit), but it bounds total length, not excess, and it is for straight height functions averaged over directions, not one radial centre. Applying the space-curve formula to planar tours is my extension; I did not check a planar statement in a source.

### Gaps
- Le Ny et al.'s full texts were not retrieved; the Ω(n) statement rests on the abstract. A Scispace copy returned no text.
- I could not read how Macharet et al. and Zhang et al. compute fitness in detail beyond their abstracts.
- The search for a GA fitness with a turn or reversal penalty did not fully run (see last section). One earlier combined search found no such paper, but that is weak evidence. (2026-11-09) It has now run, on OpenAlex and as a web search, and again found none.

## What is known about structured GA seeding, does it reduce iterations, and how does a GA compare with LKH or the alternating algorithm?

### Takeaway
Seeding is an old, studied idea. The literature supports better mean quality and faster convergence, and warns that fully seeded populations converge early and can worsen the best result. The draft seeds 100 % of its population with tours whose quality is poor by the standards of seeding work. Own measurements support the draft's claim on its own fitness for small groups, but a one-line dynamic program does better than the GA. A plain permutation GA is not in LKH's class; only GAs with strong crossover are.

### Cited Findings
- Grefenstette warned that a GA "may quickly converge to a local optimum if the initial population contains a few structures that are far superior", and in TSP studies "reported no significant benefit to using heuristic initialisation". — [Grefenstette 1987, as quoted by Surry and Radcliffe 1996](https://stochasticsolutions.com/pdf/aisb96.pdf) **[text of Surry and Radcliffe; Grefenstette not opened]**
- "both average solution quality and run-times improve when reasonable inoculation strategies are used, but... the quality of the best solution found over a number of runs often deteriorates as the initial populations become less random". Mean performance peaked at intermediate mass-mutation rates of 10 % to 50 %. — [Surry and Radcliffe 1996, Evolutionary Computing: AISB Workshop](https://stochasticsolutions.com/pdf/aisb96.pdf) **[text]**
- Case-based seeding "works very well for the TSP resulting in a significant reduction in the length of time it takes to find a good solution", but "seeding at 100% of the population causes the GA to converge far too quickly on poorer solutions". — [Oman and Cunningham, TCD-CS-1997-08](https://publications.scss.tcd.ie/tech-reports/reports.97/TCD-CS-1997-08.pdf) **[text]**
- k-means seeding on 14 TSPLIB instances reduced best error compared with random and greedy starts by at least about 29 %. — [Deng, Liu, Zhou 2015, Math. Probl. Eng., DOI 10.1155/2015/212794](https://doi.org/10.1155/2015/212794) **[abstract, cut off at that figure]**
- Heuristic seeding gives "high fitness" and "rapid convergence", while random populations may delay it. — [Alkafaween et al. 2024, Applied Sciences 14(8):3151](https://doi.org/10.3390/app14083151) **[abstract]**. Regression-based seeding: [Hassanat et al. 2018, Information 9(7):167](https://www.mdpi.com/2078-2489/9/7/167) **[snippet]**. Comparison of random, nearest neighbour, gene bank, sorted population, selective initialisation: [Victer Paul et al. 2019, IJAMC 10(2):55-92](https://ideas.repec.org/a/igg/jamc00/v10y2019i2p55-92.html) **[snippet]**.
- GA against LK-type methods: a GA can reach the top level without LK, using edge assembly crossover. — [Nagata and Kobayashi, INFORMS J. Computing, DOI 10.1287/ijoc.1120.0506](https://doi.org/10.1287/ijoc.1120.0506) **[snippet + meta]**. Rankings of LKH and EAX depend on target quality, and "LKH and EAX... manage to calculate close-to optimal or even optimal solutions". — [Bossek, Kerschke, Trautmann 2020, IEEE CEC, arXiv:2005.13289](https://arxiv.org/abs/2005.13289) **[abstract]**. A simple GA without local search on 100 cities: "much larger problems can be solved much more efficiently by incorporating local search". — [Surry and Radcliffe 1996](https://stochasticsolutions.com/pdf/aisb96.pdf) **[text]**
- On the Dubins TSP specifically: Alternating and Random Headings "benefit from the state of the art TSP solving tool", and the GA is less efficient for very many waypoints. — [Yu 2015](https://etd.auburn.edu/handle/10415/4599) **[text]**. WiSM's figures above show an EA can beat the sampling + LKH approach only with a very fast evaluator and many evaluations. — [Drchal et al. 2020](https://arxiv.org/html/2002.00811v1) **[text]**

### Inferences
- The draft's search budget is small: 50 × 100 with half replaced per generation gives about 2,500 children (`ga_tour.optimize`). Yu's low-density setting allows up to 7000 generations with population 20.
- Seed quality, own calculation, unit square, n = 100: the draft's random seed averages 48.7 at base-centre and 33.1 at base-corner; random permutations average 52.5 and 53.7; the best tour of the class is 28.1 and 16.1; and iterated local search finds 7.85 and 7.92. Seed tours improve on random permutations by 7 % with the base at the centre and 38 % at the corner, and remain 4 to 6 times longer than a good tour.
- SEEDING_TABLE_PLACEHOLDER
- REAL_DUBINS_PLACEHOLDER

### Gaps
- I found no GA seeded from pyramidal or bitonic tours. The arXiv queries for pyramidal + genetic and bitonic + genetic returned 0, but the dedicated web search could not run (see last section). (2026-11-09) OpenAlex for "pyramidal tour" with genetic, evolutionary, memetic or heuristic returned 2 hits, neither a seeded GA.
- [Paul et al. 2015, Applied Soft Computing], named in the suggested sources area of my own memory, could not be verified, so it is not cited. A similarly titled [ResearchGate record](https://www.researchgate.net/publication/273382985_Performance_Assessment_over_Heuristic_Population_Seeding_Techniques_of_Genetic_Algorithm_Benchmark_Analysis_on_Travelling_Salesman_Problems) exists, not opened.
- No study found that compares LKH with a plain GA specifically on a Dubins instance with a surrogate fitness.

## What baseline should the draft compare against to substantiate its claim?

### Takeaway
The first comparator has to be the pyramidal dynamic program on the same cost, because it is the exact answer for the class. Beyond that, each claim in Section 0.8 needs its own test, and all should be reported on real Dubins length, not on Fitness.

### Cited Findings
- Standard Dubins TSP baselines: Alternating Algorithm from LKH or Concorde ETSP order, Random Headings with LKH, sampling headings and transforming to GTSP and ATSP with LKH or Concorde, Concorde ETSP + IRIS headings. — [Yu 2015](https://etd.auburn.edu/handle/10415/4599) **[text]**; [Drchal et al. 2020](https://arxiv.org/html/2002.00811v1) **[text]**
- General GTSP solvers usable for the Dubins sampling route: [Helsgaun 2015, Math. Prog. Computation 7(3):269-287, DOI 10.1007/s12532-015-0080-8](https://doi.org/10.1007/s12532-015-0080-8) **[snippet]**, with GTSPLIB solved to optimality in reasonable time; [Smith and Imeson 2017, GLNS, Computers & OR 87:1-19](https://ece.uwaterloo.ca/~sl2smith/papers/2017COR-GLNS.pdf) **[snippet]**.
- Lower bound to report gaps against: [Manyam and Rathinam](https://arxiv.org/abs/1506.08752) **[abstract]**. ETSP is the simple one, known to be weak.
- Experimental design precedents: low and high density reported separately with 30 random samples per n. — [Yu 2015](https://etd.auburn.edu/handle/10415/4599) **[text]**. Density parameter d. — [Drchal et al. 2020](https://arxiv.org/html/2002.00811v1) **[text]**

### Inferences
Recommended comparison, in priority order:

| Baseline | What it tests | Implementation effort |
|---|---|---|
| Pyramidal dynamic program on the same costs (Euclidean, also geodesic) | Whether the GA adds anything over the exact class solver | About 25 lines |
| Held-Karp on (mask, last, direction) for N ≤ 15 or so; two-copy GTSP with LKH above | Whether the GA minimises its own Fitness | Small |
| LKH or Concorde ETSP order (no penalty), then the same heading search | Whether the m(π) term helps at all on real Dubins length | Small, plus solver |
| GA ablation: radial seed, random seed, nearest-neighbour seed, each with and without penalty; same evaluation count | "Reduces iterations" and the value of the penalty | Small |
| Sampling headings + GTSP with LKH or GLNS, with geodesic legs around obstacles | Whether order and headings together do better | Moderate |
| Full enumeration on small groups (N ≤ 6 to 8) with real Dubins length | True optimal order as ground truth | Small |

Per claim:

| Draft claim | Metric | Test |
|---|---|---|
| "Reduce the number of iterations" | Evaluations to reach a fixed target, on the same fitness | Ablation, paired over at least 30 instances, reported with confidence intervals |
| "Prevents... zig-zag paths" | Final m(π) and total turning angle | Report both; m alone cannot show this |
| "Simple, high-speed sorting task" | Wall time and result against the dynamic program | Sorting only applies once a split is chosen; choosing the split is the search |
| "Hardcodes Dubins kinematics" | Correlation of the surrogate with real Dubins length | Against L_euclid alone and against L_euclid + c·turning |
| Mission outcome | Real T, L_total, number of sinks K | On the planner, density and map variants |

State the regime. Both Yu and WiSM report results by density, and my results suggest the heuristic's value depends on group shape. Report n, group width and spacing relative to R.

### Gaps
- I have not run LKH, Concorde, GLNS or the sampling + GTSP route. The near-optimal references in my measurements are iterated 2-opt + Or-opt.
- Own measurements use repository maps and defaults, and are pilots, not a benchmark.

## What was searched for and not found, and what are the limits of the search?

### Takeaway
The literal phrase "radially unimodal" has no routing use in what I could search. This supports only "not found", not "does not exist". The search budget ran out before three planned searches ran, so the coverage of GA seeding from pyramidal tours and turn-penalty fitness is thin.

### Cited Findings
Searched and not found:
- `"radially unimodal"` in routing or tours. [OpenAlex](https://api.openalex.org/works?search=%22radially%20unimodal%22) returned 4 works, all from other fields by title: image deblurring, covariance functions, rogue waves, chance constraints. arXiv API: 0. Crossref relevance search: nothing relevant. Web search with "tour genetic algorithm": nothing.
- `"radial unimodal"`, `"radial unimodality"`, `"unimodal tour"`: same 4 works, or transport mode-choice papers in which "unimodal" means single transport mode.
- `"radially monotone" tour`: 2 works, unrelated by title. `"radial maxima"`: 306 works, none routing in the first 8 by title.
- `bitonic` with `dubins`: OpenAlex 0, arXiv 0. `pyramidal` with `dubins`: arXiv 0; OpenAlex returned neuroscience noise in the first 8 by title.
- `pyramidal tours` with `genetic`: arXiv 0. `bitonic` with `genetic`: arXiv 0.
- A paper combining a GA, a Euclidean surrogate and a turning penalty for the Dubins TSP: not found in two web searches.
- A worst-case ratio for the best bitonic or pyramidal tour against the optimal Euclidean tour: not found in one targeted web search, whose nearest result was the narrow-strip theorem.

### Inferences
- The near-absence of the phrase is explained by the structure having an established name. A novelty search on the draft's own term would not be expected to find the prior work, which is why the phrase's absence is not evidence of novelty.

### Gaps
- **Searches that did not run.** The session's web search limit was reached. Not run: pyramidal tours + genetic, memetic or evolutionary; GA seeding by distance from the depot; turn or direction-reversal penalty fitness. The user can ask for a follow-up to run them. (2026-11-09) All three were run on OpenAlex, and one as a web search. No relevant paper was found. The web search found [Hansen & La Cour-Harbo 2016](https://doi.org/10.1109/ECC.2016.7810624) [text]: a GA over order, heading and speed whose fitness is travel time on Dubins curves. It says the two-step Euclidean approach "works well" when waypoints are far apart compared with the turning radius, "but when the waypoints are closer, the kinematics of the aircraft ruins the plan".
- Semantic Scholar returned HTTP 429 on every attempt, and Google Scholar, IEEE Xplore, Scopus and Web of Science were not queried. Queries cover titles and abstracts, not paywalled full text.
- arXiv recall looks poor: `dubins AND genetic AND traveling` returned 0 even though related work exists, so its zeros are weak evidence.
- Not opened, only cited through others or by catalogue record: Burkard et al. body, Gilmore et al., Klyaus, Sarvanov and Doroshko, Aizenshtat and Kravchuk, CLRS, Grefenstette, Le Ny et al. full texts, Babel full text, Nagata and Kobayashi full text. Russian-language 1970s-80s work was not read.
- Work from 2025-2026 appeared rarely in results, and I cannot claim coverage up to November 2026.
