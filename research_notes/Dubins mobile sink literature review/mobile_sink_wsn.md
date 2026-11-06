# Mobile sink data collection in WSNs: multiple sinks, obstacles and vehicle kinematics (novelty assessment)

**How to read the evidence tags.** Every paper below was confirmed to exist by a source I opened or a search result that showed it. The tag says how much of it I actually read:

- **[FT]** I read the relevant parts of the full text directly (PDF converted to text).
- **[FT*]** Full text, but read through a fetch tool's summary, not directly.
- **[ABS]** I read the abstract, sometimes cut off at about 1100 characters. **[ABS*]** means read through a summary.
- **[SNIP]** I saw only a search tool's excerpt or summary of the page.
- **[META]** A Crossref record or reference list confirms title, authors, venue and DOI. Anything said about content is inferred from the title.
- **[UNVERIFIED]** From memory, not confirmed by any source.

"Draft" means the paper being assessed.

## How close is the prior art overall?

### Takeaway
No single paper I found combines all of: fleet-size minimisation under a time budget, Dubins kinematics, polygonal obstacles, and explicit conflict resolution between sinks. Parts of this are well covered (Dubins teams with obstacles; fleet minimisation with obstacles; obstacles with wait-based conflict repair), so a claim of novelty can rest only on the combination, and two unread 2026 papers could still weaken it.

### Cited Findings

| Work | Min. fleet size? | Dubins? | Obstacles? | Inter-sink collisions? | Radio / data model | Read |
|---|---|---|---|---|---|---|
| **Draft** | Yes, first objective | Yes | Polygons | Yes, game or wait | None, fixed dwell | n/a |
| [Szénási & Harmati 2024, Acta IMEKO 13(4)](https://doi.org/10.21014/actaimeko.v13i4.1572) | No, k in [2, 8] is an input | Yes | 15 obstacles | None found | Data load and rate | FT |
| [Szénási & Harmati 2024, Period. Polytech. 68(3)](https://doi.org/10.3311/PPee.22788) | No, k in [2, 5] | Yes | 15 obstacles | None found | Stepped rate zones | FT |
| [Huang & Savkin 2017, Comput. Commun. 111](https://doi.org/10.1016/j.comcom.2017.07.010) | No, k is an input | Yes | Yes | None found | Contact time, two-step rates | FT (chapter in [arXiv 1712.02436](https://arxiv.org/pdf/1712.02436)) |
| [Roberge & Tarbouchi 2021, Sensors 21(20)](https://doi.org/10.3390/s21206851) | No, 3 UAVs | No | No-fly zones | Yes, wait repair | 170 m range, SNR model | FT* |
| [Xu et al. 2022, IEEE/ACM ToN 30(2)](https://doi.org/10.1109/tnet.2021.3123606) | **Yes, 4-approx.** | No | None found | None found | Data volume and rate; radio range in one variant | FT |
| [Kumar & Mukherjee 2023, Ad Hoc Netw. 149](https://doi.org/10.1016/j.adhoc.2023.103239) | **Yes, constant factor** | Unknown | Yes | Unknown | Unknown | SNIP |
| [Kumar & Mukherjee 2026, COMSNETS](https://doi.org/10.1109/comsnets67989.2026.11418194) | **Yes (title)** | Unknown | Yes (CV) | Unknown | Unknown | META |
| [Fu et al. 2026, IEEE T-ITS 27](https://doi.org/10.1109/tits.2025.3645094) | Unknown | **Yes (title)** | Unknown | Unknown | Unknown | META |
| [Rahman et al. 2022, Appl. Sci. 12(22)](https://doi.org/10.3390/app122211509) | No, minimises total distance | Not stated | Yes | Not stated | Not stated | ABS |

"None found" means a word search of the text I had for robot-to-robot collision language found nothing. It is not proof of absence.

### Inferences
- The defensible novelty claim is the combination, plus any specific result such as the potential-game resolver. "First to use Dubins vehicles in WSN data collection" would be false: see [Macharet et al. 2012](https://doi.org/10.1109/cec.2012.6256477), [Wichmann & Korkmaz 2015](https://doi.org/10.1016/j.comcom.2015.06.001) and Huang & Savkin 2017.
- The nearest neighbours are the Budapest group (Szénási, formerly Olasz-Szabó, and Harmati). Their setting is strikingly similar to the draft: 200 m by 200 m, 15 obstacles, 40 nodes, Dubins cars. The draft does not cite them, as far as the brief shows. A reviewer from this line would notice.
- Roberge & Tarbouchi is the nearest neighbour on method: genetic algorithm plus 2-opt, no-fly zones, then a pairwise check at equal time steps repaired by waiting. That is close to the draft's GA and `simple_resolver`.

### Gaps
- Fu et al. 2026 and Kumar & Mukherjee 2026 could be close prior art, and I read neither (see the last section). The user should read both before claiming novelty.

## What does Kumar and Mukherjee (2023) do?

### Takeaway
It minimises the number of mobile sinks needed to collect data periodically from **mobile** sensors while avoiding obstacles, using two constant-factor approximation algorithms. I could not read the full text, so everything beyond that summary is unknown and must not be reported as fact.

### Cited Findings
**Bibliography (confirmed).** Rohit Kumar and Joy Chandra Mukherjee, "Approximation algorithms for vehicle-aided periodic data collection from mobile sensors with obstacle avoidance in WSNs", Ad Hoc Networks 149, 103239, October 2023 — [Crossref record for DOI 10.1016/j.adhoc.2023.103239](https://doi.org/10.1016/j.adhoc.2023.103239). The draft's citation details match.

**Read only as search excerpts of the ScienceDirect page [SNIP]:**
- It defines the Minimum Mobile Sink Aided Periodic Data Collection (MinSnkDC) problem: determine the minimum number of sinks (SNKs) that collect data in every time period from the mobile sensors, while avoiding collision with obstacles — [ScienceDirect listing](https://www.sciencedirect.com/science/article/abs/pii/S1570870523001592)
- Two constant-factor approximation algorithms are proposed, MinSnkDC and Modified MinSnkDC (M-MinSnkDC) — [same listing](https://www.sciencedirect.com/science/article/abs/pii/S1570870523001592)
- The problem is proved NP-complete (Theorem 1, Section 4), and the approximation bounds are in Theorems 3 and 4 (Sections 5.1 and 5.2) — [same listing](https://www.sciencedirect.com/science/article/abs/pii/S1570870523001592)
- M-MinSnkDC is reported to need fewer sinks than existing sink-based algorithms — [same listing](https://www.sciencedirect.com/science/article/abs/pii/S1570870523001592)
- The likely motivation is limited sensor memory, requiring collection within a set interval to avoid overflow. This came from a search summary covering several results, so its attribution to this paper is probable, not certain — [same listing](https://www.sciencedirect.com/science/article/abs/pii/S1570870523001592)

**Read directly in the first author's CV [FT]:**
- The thesis, "Charging and Data Collection in Wireless Rechargeable Sensor Networks" (IIT Bhubaneswar), describes approximation algorithms "to determine the minimum number of SNKs that collect data from all the MSs in each time period while avoiding collision with obstacles in the area where an SNK has sufficient energy to visit the MSs" — [CV](https://svnit.ac.in/facup/ResumeRohit.pdf)
- The same thesis also handles "the MSs and SSs in the presence of obstacles using multiple SNKs, where each SNK can only travel a fixed distance", again finding the minimum number of sinks and delivering to the base station — [CV](https://svnit.ac.in/facup/ResumeRohit.pdf)

**Lineage.**
- [Dash 2018, Pervasive Mob. Comput. 46](https://doi.org/10.1016/j.pmcj.2018.02.006) [ABS] has the same core: mobile sensors moving on predefined paths, periodic collection with the minimum number of sinks, then a visit to the base station. It proves NP-hardness and gives two approximation algorithms.
- [Tazeen, Dash & De 2022, Wireless Netw. 28](https://doi.org/10.1007/s11276-022-02915-z) [SNIP] extends this to unpredictable motion along road networks.

### Inferences
- MinSnkDC reads as the Dash (2018) problem with obstacles added, but I could not confirm that the paper says so.
- Differences from the draft that I can support: its sensors move, while the draft's are static; and it has approximation guarantees, while the draft's bisection has none that I know of. On the draft's first objective it is prior art, not merely background.
- The closest paper in this line is probably not the 2023 one. [Kumar & Mukherjee 2026, COMSNETS, pp. 1-9](https://doi.org/10.1109/comsnets67989.2026.11418194), "An Approximation Algorithm for Minimizing the Number of Distance Constrained Mobile Sinks for Data Collection from Static and Mobile Sensors" [META], covers static sensors and a per-sink distance bound, which is equivalent to the draft's time budget at constant speed and without dwell. The CV description suggests it also has obstacles.

### Gaps
- **Not known, because I could not read the full text:** the approximation ratios, the obstacle model, how obstacle-avoiding distances are computed, whether sinks have any kinematic limits, whether sinks avoid one another, radio range, simulation setup and baselines.
- I tried the ScienceDirect article and abstract pages (403), the Elsevier API listed in Crossref (400 without a key), and Crossref and Semantic Scholar for an abstract (none). [OpenAlex](https://api.openalex.org/works/doi:10.1016/j.adhoc.2023.103239) marks it closed access with no repository copy. The IIT Bhubaneswar repository refused the connection.

## What are the standard problem formulations and key surveys?

### Takeaway
The field has four framings: uncontrolled or controlled data mules, deadline-driven mobile element scheduling, rendezvous or relay-assisted collection, and geometric tours through radio neighbourhoods. Most of those I read treat the sink as a point that collects by radio, and the draft's direct-visit, no-radio model is the simplest special case.

### Cited Findings
**Foundations**
- Data MULEs: [Shah, Roy, Jain & Brunette 2003, SNPA](https://doi.org/10.1109/snpa.2003.1203354) [META, SNIP]. Coined the term, with uncontrolled carriers in a three-tier architecture.
- Mobile element scheduling: [Somasundara, Ramamoorthy & Srivastava 2004, RTSS](https://doi.org/10.1109/real.2004.31) and [2007, IEEE TMC 6](https://doi.org/10.1109/tmc.2007.57) [SNIP]. Heuristics based on earliest-deadline-first choose the path so as to minimise buffer overflow at each sensor.
- Rendezvous: [Xing et al. 2008, IEEE TMC 7](https://doi.org/10.1109/tmc.2008.58) and [Xing et al. 2008, MobiHoc](https://doi.org/10.1145/1374618.1374650) [SNIP]. Sensors relay to chosen stop points. A dedicated review is [Mehto, Tapaswi & Pattanaik 2020, Wireless Netw. 26](https://doi.org/10.1007/s11276-019-02022-6) [META].
- Bounded-hop and clustering: [SenCar, Ma & Yang 2006](https://doi.org/10.1007/11776178_30), [Zhao & Yang 2009](https://doi.org/10.1109/mobhoc.2009.5336976), [Zhao, Yang & Wang 2015, IEEE TMC 14](https://doi.org/10.1109/tmc.2014.2338315) [META].
- Tour planning with several collectors: [Ma, Yang & Zhao 2013, IEEE TVT 62](https://doi.org/10.1109/tvt.2012.2229309) [SNIP]. Several collectors follow shorter subtours at once to meet strict time limits. The summary describes a TSP or covering-salesman formulation.
- Lifetime-driven sink mobility: [Gandham et al. 2003](https://doi.org/10.1109/glocom.2003.1258265), [Luo & Hubaux 2005](https://doi.org/10.1109/infcom.2005.1498454), [Wang et al. 2005](https://doi.org/10.1109/hicss.2005.259) [META].
- TSP with neighbourhoods: [Yuan, Orlowska & Sadiq 2007, IEEE TKDE 19](https://doi.org/10.1109/tkde.2007.1062) [META]; that it is TSPN-based is [UNVERIFIED]. [Xu et al. 2022](https://doi.org/10.1109/tnet.2021.3123606) [FT] reviews TSPN with equal disks, citing a 6.75-approximation by Dumitrescu and Mitchell.
- Harvesting with robots: [Tekdas et al. 2009, IEEE Wireless Commun. 16](https://doi.org/10.1109/mwc.2009.4804365), [Tekdas, Bhadauria & Isler 2012, IJRR 31](https://doi.org/10.1177/0278364912439429) [META].

**Label-covering tours and data mule scheduling**
- In the data mule scheduling work of Sugihara and Gupta, each edge of a graph carries labels, and a "label-covering" tour is one where, "for any label, there exists at least one edge in the tour that contains the label". The objective is the minimum-cost such tour, using Euclidean cost as a proxy for travel time — [Sugihara & Gupta, "Optimizing Energy-Latency Trade-off in Sensor Networks with Controlled Mobility"](https://escholarship.org/content/qt67p7k1pr/qt67p7k1pr_noSplash_0a88a0ce25c94e64d605b9424810170e.pdf) [FT, this passage only; I did not confirm its venue or year].
- Related papers: [Sugihara & Gupta 2008, DCOSS](https://doi.org/10.1007/978-3-540-69170-9_26) and [2011, ACM TOSN 8](https://doi.org/10.1145/1993042.1993043) [SNIP]. The motion problem is split into path choice, speed changes and collection timing, with the aim of minimising delivery latency.

**Surveys**
- [Di Francesco, Das & Anastasi 2011, ACM TOSN 8](https://doi.org/10.1145/1993042.1993049) [SNIP]: discovery, data transfer and routing by mobile elements.
- [Khan et al. 2014, Sensors 14](https://doi.org/10.3390/s140202510) [ABS]: classes are path constrained, path unconstrained and controlled sink mobility. States that mobile sinks "alleviate energy-holes issues".
- [Gu, Ren, Ji & Li 2016, IEEE COMST 18](https://doi.org/10.1109/comst.2015.2388779) [META].
- [Huang, Savkin, Ding & Huang 2019, Comput. Netw. 148](https://doi.org/10.1016/j.comnet.2018.10.018) [ABS]: taxonomy by task, namely collection, delivery and combination. It is the survey closest to a robotics view.
- [Nguyen & Nguyen 2019, arXiv 1906.07063](https://arxiv.org/abs/1906.07063) [FT, sections only]: lifetime, classified by mobile sensors, one sink or several sinks.
- [Singh & Kumar 2020, JAIHC 11](https://doi.org/10.1007/s12652-019-01268-4) [SNIP]: trajectory schemes for all kinds of mobile element.
- [Temene, Sergiou, Georgiou & Vassiliou 2022, Ad Hoc Netw. 125, 102726](https://doi.org/10.1016/j.adhoc.2021.102726) [META]: cited by the draft. I found no abstract, so I cannot say what it covers.
- [Wei et al. 2022, IEEE IoT J. 9](https://doi.org/10.1109/jiot.2022.3176903) [ABS]: UAV-assisted collection, covering clustering, collection mode and joint path and resource planning.
- [Matin & Islam 2012, InTech chapter](https://doi.org/10.5772/49376) [META]: cited by the draft. A general WSN overview, judging by its title; not a mobile sink source.

### Inferences
- In the terms of Khan et al. the draft is "controlled sink mobility". In terms of the Sugihara and Gupta framework, it fixes the speed model and collection schedule and solves only the path.

### Gaps
- I did not read the full text of the surveys, so I cannot say how many of them discuss kinematics or inter-sink collisions.

## How is the number of sinks chosen, how are sensors partitioned, and how are tours computed under time limits?

### Takeaway
Most multi-sink papers take the number of sinks k as an input and minimise the longest tour. Minimising the fleet under a time bound is an established problem with constant-factor algorithms, including a direct-visit variant, and the draft's tour-time definition matches it.

### Cited Findings
**Fleet size minimised, with a bound**
- [Xu et al. 2022](https://doi.org/10.1109/tnet.2021.3123606) [FT, [PDF](https://www.cs.cityu.edu.hk/~weliang/papers/XXZLXLJD22.pdf)] studies deploying "the minimum number of UAVs", where each tour's total time, meaning flying time plus data collection time, must not exceed B. Its first variant requires the UAV to fly to each device, as the draft does. It gives a 4-approximation for that variant and the first constant-factor algorithm when a radio range is allowed.
- Its results: 11-19 % fewer UAVs than benchmarks on average. At n = 300 the means were 19.1 against 24.4, 23.9 and 22.6 — [Xu et al. 2022](https://www.cs.cityu.edu.hk/~weliang/papers/XXZLXLJD22.pdf). Earlier version: [Zhang et al. 2021, INFOCOM](https://doi.org/10.1109/infocom42981.2021.9488887) [SNIP].
- History of the unrooted problem, as Xu et al. report it: 6-approximation (Arkin, Hassin & Levin 2006), 5-approximation (Khani & Salavatipour 2014), 4 2/3 and 4 4/7 (Yu, Liu & Bao 2019), then 4 — [Xu et al. 2022](https://www.cs.cityu.edu.hk/~weliang/papers/XXZLXLJD22.pdf) [FT; these sources are seen only in its reference list].
- **Rooted case, which is the draft's** (every tour starts and ends at the base): Nagarajan & Ravi 2012 give O(log B), though tours may exceed the bound. Liang et al. 2016 and Zhang et al. (Wireless Netw. 25) give algorithms that respect it — [Xu et al. 2022](https://www.cs.cityu.edu.hk/~weliang/papers/XXZLXLJD22.pdf) [FT; sources seen only in its reference list; both concern charging vehicles].
- Xu et al. warn that searching K = 1 to n with a min-max solver until the longest tour fits B "does not deliver a constant approximate solution" — [Xu et al. 2022](https://www.cs.cityu.edu.hk/~weliang/papers/XXZLXLJD22.pdf).
- WSN mobile sink versions: [Dash 2018](https://doi.org/10.1016/j.pmcj.2018.02.006) [ABS], Kumar & Mukherjee 2023 and 2026, and [Srinivas, Donta & Amgoth 2020, Comnetsat](https://doi.org/10.1109/comnetsat50391.2020.9328947) [META].

**k given, longest tour minimised**
- [k-SVPP in Huang & Savkin 2017](https://arxiv.org/pdf/1712.02436) [FT]: builds one viable path, splits it by k-SPLITOUR (Frederickson, Hecht & Kim 1978), then rebuilds each path. It minimises the longest path and is tested at k = 2 and 3.
- [Szénási & Harmati 2024](https://doi.org/10.21014/actaimeko.v13i4.1572) [FT]: k-ACO builds clusters and visiting order together, with 2-opt on each subtour, and reports "k paths of approximately equal lengths for a variable number of robots". It also has a multi-base-station variant.
- [Deng et al. 2020, IEEE/ACM ToN 28](https://doi.org/10.1109/tnet.2020.2999630) [ABS]: min-max cycle cover with neighbourhoods; K is given.
- [Roberge & Tarbouchi 2021](https://pmc.ncbi.nlm.nih.gov/articles/PMC8541541/) [FT*]: k-means for download points, then a GA with 2-opt for assignment and order; the mission time of the slowest UAV is minimised; three UAVs are fixed.

**Other objectives under a bound**
- [Rahman et al. 2022](https://doi.org/10.3390/app122211509) [ABS]: an integer linear program minimising total distance under flight-time limits.
- [Almi'ani, Viglas & Libman 2010, LCN](https://doi.org/10.1109/lcn.2010.5735777) [META]: tour-length-constrained mobile elements.

### Inferences
- Reading the Xu tours as unrooted is my inference: the paper says UAVs can forward data by 4G/5G and treats "single-rooted" as a separate problem. So the 4-approximation may not transfer directly to the draft.
- I saw no paper stating the draft's three-level ordering of fleet size, then longest time, then total length. A reviewer familiar with Xu et al. would ask what guarantee the draft's bisection has. Benchmarks from this line (BTCAlg, MCCPAlg1/2) are obvious comparators and could be run with Dubins distances.

### Gaps
- I did not search specifically for lexicographic objectives, so that absence is weak evidence.

## What exists on mobile sink routing among obstacles?

### Takeaway
Obstacle-aware sink routing exists and is growing, but most work still assumes free space. Methods range from tangent graphs to RRT, and most treat the sink as a point without turning limits.

### Cited Findings
- "In most of the examined works, the MSs are deemed to move in a free space" — [Nguyen & Nguyen 2019](https://arxiv.org/pdf/1906.07063v2.pdf) [FT]. A 2024 paper likewise says few algorithms consider obstacles — [Gowthami et al. 2024, Contemp. Math.](https://doi.org/10.37256/cm.5120243026) [SNIP].

| Method | Paper | Notes | Read |
|---|---|---|---|
| Tangent graph, then ATSP, then dynamic programming | [Huang & Savkin 2017](https://arxiv.org/pdf/1712.02436) | Safety margins around obstacles, visiting circles around nodes | FT |
| Tangent graph with obstacles added to the sequence, then Dijkstra | [Olasz-Szabó & Harmati 2022, Acta IMEKO 11(3)](https://distantreader.org/stacks/journals/actaimeko/actaimeko-1254.pdf) | Up to 92 m shorter than the earlier assumptions | FT |
| Detour points by vector rotation, GA, Dijkstra | [Rahman et al. 2022](https://doi.org/10.3390/app122211509) | Multiple UAVs | ABS |
| Clustering with ACO, GA, single sink tour | [Najjar-Ghabel et al. 2020, Ad Hoc Netw. 106](https://doi.org/10.1016/j.adhoc.2020.102243) | DGOB; balances energy and delay; collects from cluster heads | ABS |
| Inner-centre path, back-routing avoidance | [Chang et al. 2020, EURASIP JWCN](https://doi.org/10.1186/s13638-020-01873-4) | Single sink; radio range and obstacles | ABS |
| Bug 2 with cat swarm | [Gowthami et al. 2024](https://doi.org/10.37256/cm.5120243026) | | SNIP |
| RRT | [Boyineni et al. 2023, JAIHC 15](https://doi.org/10.1007/s12652-023-04717-3) | | META |
| Avoid whole regions, Warshall-Floyd | [Xie et al. 2017, Peer-to-Peer Netw. Appl.](https://arxiv.org/pdf/1906.07063v2.pdf) | As described in Nguyen & Nguyen | FT of the review |
| Unreachable regions, reduced to TSP | [Nie, Wu & Zheng 2017, MSWiM](https://doi.org/10.1145/3127540.3127544) | Urban and forest scenarios | SNIP |
| VNS with PRM* | [Pěnička, Faigl & Saska 2019, RA-L](https://comrob.fel.cvut.cz/papers/ral19pop.pdf) | Orienteering; arbitrary motion model, including Dubins | FT |
| 3D grid with no-fly zones | [Roberge & Tarbouchi 2021](https://pmc.ncbi.nlm.nih.gov/articles/PMC8541541/) | Insertion of zones into the graph is not detailed | FT* |

- Further title-level hits: [Chanak et al. 2014, IEEE TCE](https://doi.org/10.1109/tce.2014.7027292), [Feiroz Khan & Siva Kumar 2019, IJEECS](https://doi.org/10.11591/ijeecs.v15.i2.pp879-887), [Zheng & Liu 2025, IEEE TVT 74](https://doi.org/10.1109/tvt.2024.3419842), [Zhong et al. 2025, MSN](https://doi.org/10.1109/msn69125.2025.00011), [Jenabi et al. 2026, IEEE Access](https://doi.org/10.1109/access.2026.3710676), [Karunakar et al. 2026, ICCSC](https://doi.org/10.1109/iccsc67078.2026.11468635), [Niranjan et al. 2026, ICECCC](https://doi.org/10.1109/iceccc70334.2026.11633036) [META]. A paper titled ["Cluster-Based Routing for the Mobile Sink in Wireless Sensor Networks With Obstacles"](https://www.researchgate.net/publication/301645973_Cluster-Based_Routing_for_the_Mobile_Sink_in_Wireless_Sensor_Networks_With_Obstacles) appeared in results [SNIP; authors and venue not confirmed].

### Inferences
- Combining obstacles with curvature limits is much rarer than obstacles alone. The clear cases I read are the Huang and Savkin line and the POP extension. The draft's use of geodesic distances for ordering and Dubins paths for feasibility belongs to this smaller group.

### Gaps
- For most title-level hits I do not know whether the sink has kinematic limits.

## Which WSN or UAV data-collection papers use Dubins or curvature-constrained kinematics?

### Takeaway
There is a real, if thin, line of 2D Dubins data collection going back to at least 2012, and it treats data transfer as contact time while moving. The Faigl and Pěnička line is mostly single-vehicle orienteering with neighbourhoods rather than full coverage by a fleet.

### Cited Findings
**Framed as WSN data collection**
- [Macharet et al. 2012, IEEE CEC](https://doi.org/10.1109/cec.2012.6256477) [ABS]: nonholonomic vehicle with curvature limits, RF footprint intersections choose waypoints, a metaheuristic orders them, Dubins paths make the tour feasible. It appears to use a single vehicle.
- [Wichmann & Korkmaz 2015, Comput. Commun. 72](https://doi.org/10.1016/j.comcom.2015.06.001) [SNIP]: TSP-based smooth paths, adjusted for required contact time, extended to multiple sinks to cut average delay per packet.
- [Huang & Savkin 2017](https://doi.org/10.1016/j.comcom.2017.07.010) [FT of a chapter with the same title in [arXiv 1712.02436](https://arxiv.org/pdf/1712.02436), a report whose cover says "Guided by: Hailong Huang"; I did not read the journal version]: a "viable" path is smooth, free of collision with nodes and obstacles, closed, and gives enough contact time. It is formulated as a variant of DTSPN; the chapter calls it "a sampled DTSPN".
- [Olasz-Szabó & Harmati 2022](https://distantreader.org/stacks/journals/actaimeko/actaimeko-1254.pdf) and the two [Szénási & Harmati 2024](https://doi.org/10.21014/actaimeko.v13i4.1572) papers [FT], which add teams, priority nodes and moving nodes.
- [Fu, Sun, Yao, Wu & Wu 2026, IEEE T-ITS 27, 2214-2224](https://doi.org/10.1109/tits.2025.3645094) [META]: "Dubins Path Planning of Heterogeneous UAV Collaborative Data Collection for IoT Network". The title suggests it is highly relevant, but IEEE Xplore returned an empty page and I have no abstract.
- Underwater: [Cai & Zhang 2017, Electron. Lett. 53](https://doi.org/10.1049/el.2016.3836) [SNIP] and [2018, Sensors 18](https://doi.org/10.3390/s18072105) [FT*]: 3D Dubins curves, one AUV, 3-19 nodes, no obstacles.
- [Kuo et al. 2021, IEEE TGCN 5](https://doi.org/10.1109/tgcn.2021.3093453) [META]: fixed-wing multi-UAV data gathering; I do not know whether it uses Dubins paths.

**CTU Prague line (Faigl, Pěnička, Váňa and others)**
- [Dubins Orienteering Problem, Pěnička et al. 2017, IEEE RA-L](https://doi.org/10.1109/lra.2017.2666261) [META].
- [DOPN, Pěnička et al. 2017, ICUAS](https://doi.org/10.1109/icuas.2017.7991350) [SNIP]: reward collected within a neighbourhood, solved by Variable Neighborhood Search, motivated by UAV data collection with a battery budget.
- [Pěnička et al. 2019, Auton. Robots](https://doi.org/10.1007/s10514-019-09844-5) [ABS*]: the same problem, verified on a real hexarotor.
- [Váňa et al. 2017, ECMR](https://doi.org/10.1109/ecmr.2017.8098715) [SNIP]: Dubins airplane with climb and dive limits and a travel budget.
- [Physical Orienteering Problem, Pěnička, Faigl & Saska 2019](https://comrob.fel.cvut.cz/papers/ral19pop.pdf) [FT]: names WSN data collection only as a motivating example.
- [Deckerová & Faigl 2024, WSOM](https://doi.org/10.1007/978-3-031-67159-3_2) [ABS*, [author page](https://deckejin.github.io/publications/deckerova24wsom.html)]: Close Enough Dubins TSP with Time Constraints. The vehicle must stay near each station for a minimum time, solved with a modified Growing Self-Organizing Array. This is the closest analogue I found to the draft's dwell.
- [Faigl & Váňa 2017, IJCNN](https://doi.org/10.1109/ijcnn.2017.7966405) (team of aerial vehicles), [Faigl & Hollinger 2018, IEEE TNNLS](https://doi.org/10.1109/tnnls.2017.2678482), [Pěnička et al. 2017, reactive Dubins TSP](https://doi.org/10.1109/ecmr.2017.8098704) [META].

**Underlying Dubins TSP work** (likely covered by another researcher): [Savla, Frazzoli & Bullo 2005](https://doi.org/10.1109/acc.2005.1470055), [Isaacs & Hespanha 2013](https://doi.org/10.3390/a6010084), [Rathinam, Sengupta & Darbha 2007](https://doi.org/10.1109/tase.2006.872110) (multiple vehicles) [META].

**Points in this literature that bear directly on the draft**
- "[M]any existing approaches set the sensor nodes' locations as target positions for the robots, e.g., see [48], which leads to the collisions with the sensor nodes" — [Huang & Savkin chapter](https://arxiv.org/pdf/1712.02436) [FT]. The draft's sinks physically visit sensors.
- The Dubins car "can only move with constant velocity and bounded angular velocity" — [Szénási & Harmati 2024](https://doi.org/10.21014/actaimeko.v13i4.1572) [FT]. Data is then read while moving: with 0.5 MB, 250 kB/s and 4 m/s, the robot needs an 8 m arc on the visiting circle, about a third of its 25.12 m circumference — [Olasz-Szabó & Harmati 2022](https://distantreader.org/stacks/journals/actaimeko/actaimeko-1254.pdf) [FT].

### Inferences
- The draft's stop of 1-5 s departs from this model, since these papers collect while moving at constant speed. A reviewer could ask why a vehicle that can stop needs a minimum turning radius at all. The draft should justify this, for example as a car-like ground vehicle.
- The draft's dwell equals data volume divided by rate. At 250 kB/s, 1-5 s is 0.25-1.25 MB. Stating it this way would align the draft with the data models in this line at no cost.

### Gaps
- Fu et al. 2026 is unread, and it is the most important remaining check on this question.
- The Crossref title scan misses papers without "Dubins" in the title. Huang & Savkin itself would not have been caught by it, so more such papers probably exist.

## Which works address collision avoidance between sinks or UAVs, and is it usually ignored?

### Takeaway
In ground-sink WSN work physical collision between sinks appears to be ignored, while UAV communications papers usually impose a minimum separation constraint. I found one close data-collection paper that repairs conflicts after planning by waiting, and no game-theoretic resolver of physical conflicts between sinks.

### Cited Findings
- **Post-hoc repair.** [Roberge & Tarbouchi 2021](https://pmc.ncbi.nlm.nih.gov/articles/PMC8541541/) [FT*] samples UAV positions at identical time steps and flags a collision when a pair is closer than a threshold and neither is at the start point. It adds waiting time at the previous download point, repeating until no collisions remain. This is very close to the draft's "wait your turn" resolver.
- **Hard constraint in optimisation.** ||q_m(t) - q_j(t)|| >= d_min for all t and all j != m — [Wu, Zeng & Zhang 2018, IEEE TWC 17](https://doi.org/10.1109/twc.2017.2789293), [arXiv](https://arxiv.org/pdf/1705.02723) [FT, constraint only]. This paper concerns downlink service, not data collection. Only a maximum speed appears in the part I read, and my word search found no turning limit.
- **In AoI data collection.** [Wang et al. 2023, IEEE TCOM 71](https://doi.org/10.1109/tcomm.2023.3255240) [ABS] lists kinematic, energy, trajectory and collision avoidance constraints. An [arXiv excerpt](https://arxiv.org/pdf/2303.01381) [SNIP] shows that the distance between any two UAVs at a time slot cannot be below a safe distance.
- **Learned.** [Wang, Gursoy, Erpek & Sagduyu 2022, IEEE IoT J. 9](https://doi.org/10.1109/jiot.2022.3153585) [SNIP]: UAVs do not communicate and must sense neighbours, and it notes that in earlier work "the collision avoidance and communication constraints were not taken into account together". Also [Wang et al. 2021, GLOBECOM](https://doi.org/10.1109/globecom46510.2021.9686015) and [Fang et al. 2025, ICCC](https://doi.org/10.1109/iccc68654.2025.11437601) [META].
- **Absent in the Dubins team papers I read.** Word searches of [Szénási & Harmati 2024](https://doi.org/10.21014/actaimeko.v13i4.1572), the [Huang & Savkin chapter](https://arxiv.org/pdf/1712.02436) and [Xu et al. 2022](https://www.cs.cityu.edu.hk/~weliang/papers/XXZLXLJD22.pdf) found only collisions with obstacles and nodes. Szénási & Harmati note that multiple base stations "can lead to non-intersecting sub-paths", but as a route-length benefit.
- **Title scan.** In Crossref's top 300 title matches, none had both "sink" and "collision", or both "mule" and "collision".
- **Terminology trap.** "Collision-free" in this field often means radio scheduling. In [Ma et al. 2020, Sensors 20](https://doi.org/10.3390/s20113034) [ABS] it means collision-free communication between nodes and UAVs.
- **Game theory.** [Papatheodorou et al., arXiv 2411.14619](https://arxiv.org/abs/2411.14619) [ABS*] uses game-theoretic learning to assign retrieval tasks to robots and reduce energy, reported as published at CoDIT 2018. It uses games for allocation, not for conflicts between vehicles.

### Inferences
- Explicit collision handling between sinks, in a Dubins team setting with obstacles, appears to be a real gap in the WSN line. It is probably the draft's strongest differentiator.
- Reviewers from the UAV side will compare it with hard constraints. The draft reports remaining conflicts on some maps (open item 2 in the repository's `CLAUDE.md`: closest approach 0.28 m against `d_safe` = 3 m on seed 5). Works like Roberge's report iterating until none remain, though without a turning limit or the same density, so the draft should expect that question.

### Gaps
- I found no survey that measures how often collisions between sinks are ignored, so "usually ignored" rests on the papers I read and the title scan.
- My one query on Dubins, game theory and conflict resolution ran against OpenAlex and failed. Another researcher's coverage of multi-vehicle conflict resolution should take precedence on this point.

## Which metrics are standard, and would omitting radio range and energy look incomplete?

### Takeaway
Standard metrics depend on the venue: network lifetime, energy, latency and delivery ratio in WSN journals, AoI and energy in UAV communications, and path length, time or fleet size in routing and robotics. Among WSN-framed papers I read, none omits both radio or data modelling and any energy evaluation, so the draft's omissions would stand out there, especially given its energy-hole motivation.

### Cited Findings
| Paper | What is evaluated | Radio or data modelling | Read |
|---|---|---|---|
| [Huang & Savkin 2017](https://arxiv.org/pdf/1712.02436) | Path length, collection time, computation time, **energy against multihop** (about 95 % saved at sensors) | Contact time; 250 KB/s to 20 m, 19.2 KB/s to 50 m | FT |
| [Olasz-Szabó & Harmati 2022](https://distantreader.org/stacks/journals/actaimeko/actaimeko-1254.pdf) | Path length, graph vertices | 0.5 MB, 250 kB/s | FT |
| [Szénási & Harmati 2024](https://doi.org/10.21014/actaimeko.v13i4.1572) | Collection time, total sub-path length, normalised AVEDEV | 0.5 MB, 250 kB/s | FT |
| [Xu et al. 2022](https://www.cs.cityu.edu.hk/~weliang/papers/XXZLXLJD22.pdf) | **Number of UAVs** | 5-10 MB, 1 Mb/s; 500 m range in one variant | FT |
| [Roberge & Tarbouchi 2021](https://pmc.ncbi.nlm.nih.gov/articles/PMC8541541/) | Per-UAV distance, time, speed, energy; runtime | SNR range model | FT* |
| [Pěnička et al. 2019](https://comrob.fel.cvut.cz/papers/ral19pop.pdf) | Collected reward, computation time | None in the parts I read; reward only | FT |
| [Rahman et al. 2022](https://doi.org/10.3390/app122211509) | Total distance | Not stated in the abstract | ABS |
| [Najjar-Ghabel et al. 2020](https://doi.org/10.1016/j.adhoc.2020.102243) | Energy against delay | Cluster-head collection | ABS |
| [Chang et al. 2020](https://doi.org/10.1186/s13638-020-01873-4) | Moving distance, sink lifetime | Communication range | ABS |
| [Wang et al. 2023](https://doi.org/10.1109/tcomm.2023.3255240) | Average AoI | Transmission scheduling, UAV energy | ABS |
| [Zhan & Zeng 2020, IEEE TCOM 68](https://doi.org/10.1109/tcomm.2019.2962479) | UAV cost against sensor energy | Wake-up time, transmit power | ABS |
| [Khan et al. 2014](https://doi.org/10.3390/s140202510) (survey) | Delivery latency, packet delivery ratio, energy | n/a | ABS |

- Related, by title: [Zhan, Zeng & Zhang 2018](https://doi.org/10.1109/lwc.2017.2776922) (energy), [Gong et al. 2018, JSAC](https://doi.org/10.1109/jsac.2018.2864420) (flight time), [Abd-Elmagid & Dhillon 2019](https://doi.org/10.1109/tvt.2018.2885871) (peak AoI) [META].
- **Radio range changes the fleet size directly.** "The number of deployed UAVs may be significantly reduced" by exploiting range. Fig. 1 of the paper shows two UAVs falling to one, and the numbers with neighbourhoods are "much smaller" — [Xu et al. 2022](https://www.cs.cityu.edu.hk/~weliang/papers/XXZLXLJD22.pdf) [FT]. [Deng et al. 2020](https://doi.org/10.1109/tnet.2020.2999630) [ABS] similarly departs from "existing studies [that] assumed that one vehicle needs to move to the location of a POI".
- **Direct visits are still publishable.** Xu et al. treat the fly-to-each-device case as a first-class variant, in a top networking journal, with fleet size as the main metric and no sensor energy that I saw — [Xu et al. 2022](https://www.cs.cityu.edu.hk/~weliang/papers/XXZLXLJD22.pdf).
- **Assumptions criticised in the Dubins WSN line.** Obstacle-free fields, ignored dynamic constraints, and negligible data transfer time, the last being "reasonable for the light data load nodes" but not for heavy loads — [Huang & Savkin chapter](https://arxiv.org/pdf/1712.02436) [FT].

### Inferences
- **Yes, expect it to look incomplete at a WSN or communications venue.** The issue is mainly the mismatch: the draft motivates with the energy hole but measures no energy, and an energy hole cannot be shown without a radio model. [Khan et al. 2014](https://doi.org/10.3390/s140202510) confirm that energy holes are the standard motivation, so reviewers will expect a measure.
- **It would be less of a problem at a robotics or routing venue**, where path length, time and fleet size are accepted, as the Pěnička and Xu papers show. Choosing the venue, or changing the framing, is a real option.
- **Radio range is the most serious omission at this map scale.** My arithmetic: the draft has 100 sensors per 10,000 m², or 0.01 per m². Using Huang and Savkin's ranges and ignoring edges, a sink has about 12.6 sensors within 20 m and about 78.5 within 50 m. So with any radio, travel would shrink sharply and dwell would become the main limit. The repository's measured fleet is about 15 sinks per map (155 over seeds 1-10), against the floor of about 4 worked out in the next point. A reviewer may reasonably ask whether so many sinks are needed.
- **Dwell alone sets a floor.** If dwell is uniform on 1-5 s the mean is 3 s, so 100 sensors need about 300 s of stopping when reads happen one at a time. With 80 s budgets that requires at least 4 sinks before any travel.
- A low-cost way to reduce the exposure: state dwell as data volume over rate, report sensor energy under one-hop upload against a multihop baseline as Huang and Savkin do, and add a sensitivity run with a nonzero collection radius, or explicitly restrict the setting to low-power, very short-range links such as contact or near-field reads.

### Gaps
- I found no study that measures what reviewers accept. These judgements are inferred from what published papers report.

## Which simulators, map sizes, sensor counts and baselines are standard?

### Takeaway
There is no single standard. Close path-planning papers use custom geometric simulations on maps from 200 m to several kilometres, and the draft's sensor density is unusually high against all of them.

### Cited Findings
| Paper | Map | Sensors | Obstacles | Vehicles | Speed | Repeats | Baselines |
|---|---|---|---|---|---|---|---|
| **Draft** | 100 m x 100 m | about 100 | 10-15 | Minimised | 5 m/s | Seeds 1-10 | Own variants |
| [Huang & Savkin](https://arxiv.org/pdf/1712.02436) | 200 m x 200 m | 10-50 | Disjoint set | k = 1, 2, 3 | 1-6 m/s | 20 per setting | Viable k-SPLITOUR; Shortest Path Routing |
| [Olasz-Szabó & Harmati 2022](https://distantreader.org/stacks/journals/actaimeko/actaimeko-1254.pdf) | 200 m x 200 m | 39 plus base | 15 | 1 in the tests | 4 m/s | 10 fields | Original SVPP assumptions |
| [Szénási & Harmati 2024](https://doi.org/10.21014/actaimeko.v13i4.1572) | 200 m x 200 m | 40 | 15 | k = 2-8 | 4 m/s | 10 fields | Own variants |
| [Xu et al. 2022](https://www.cs.cityu.edu.hk/~weliang/papers/XXZLXLJD22.pdf) | 5 km x 5 km x 1 km | 100-500 | None found | Minimised | 6-14 m/s | 100 topologies | BTCAlg, MCCPAlg1/2, minMax |
| [Roberge & Tarbouchi](https://pmc.ncbi.nlm.nih.gov/articles/PMC8541541/) | 1700 m x 1000 m | 70; 100 | 3 zones | 3 | 12.5 m/s optimal | | Shortest-path component only |
| [Chang et al. 2020](https://doi.org/10.1186/s13638-020-01873-4) | 5 km x 5 km [SNIP] | | Yes | 1 | | | |
| [Cai & Zhang 2018](https://doi.org/10.3390/s18072105) | 10 x 10 x 10 cube | 3-19 | None | 1 | | 100 Monte Carlo | Linear-interpolation 3D Dubins |

- Budgets: [Xu et al.](https://www.cs.cityu.edu.hk/~weliang/papers/XXZLXLJD22.pdf) use 20-60 minutes, against the draft's 80 s.
- Network simulators do appear in protocol-oriented work. One sink mobility comparison used NS-2, measuring throughput, delivery ratio and end-to-end delay — [i-JIM study](https://online-journals.org/index.php/i-jim/article/view/42121) [SNIP]. A [survey of WSN simulators](https://typeset.io/pdf/simulators-for-wireless-sensor-networks-a-review-2dtydmts3p.pdf) covers NS-2, OMNeT++ and J-Sim [SNIP].
- A sampling-based decoupled baseline is used for Dubins variants — [Deckerová & Faigl 2024](https://deckejin.github.io/publications/deckerova24wsom.html) [ABS*].

### Inferences
- My arithmetic on density: the draft has 0.01 sensors per m², against 0.00125 for Huang and Savkin at n = 50, 0.001 for Szénási and Harmati, and 0.000012 for Xu et al. at n = 300. That is 8-10 times the Dubins papers and three orders of magnitude above Xu et al. This strengthens the radio range question above.
- A 10-seed sample is at the low end of 10 to 100 instances; the Budapest papers also use 10 fields. More seeds and confidence intervals would help.
- The draft compares only its own variants. The expected baselines would be: viable k-SPLITOUR, a minimum cycle cover algorithm from the Xu et al. list using Dubins or geodesic distances, a Euclidean TSP split with Dubins applied afterwards, and an ACO team planner like Szénási and Harmati.
- A network simulator seems unnecessary for a paper about geometric planning, as the close work I read reports custom simulation. It becomes relevant only if delivery ratio or latency under real radios is claimed.

### Gaps
- I found no source ranking simulators by use in mobile sink work, and nothing on MATLAB's share. In the parts I read, the closest papers name no network simulator, but I did not read every simulation section.

## What was searched for and not found, and what limits the search?

### Takeaway
The main limit is access: several key papers are paywalled, and a shared search limit ended the search before the last follow-ups. The novelty judgement should be treated as provisional until Fu et al. 2026 and Kumar & Mukherjee 2023 and 2026 are read.

### Cited Findings
**Searched and not found**
- Any paper with "Dubins" and "mule" in the title: 0 of Crossref's top 300 title matches.
- "sink" with "collision", or "mule" with "collision", or "mule" with "obstacle" in the title: 0 of the top 300 for each query.
- "Dubins" and "sink" in the title: only [Cai & Zhang 2017](https://doi.org/10.1049/el.2016.3836), which is underwater.
- A single paper combining fleet minimisation, Dubins paths, obstacles and conflicts between sinks: none found.
- A game-theoretic resolver of physical conflicts between mobile sinks: none found.
- A survey quantifying how often kinematics or sink collisions are ignored: none found.
- Full text or abstract for Kumar & Mukherjee 2023 and 2026, Fu et al. 2026, Temene et al. 2022.

**Papers remembered but not confirmed: do not cite without checking**
- [UNVERIFIED] Kim et al., "Minimum latency multiple data MULE trajectory planning in wireless sensor networks", IEEE TMC, about 2014. Crossref returned no match.
- [UNVERIFIED] Venue and year of the Sugihara and Gupta paper read above (I recall INFOCOM 2009).
- [UNVERIFIED] That the ResearchGate paper on cluster-based routing with obstacles is IEEE Access 2016 by Tang and others. The DOI I guessed for it resolved to an unrelated paper.

**Limits**
- **Search budget.** The shared web search limit for this session was reached during the last round, so follow-up searches on Fu et al. 2026, Kumar & Mukherjee 2026, Srinivas et al. 2020 and Boyineni et al. 2023 did not run. Sending a follow-up message, or raising `CLAUDE_CODE_MAX_WEB_SEARCHES_PER_SESSION`, would allow them.
- **Title scan only.** Systematic scans used Crossref's top 300 title matches, filtered on title words. Papers with these terms only in the abstract are missed; Huang & Savkin is one such paper.
- **Services unavailable.** OpenAlex's shared daily budget was used up; it offered a key tied to an email address, which I did not request. DBLP put up a bot check, so all 20 planned title queries failed, and I did not try to get past it. Semantic Scholar's batch endpoint was rate limited, though single lookups worked.
- **Publishers.** ScienceDirect and MDPI returned 403, Springer redirected to login, and IEEE Xplore returned empty pages. I did not use proxies. Google Scholar was not accessed.
- **Summaries.** WebSearch and WebFetch return model-written summaries, so [SNIP], [ABS*] and [FT*] items could contain errors of paraphrase. Direct quotations marked [FT] come from text I extracted myself.

### Inferences
- The absence of title hits on sinks and collisions is suggestive, not conclusive, given the title-only limit and the term "collision" being used for radio in this field.

### Gaps
- Papers to obtain through a library before submission, in order: Fu et al. 2026; Kumar & Mukherjee 2026; Kumar & Mukherjee 2023; Temene et al. 2022 (to check the draft's citation is apt); Wichmann & Korkmaz 2015; Srinivas et al. 2020.
