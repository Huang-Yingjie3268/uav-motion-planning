# Coursework structure and maintenance notes

These notes describe the supplied coursework before the maintenance changes recorded on 2026-10-08. The component table records the earlier implementation and reasons for those changes. The project was the owner's individual coursework submission using starter code and AI assistance; component-level source details are summarized in [background and source notes](contribution_verification.md).

| Component | Coursework purpose | Technical Value | Earlier maintenance decision | Source context | Reason |
|---|---|---|---|---|---|
| create_grid | Inflate map obstacles at fixed altitude | Configuration-space construction | Refactor | High structural match with Udacity starter; exact source version unconfirmed | Bounds exclude inflation; truncation and strict altitude inequality under-cover boundaries. |
| Action / valid_actions | Eight-direction collision-aware movement | Essential graph model | Keep, validate input | Cardinal structure matches starter; diagonals and no-corner-cutting are coursework extensions | Existing diagonal guards are sound for valid cells but invalid current nodes are unchecked. |
| a_star | Priority-queue search with best g-cost | Core algorithm | Refactor minimally | Starter-derived structure; relaxation extension present in coursework | Missing endpoint checks; closed nodes cannot reopen for inconsistent admissible heuristics; stats count goal as expansion. |
| iterative_astar | Threshold-bounded DFS | Principal algorithm demonstration | Refactor | Present in coursework; absent from compared public starter | Node-only best_path_cost ignores path-dependent forbidden ancestors and marks pending siblings early; recursive depth is limited. No universal correctness conclusion follows from one report result. |
| heuristic / heuristic_octile | Euclidean / Octile estimates | Model-specific comparison | Keep | Euclidean matches starter purpose; Octile present in coursework | Octile is exact without obstacles for costs 1 and sqrt(2). |
| bresenham | Rounded interpolation samples | Intended visibility test | Replace implementation, retain name | Present in the supplied coursework | It is not actual Bresenham and misses cells touched by continuous segments, particularly corner contact. |
| prune_path | Greedy waypoint reduction | Useful optimization | Refactor | Present in the supplied coursework | Must check conservative supercover visibility, including adjacent fallback segments and short paths. |
| subdivide_path | Integer waypoint subdivision | No active use | Remove | Present in the supplied coursework | Neither supplied module calls it; rounding also complicates continuous safety. |
| a_star_through_points | Independent segments through ordered points | Constraint handling | Keep, add optional stats | Present in the supplied coursework | Segment-wise pruning preserves endpoints; deduplication works. Cost is raw graph cost, not pruned geometry; repeated points need tests. |
| States / callbacks / transitions | Drone lifecycle | Integration, mostly framework | Keep, fix guards | High structural match with Udacity starter | PLANNING callback can take off before full validation; exceptions do not guarantee preflight abort. |
| Waypoint tracking | Acceptance radii and fly-through | Application-specific control | Refactor | Present in the supplied coursework | Required/route predicates exclude final point; transit projection lacks lateral-distance bound; overshoot can accept required points outside radius. No physical traversal log. |
| plan_path | Map load, coordinate conversion, configured route | Integration | Refactor | Framework starter-derived; mission extensions present in coursework | Hard-coded shaping changes task; expensive duplicated comparisons in preflight; path.index is inadequate for repeated points. |
| send_waypoints | msgpack simulator display | Framework protocol | Keep | Matches Udacity starter | Uses private connection._master, simulator/version compatibility needs integration verification. |
| debug / check_state / comparisons | Coursework output | Little production value | Remove or move to examples | Present in the supplied coursework | Unused check_state, redundant benchmarking, question labels; retain coursework background in docs. |
| Private coursework report | Historical implementation and results | Context only | Exclude from repository | Supplied individual coursework report; kept private | Do not publish report, personal identifiers, grading questions or full answers. |
| colliders.csv | Simulator map | Optional integration input | Read locally, do not redistribute | Local file discovered; redistribution permission unconfirmed | It exists beside the scripts despite not being attached. No synthetic substitute will be called the original map. |

## Source and publication review

Public starter comparison: [planning utilities](https://github.com/udacity/FCND-Motion-Planning/blob/master/planning_utils.py), [controller](https://github.com/udacity/FCND-Motion-Planning/blob/master/motion_planning.py). The framework is acknowledged explicitly. The owner completed the assignment with AI assistance, including integration, debugging and testing. These records do not establish exact sources or independent authorship for every extension. Later maintenance also used AI coding assistance.

No blanket open-source LICENSE is assigned while course publication policy and derivative-code rights remain unresolved. See contribution_verification.md. The private report and map are not copied. Original course files are left intact; the reviewed portfolio version is created in a separate project directory.

## Validation approach

The later tests use independent Dijkstra movement logic and geometric rectangle intersection as reference oracles, exhaustive tiny maps, deterministic randomized maps, long corridors and mocked controller regressions. Executed results and uncompleted simulator checks are recorded in [validation_report.md](validation_report.md).
