# Algorithm and collision model

## Search contracts

`a_star(grid, h, start, goal, return_stats=False)` keeps its two-value interface. `iterative_astar` adds the same optional stats flag. Their historical empty-path failure convention is `([], 0.0)`; always test path presence rather than interpreting zero cost as success. `a_star_through_points` preserves its `([], inf)` failure convention. The `explored` stats key aliases the new, consistently defined `expanded` count.

A* retains a PriorityQueue and best-g relaxation. It checks a popped g against the current best value and skips stale entries. Improving a closed node reopens it. Parent pointers update with every accepted improvement. Positive edge costs prevent a parent cycle. With an admissible h and zero goal estimate, goal extraction returns an optimal graph cost; overestimating heuristics are not covered.

IDA* starts with h(start), evaluates g+h, returns the minimum exceeded bound and restarts with that threshold. Explicit iterator frames preserve depth-first execution and reconstruct the active path. Goal nodes are not counted as expansions. All iteration expansion counts are accumulated, including unsuccessful iterations. Exhausting a contour without a larger f-cost terminates failure.

The dominance state is `(current_node, frozenset(path_nodes))`, with the least g recorded for that exact state in the current iteration. Equal forbidden sets imply equal available simple continuations; a cheaper prefix therefore has at least as much threshold budget. States with different ancestor sets cannot be safely conflated by that argument. Entries are recorded only after the bound check, when entering a state, rather than while generating unvisited siblings. This trades memory and some pruning power for a transparent correctness condition. Cycles are excluded, which preserves completeness for the finite positive-cost grid because an optimal path is simple. The table can have exponentially many contexts; this is not a linear-memory implementation.

The f threshold uses absolute tolerance `1e-10` to avoid spurious extra contours due to floating accumulation. Optimality is interpreted within that numerical tolerance. Domination and A* relaxation do not suppress real improvements using a tolerance. Tests compare costs independently within `1e-9`.

## Heuristic formula

For grid offsets dx and dy, Octile distance is `max(dx, dy) + (sqrt(2) - 1) * min(dx, dy)`. It is the exact obstacle-free distance for this eight-direction model and remains admissible and consistent with obstacles. Euclidean distance is also a lower bound.

## Geometry and safety

Grid nodes represent centers of closed unit squares. A nonzero cell is occupied. `valid_actions` forbids a diagonal if either side cell is occupied; segment contact with a blocked corner is treated as collision. The revised grid inflates both horizontal extents and obstacle top by safety distance before rasterization. Bounds include the inflated boxes and a free surrounding border where possible. Every cell whose closed square intersects an inflated box is marked; equality at flight altitude remains blocked. This is more conservative than truncating box coordinates and changes offsets and sometimes graph topology.

`bresenham` retains a compatibility name but implements integer supercover traversal, not the old rounded sampling routine. Integer boundary-event comparisons detect exact corner crossings and include both side cells. `line_of_sight` rejects any touched occupied or out-of-bounds cell. An independent slab-intersection oracle tests every endpoint pair in a 5 x 5 grid, including reversals, cardinal segments, grazing corners and steep slopes.

`prune_path` validates all original nodes and segments, even one- and two-node paths. It checks adjacent fallback segments too, then greedily takes the farthest visible endpoint. Invalid input raises ValueError instead of silently preserving an unsafe shortcut. No extra safety margin is invented during pruning: safety must already be represented in the grid.

## Checkpoints and cost

For a specified order, the sum of independent shortest grid segments is the minimum graph cost satisfying that order. Segment-wise pruning preserves every checkpoint endpoint; duplicate joins are dropped. A sequence of consecutive repeated checkpoint requests needs one visit. Returning later to the same checkpoint retains a distinct visit in the sequence. Paths may incidentally pass a later checkpoint earlier; its required ordered visit must still occur later in the planned sequence.

The returned cost is the sum of raw graph costs. Optional mission stats include raw node count and pruned geometric distance; neither distance includes speed, time, acceleration or full vehicle dynamics. Optional entry/zigzag/cruise constraints deliberately alter the optimization problem. They are not described as a globally shortest three-point route.

## Controller validation boundary

The original lifecycle and protocol are retained. A readiness flag prevents state callbacks from taking off while planning is underway. Preflight exceptions, including display/SDK failures, disarm before takeoff. Required-point classification includes the final waypoint. Fly-through is limited to nonrequired transit points, with an additional lateral bound. Required points and the final destination are never accepted solely because their target plane was crossed.

Software route membership is checked in sequence before flight. Sampled arrival records are generated only from accepted measured positions and altitude. They prove proximity within a configurable radius, not exact traversal. The final destination also requires low horizontal speed; missed required visits prevent mission-success landing. This is a demonstration controller: tracking deviation, collision recovery, timeout/failsafe behavior and continuous trajectory clearance still require supervised integration work.
