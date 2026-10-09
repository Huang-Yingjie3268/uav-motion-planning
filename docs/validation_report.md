# Testing and benchmark notes

Validated locally on 2026-10-08. This report concerns the refactored portfolio version, not the untouched original course files.

## Algorithm validation

**106 passed, 0 failed, 1 skipped**, in 5.57 seconds. Machine-readable summary: [test_results.json](../assets/test_results.json). The external map was configured for this run.

Coverage includes the requested blocked/out-of-bounds/start-equals-goal cases, unreachable goals, straight and diagonal shortest paths, blocked corners, detours, independent Dijkstra comparisons, reopening/stale queues, Octile admissibility and consistency, checkpoint order/repetition/failure, inflation and conservative visibility, and controller arrival/readiness regressions.

Both solvers run against all 512 obstacle masks of a 3 x 3 grid. Thirty seeded 3 x 4 maps are checked using both heuristics and an independently implemented Dijkstra oracle. These layouts are scenarios inside tests, not individually counted as pytest test cases. Costs and path steps are independently checked; agreement between two production algorithms is not the correctness oracle.

Supercover traversal is compared with independent closed-square slab intersection for all 625 pairs of centers on a 5 x 5 grid, including reversal. Ten seeded larger layouts independently check pruned-segment geometry. IDA* also passes a 1100-cell corridor and a multi-threshold detour, with expansion totals equal to the sum of per-iteration counts.

The external map's direct A* cost is independently checked with Dijkstra. Default and shaped checkpoint missions pass in both directions. Four additional full-preflight tests exercise the actual controller preparation logic on that map using SDK doubles; no measured visit is fabricated by route validation.

The single skipped test is actual SDK import / simulator TCP connectivity, disabled because a functioning live SDK/simulator session was not established. This opt-in test would check connectivity only. Full physical/simulator route traversal, tracking clearance, final arrival and landing remain unverified. A separate run without external assets returned **101 passed, 0 failed, 6 skipped** in 1.74 seconds: four mocked full-preflight cases, one offline map case and the live connectivity check.

## Recorded benchmarks

Synthetic fixture: 9 x 12, start (1, 1), goal (7, 10). Single search-only samples:

| Method | Grid cost | Expanded | Elapsed seconds | Raw nodes | Pruned waypoints |
|---|---:|---:|---:|---:|---:|
| A* Euclidean | 12.656854249 | 45 | 0.002468 | 12 | 3 |
| A* Octile | 12.656854249 | 40 | 0.002252 | 12 | 3 |
| IDA* Octile | 12.656854249 | 221 | 0.018197 | 12 | 4 |

Three-checkpoint synthetic mission: grid cost 25.242640687, pruned geometric distance 24.508270433, six pruned waypoints. Saved images are planning figures, not simulator screenshots.

Available external map: local start (0, 0), goal (155, -15), altitude 5, safety distance 5. Revised rasterization and input hash are recorded in [benchmark_colliders.json](../assets/benchmark_colliders.json).

| Method | Grid cost | Expanded | Elapsed seconds | Raw nodes | Pruned waypoints |
|---|---:|---:|---:|---:|---:|
| A* Euclidean | 161.213203436 | 2508 | 0.213033 | 156 | 2 |
| A* Octile | 161.213203436 | 732 | 0.050139 | 156 | 2 |
| IDA* Octile | 161.213203436 | 2176 | 0.227916 | 156 | 2 |

Raw CSVs are the authoritative full-precision records. Timing samples do not establish a general ranking. Counts measure successor evaluations, excluding the goal and stale entries; IDA* sums all contours. Different tie paths can yield different pruned waypoint counts at the same optimal raw cost.

The private coursework report recorded 2412, 733 and 211 expanded nodes for the respective methods at cost 161.213. Those are historical coursework-reported observations. Only the direct cost is reproduced here; the full historical counts, pruning behavior and reported flight are not. The revised obstacle rasterization includes safety inflation in bounds and closed-cell coverage, A* accounting excludes goal expansion, and IDA* uses a stricter dominance condition. This prevents treating old and new counts as interchangeable.

## Main changes

| Before | After |
|---|---|
| A* no endpoint validation and no closed-node reopening | Validated endpoints, explicit stale g check, reopening and compatible return_stats |
| Recursive IDA*, node-only path dominance | Explicit bounded DFS frames and exact ancestor-context dominance, summed stats |
| Rounded samples used as line visibility | Conservative supercover plus independent geometric tests |
| Inflation clipped to uninflated bounds | Inflated bounds and closed-cell/altitude contact coverage |
| Hard-coded shaped mission by default | JSON configuration, three required points by default, opt-in shaping |
| Required overshoot could accept a point outside its radius | Measured position and altitude required; final goal radius/speed separately checked |
| PLANNING callback could launch before route preparation finishes | Readiness guard, full preflight validation, SDK failure abort |
| Route membership alone reported as verification | Planned order separated from sampled visit records |
| Coursework benchmark blocks in preflight | Independent examples and external-map benchmark command |
| Unused subdivision helper and debug state | Removed; no new planning algorithms beyond a test-only reference oracle |
| Private answer material mixed with portfolio context | Report remains external; concise historical results and provenance review only |

The controller keeps its class, callback registration, states, NED conversion, MAVLink connection and msgpack display protocol. A snapped start-center waypoint is retained for explicit handling of quantization and start checkpoints.

## Environment and unresolved checks

Successful standalone runtime: Python 3.14.6, NumPy 2.4.6, Matplotlib 3.11.0, pytest 9.0.3, Windows. Existing Anaconda packages were used after dependency installation attempts hit local networking/SSL failures. No installation-success claim is made for a newly created virtual environment.

Unrelated pytest plugin loading and its debugger imported Windows asyncio components that failed with WinError 10106. The successful test command disabled external plugin autoload and pytest's debugger plugin. This changes neither tests nor production algorithm code.

Legacy Python 3.6 syntax compilation passed with PYTHONHASHSEED=0. Actual runtime import failed in NumPy random initialization with a Windows provider DLL error; an earlier interpreter launch failed obtaining random bytes. The legacy SDK environment is not newly certified. See [simulator_setup.md](simulator_setup.md).

Original map availability allowed offline verification; missing map data is therefore not the limitation on this machine. The map is intentionally external pending redistribution review. A functioning live simulation session, new flight logs and confirmed upstream/course publication rights remain outstanding.

## Included files and publication scope

The actual deliverable file list is [file_manifest.txt](file_manifest.txt). The complete English README, configuration, algorithms, controller, tests, runnable examples, CSV measurements and inspected figures are included. No blanket LICENSE is assigned pending permission review, and no private report or map is bundled.

The standalone package is prepared for review; public release remains conditional on course publication permission and applicable upstream rights. The owner's responsibility for the individual coursework submission is confirmed; exact extension sources and AI-assistance boundaries are not fully recorded. Recorded software checks do not establish simulator flight: supervised integration, traversal and landing remain unverified.
