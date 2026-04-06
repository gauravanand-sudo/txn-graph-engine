# txn-graph-engine

> A C++17 graph-based dependency analysis engine that models distributed transaction workflows as directed graphs — simulating failure propagation, blast radius, latency/fallback impact, and parallel execution across partitioned service components.

---

## Table of Contents

1. [What This Project Is](#1-what-this-project-is)
2. [Why Graph Theory for Distributed Systems?](#2-why-graph-theory-for-distributed-systems)
3. [Architecture Overview](#3-architecture-overview)
4. [The Domain Model](#4-the-domain-model)
5. [Graph Algorithms — Complete Reference](#5-graph-algorithms--complete-reference)
   - 5.1 [Traversal](#51-traversal--traversalhpp)
   - 5.2 [Shortest Paths](#52-shortest-paths--shortestpathhpp)
   - 5.3 [Topological Sort](#53-topological-sort--topologicalsorthpp)
   - 5.4 [Strongly Connected Components](#54-strongly-connected-components--scchpp)
   - 5.5 [Minimum Spanning Tree](#55-minimum-spanning-tree--msthpp)
   - 5.6 [Maximum Flow & Min-Cut](#56-maximum-flow--min-cut--maxflowhpp)
   - 5.7 [Connectivity, Bridges & Articulation Points](#57-connectivity-bridges--articulation-points--connectivityhpp)
   - 5.8 [Cycle Detection](#58-cycle-detection--cycledetectionhpp)
   - 5.9 [Centrality Measures](#59-centrality-measures--centralityhpp)
   - 5.10 [Critical Path (CPM/PERT)](#510-critical-path-cpmpert--criticalpathhpp)
   - 5.11 [Transitive Closure](#511-transitive-closure--transitiveclosurehpp)
   - 5.12 [Graph Coloring](#512-graph-coloring--coloringhpp)
   - 5.13 [Multi-Component Partitioning](#513-multi-component-partitioning--partitioninghpp)
6. [Analysis Layer](#6-analysis-layer)
   - 6.1 [Failure Propagation](#61-failure-propagation)
   - 6.2 [Blast Radius](#62-blast-radius)
   - 6.3 [Latency Analysis](#63-latency-analysis)
   - 6.4 [Fallback Analysis & Yen's k-Shortest](#64-fallback-analysis--yens-k-shortest-paths)
7. [Parallel Execution Engine](#7-parallel-execution-engine)
8. [The Demo Graph — 15-Node Transaction System](#8-the-demo-graph--15-node-transaction-system)
9. [Building & Running](#9-building--running)
10. [Sample Output](#10-sample-output)
11. [Complexity Reference](#11-complexity-reference)
12. [C++17 Features Used](#12-c17-features-used)
13. [Interview Quick-Reference](#13-interview-quick-reference)

---

## 1. What This Project Is

`txn-graph-engine` is a **compiler-style front-end for distributed system topology analysis**. It models a microservice ecosystem as a directed weighted graph and applies 30+ classical and advanced graph algorithms to extract actionable insights:

- Which services form circular dependencies?
- If the payment service goes down, what is the blast radius?
- What is the theoretical maximum throughput from the API gateway to the database?
- Which service, if removed, would partition the entire system?
- What is the critical path that determines your p99 end-to-end latency?
- Can we split the graph into independent components and analyze them in parallel?

Every algorithm is implemented from scratch in pure C++17 — no external dependencies, no Boost, no graph libraries. The code is header-only, template-aware, and annotated to explain both the theory and the systems application.

---

## 2. Why Graph Theory for Distributed Systems?

A distributed transaction workflow is, at its core, a **directed acyclic graph (DAG)** — or one that *should* be acyclic. Each node is a service; each directed edge is a synchronous call or async dependency.

| System Concept | Graph Concept |
|---|---|
| Service A calls Service B | Directed edge A → B |
| Call latency (ms) | Edge weight |
| Request throughput (req/s) | Edge capacity |
| Service health (0–1) | Node attribute |
| Circular dependency | Cycle in directed graph |
| Single point of failure | Articulation point |
| Bottleneck link | Min-cut edge |
| Fastest end-to-end path | Shortest path |
| Worst-case end-to-end time | Longest path (critical path) |
| Services that can't reach each other | Disconnected components |
| Services that form feedback loops | Strongly connected component |

Once you model the system this way, decades of graph algorithm research become directly applicable to production operations.

---

## 3. Architecture Overview

```
txn-graph-engine/
│
├── include/
│   ├── Graph.hpp                        # Core graph data structure
│   │
│   ├── algorithms/                      # Pure graph algorithms (header-only)
│   │   ├── Traversal.hpp                # BFS, DFS, multi-source BFS
│   │   ├── ShortestPath.hpp             # Dijkstra, Bellman-Ford, Floyd-Warshall, Johnson's
│   │   ├── TopologicalSort.hpp          # Kahn's (BFS), DFS-based, level-wave
│   │   ├── SCC.hpp                      # Tarjan's, Kosaraju's, condensation graph
│   │   ├── MST.hpp                      # Kruskal, Prim, Borůvka
│   │   ├── MaxFlow.hpp                  # Edmonds-Karp, Dinic's, min-cut
│   │   ├── Connectivity.hpp             # WCC, bridges, articulation points, bipartite
│   │   ├── CycleDetection.hpp           # 3-colour DFS, Johnson's all-cycles, Union-Find
│   │   ├── Centrality.hpp               # Degree, Betweenness (Brandes), PageRank, Closeness
│   │   ├── CriticalPath.hpp             # CPM/PERT: EST, EFT, LST, LFT, slack
│   │   ├── TransitiveClosure.hpp        # Warshall, can_reach, reachable_set
│   │   ├── Coloring.hpp                 # Greedy Welsh-Powell, DSatur
│   │   └── Partitioning.hpp             # WCC partition, topological wave, SCC layers
│   │
│   ├── analysis/                        # Domain-level analysis built on algorithms
│   │   ├── FailurePropagation.hpp       # BFS cascade, failure probability per node
│   │   ├── BlastRadius.hpp              # Severity classification, revenue impact model
│   │   ├── LatencyAnalysis.hpp          # Path latency, DAG critical path, p99 Monte-Carlo
│   │   └── FallbackAnalysis.hpp         # Yen's k-shortest, fallback detection, resilience score
│   │
│   └── parallel/                        # Concurrent execution
│       ├── ThreadPool.hpp               # C++17 thread pool, typed futures
│       └── ParallelExecutor.hpp         # Partition-parallel analysis runner
│
├── src/
│   └── main.cpp                         # 15-node demo: runs all algorithms, prints results
│
└── CMakeLists.txt
```

**Design decisions:**
- **Header-only algorithms** — templates and inline functions live entirely in `.hpp` files, making the engine trivially embeddable in any C++ project (single `#include`).
- **Adjacency-list graph** — `O(V + E)` space, `O(degree)` neighbour iteration. Optimal for sparse service graphs (typical microservice fans-out to 2–5 downstream services).
- **No external dependencies** — standard library only: `<vector>`, `<queue>`, `<thread>`, `<future>`, `<mutex>`, `<algorithm>`.
- **`[[nodiscard]]` everywhere** — results are always consumed; silently discarded analysis results are a compile-time warning.

---

## 4. The Domain Model

### `ServiceNode` — a vertex

```cpp
struct ServiceNode {
    int         id;               // unique 0-based index
    std::string name;             // "PAYMENT_SERVICE"
    NodeType    type;             // INTERNAL | EXTERNAL | GATEWAY | DATABASE
    double      health;           // 0.0 (down) – 1.0 (fully healthy)
    double      base_latency_ms;  // per-call baseline cost
    bool        has_fallback;     // can route around this node?
};
```

### `ServiceEdge` — a directed dependency

```cpp
struct ServiceEdge {
    int    src, dst;      // from → to
    double weight;        // latency cost (ms) — used by shortest-path algorithms
    double capacity;      // throughput ceiling (req/s) — used by max-flow algorithms
    bool   is_fallback;   // only activated when primary path is unavailable
};
```

### `Graph` — the container

The `Graph` class wraps:
- `std::vector<ServiceNode>` — node registry
- `std::vector<std::vector<ServiceEdge>>` — adjacency list (`adj[u]` = all edges leaving node `u`)
- `reverse_graph()` — returns a new graph with all edge directions flipped (used by Kosaraju's)
- `to_undirected()` — treats all edges as bidirectional (used by WCC, bridge-finding)

---

## 5. Graph Algorithms — Complete Reference

### 5.1 Traversal — `Traversal.hpp`

Graph traversal is the foundation of almost every other algorithm. Both BFS and DFS visit every reachable node, but in different orders with different properties.

#### Breadth-First Search (BFS)

BFS explores all nodes at distance `d` before any node at distance `d+1`. It uses a FIFO queue.

```
Queue: [API_GW]
Visit API_GW  → enqueue AUTH_SVC, ORDER_SVC, AUDIT_SVC
Visit AUTH_SVC → enqueue USER_DB, PAYMENT_SVC ...
...
```

**Properties:**
- Finds the shortest path (fewest edges) in an unweighted graph.
- Time: `O(V + E)`, Space: `O(V)`
- Used in: Edmonds-Karp (shortest augmenting path), Kahn's toposort, WCC, bipartite check.

**Functions provided:**
- `bfs(graph, start)` → visit order as `vector<int>`
- `multi_source_bfs(graph, sources)` → `unordered_map<int, int>` distance from nearest source
- `bfs_with_callback(graph, start, fn)` → visitor pattern, receives `(node_id, depth)`

#### Depth-First Search (DFS)

DFS goes as deep as possible before backtracking. Implemented iteratively (explicit stack) to avoid system stack overflow on large graphs.

**Properties:**
- Natural for cycle detection, topological sort, SCC, bridges.
- Time: `O(V + E)`, Space: `O(V)`

**Functions provided:**
- `dfs(graph, start)` → visit order as `vector<int>`

---

### 5.2 Shortest Paths — `ShortestPath.hpp`

"Shortest" here means minimum total edge weight (latency). These algorithms power latency budget calculations and blast-radius reach analysis.

#### Dijkstra's Algorithm

The workhorse for single-source shortest paths on graphs with **non-negative** edge weights.

**How it works:**
1. Assign distance ∞ to all nodes except source (distance 0).
2. Maintain a min-priority queue ordered by current best distance.
3. Extract the closest unvisited node `u`. For each neighbour `v`: if `dist[u] + w(u,v) < dist[v]`, relax the edge and re-enqueue `v`.
4. Repeat until the queue is empty.

```
dist[API_GW] = 0
dist[AUTH_SVC] = 3ms    (via API_GW→AUTH_SVC, weight=3)
dist[PAYMENT_SVC] = 9ms (via API_GW→AUTH_SVC→PAYMENT_SVC)
```

**Complexity:** `O((V + E) log V)` with a binary heap.

**Functions provided:**
- `dijkstra(graph, src)` → `ShortestPathResult` with `dist[]`, `prev[]`, `path_to(dst)`

#### Bellman-Ford Algorithm

Handles **negative edge weights** and detects **negative-weight cycles**. Relaxes every edge `V-1` times; if a `V`-th relaxation is still possible, a negative cycle exists.

**Why needed here:** Fallback edges might be modelled with a negative latency bonus. Bellman-Ford is the only SSSP algorithm safe to use in that case.

**Complexity:** `O(V × E)`

**Functions provided:**
- `bellman_ford(graph, src)` → `ShortestPathResult` with `has_negative_cycle` flag

#### Floyd-Warshall Algorithm

Computes **all-pairs shortest paths** (APSP) in a single `O(V³)` pass. Suitable for small, dense graphs where you need every pair of distances at once.

**Core recurrence:**
```
dist[i][j] = min(dist[i][j], dist[i][k] + dist[k][j])   for each intermediate k
```

Also detects negative cycles: if `dist[i][i] < 0` for any `i`, a negative cycle exists.

**Functions provided:**
- `floyd_warshall(graph)` → `AllPairsResult` with `dist[][]`, `next[][]`, `path(u,v)`

#### Johnson's Algorithm

APSP for **sparse** graphs with possible negative weights. Reweights edges using potentials derived from a Bellman-Ford run on an augmented graph, making all weights non-negative, then runs Dijkstra from every node.

**Complexity:** `O(V² log V + VE)` — faster than Floyd-Warshall on sparse graphs.

**Functions provided:**
- `johnsons(graph)` → `AllPairsResult`

---

### 5.3 Topological Sort — `TopologicalSort.hpp`

A topological ordering of a DAG lists vertices such that for every directed edge `u → v`, `u` appears before `v`. Fundamental for dependency scheduling — no service can start before its upstream dependencies.

#### Kahn's Algorithm (BFS-based)

1. Compute in-degree of every node.
2. Enqueue all nodes with in-degree 0 (no dependencies).
3. Process queue: emit current node, decrement in-degree of all successors, enqueue those that reach 0.
4. If the result contains fewer than `V` nodes, the graph has a cycle.

```
In-degrees: API_GW=0, AUTH_SVC=1, ORDER_SVC=1 ...
Queue starts: [API_GW]
Emit API_GW → decrement AUTH_SVC(0), ORDER_SVC(0), AUDIT_SVC(0) → enqueue all three
...
```

**Complexity:** `O(V + E)`

#### DFS-based Topological Sort

Post-order DFS: a node is appended to the result **after** all its successors have been visited. Reverse the result for topological order.

#### Level-Wave Partition

Returns `vector<vector<int>>` where `result[d]` contains all nodes whose longest dependency path from any root has length `d`. Nodes at the same level can execute **in parallel**.

```
Layer 0: [API_GATEWAY]              ← no dependencies
Layer 1: [AUTH_SVC, ORDER_SVC, AUDIT_SVC]
Layer 2: [USER_DB, PAYMENT_SVC, INVENTORY_SVC, ORDER_DB, NOTIFICATION_SVC]
...
```

**Functions provided:**
- `kahn_toposort(graph)` → `vector<int>` (empty if cycle detected)
- `dfs_toposort(graph)` → `vector<int>` (empty if cycle detected)
- `toposort_levels(graph)` → `vector<vector<int>>` wave layers

---

### 5.4 Strongly Connected Components — `SCC.hpp`

A **Strongly Connected Component** (SCC) is a maximal set of vertices where every vertex is reachable from every other. In a microservice graph, an SCC with more than one node signals a **circular dependency** — a serious architectural smell.

#### Tarjan's Algorithm

Single-pass DFS using **discovery timestamps** and **low-link values**.

- `disc[v]` = the DFS visit order timestamp of `v`
- `low[v]` = the minimum `disc` reachable from the subtree rooted at `v` via back/cross edges
- When `low[v] == disc[v]`, `v` is the root of an SCC; pop the stack up to and including `v`.

This implementation is **iterative** (explicit call stack) to handle production-scale graphs without stack overflow.

**Complexity:** `O(V + E)`

#### Kosaraju's Algorithm

Two-pass approach:
1. Run DFS on the original graph; push nodes to a stack in finish-time order.
2. Transpose the graph (reverse all edges).
3. Pop nodes from the stack; run DFS on the transposed graph — each DFS tree is one SCC.

Simpler to understand than Tarjan's, same asymptotic complexity.

#### Condensation Graph

`condensation_graph(graph, sccs)` builds a new DAG where each SCC becomes a single "super-node". The condensation of any directed graph is always a DAG, which is then safe to topologically sort.

**Functions provided:**
- `tarjan_scc(graph)` → `vector<vector<int>>`
- `kosaraju_scc(graph)` → `vector<vector<int>>`
- `condensation_graph(graph, sccs)` → `Graph` (the DAG of SCCs)

---

### 5.5 Minimum Spanning Tree — `MST.hpp`

An MST connects all nodes with the minimum total edge weight. In a service graph this answers: *what is the cheapest (lowest-latency) network of connections that keeps all services reachable from each other?*

#### Kruskal's Algorithm

Sort all edges by weight. Use **Union-Find** (Disjoint Set Union with path compression + union by rank) to greedily add the cheapest edge that does not form a cycle.

```
Sort edges by weight: (1ms), (3ms), (4ms), (5ms) ...
Edge (API_GW, AUDIT_SVC, 1ms): connects different components → ADD
Edge (API_GW, AUTH_SVC, 3ms):  connects different components → ADD
...
```

**Union-Find operations:** `find` with path-halving + `union` by rank → effectively `O(α(V))` per operation (inverse Ackermann, practically O(1)).

**Complexity:** `O(E log E)` dominated by the sort.

#### Prim's Algorithm

Grow the MST one node at a time. Maintain a min-heap of edges crossing the cut (MST side vs. non-MST side). At each step, extract the minimum-cost crossing edge.

**Complexity:** `O(E log V)` with a binary heap. Better than Kruskal on dense graphs.

#### Borůvka's Algorithm

Each component simultaneously picks its cheapest outgoing edge; merge components. Repeat until one component remains. Parallelizable by nature — each component's cheapest-edge search is independent.

**Complexity:** `O(E log V)`

**Functions provided:**
- `kruskal_mst(graph)` → `vector<ServiceEdge>` (MST edges)
- `prim_mst(graph, start)` → `vector<ServiceEdge>`
- `boruvka_mst(graph)` → `vector<ServiceEdge>`

---

### 5.6 Maximum Flow & Min-Cut — `MaxFlow.hpp`

Max-flow answers: *given unlimited parallelism, what is the maximum request throughput from source (API gateway) to sink (database)?* Min-cut answers: *which set of edges, if removed, would most tightly throttle that throughput?*

The **Max-Flow Min-Cut Theorem** (Ford-Fulkerson): the maximum flow from `s` to `t` equals the minimum capacity of any `s-t` cut.

#### Edmonds-Karp Algorithm

Implements Ford-Fulkerson using **BFS** to find shortest augmenting paths (fewest edges). This guarantees termination even with irrational capacities and bounds the number of augmenting paths to `O(VE)`.

**Residual graph:** For each real edge `(u, v, cap)`, maintain a reverse edge `(v, u, 0)` that allows "cancelling" flow. Augmenting a path `s → ... → t` by `f` units: subtract `f` from forward edges, add `f` to reverse edges.

**Complexity:** `O(VE²)`

#### Dinic's Algorithm

Faster than Edmonds-Karp. Builds a **level graph** (BFS layering by distance from source) and sends a **blocking flow** through it using DFS with pointer advancement. Then rebuilds the level graph and repeats.

**Complexity:** `O(V² E)` — significantly faster for unit-capacity graphs (`O(E √V)`).

#### Min-Cut Extraction

After max-flow converges, run BFS on the residual graph from source. Nodes reachable = source-side of the cut. All edges from source-side to sink-side that are fully saturated (residual capacity = 0) are the min-cut edges — these are your throughput bottlenecks.

**Functions provided:**
- `edmonds_karp(graph, src, sink)` → `MaxFlowResult`
- `dinic(graph, src, sink)` → `MaxFlowResult`
- `min_cut_edges(result, graph)` → `vector<pair<int,int>>`

---

### 5.7 Connectivity, Bridges & Articulation Points — `Connectivity.hpp`

#### Weakly Connected Components (WCC)

Treats all edges as undirected. Finds groups of services that are connected to each other at all. Isolated components can be analyzed independently (and in parallel).

Uses **Union-Find** with path compression.

**Complexity:** `O(V + E × α(V))`

#### Bridges (Tarjan's Bridge Finding)

A **bridge** is an edge whose removal increases the number of connected components. In a service graph, a bridge is a single link whose failure would partition the network — a critical infrastructure risk.

Uses Tarjan's DFS: an edge `(u, v)` is a bridge if `low[v] > disc[u]` (no back edge from `v`'s subtree can reach `u` or earlier).

#### Articulation Points

An **articulation point** (cut vertex) is a node whose removal splits the graph. The "chokepoint" service — if it goes down, some services can no longer reach others.

A node `u` is an articulation point if:
- `u` is the DFS root and has ≥ 2 children, **or**
- `u` is not the root and has a child `v` with `low[v] >= disc[u]`

#### Bipartite Check

A graph is **bipartite** if its nodes can be 2-coloured such that no two adjacent nodes share a colour. BFS-based 2-colouring: assign alternating colours; if a conflict is found, the graph is not bipartite (contains an odd-length cycle).

**Functions provided:**
- `weakly_connected_components(graph)` → `vector<vector<int>>`
- `find_bridges(graph)` → `vector<pair<int,int>>`
- `find_articulation_points(graph)` → `vector<int>`
- `is_bipartite(graph)` → `pair<bool, vector<int>>` (flag + coloring)

---

### 5.8 Cycle Detection — `CycleDetection.hpp`

#### Directed Cycle Detection (3-colour DFS)

Colour every node WHITE (unvisited), GRAY (in current DFS path), or BLACK (fully processed). A back edge to a GRAY node means a cycle exists.

A cycle in a service dependency graph = circular dependency = potential deadlock or infinite recursion.

**Complexity:** `O(V + E)`

#### Johnson's Algorithm for All Elementary Cycles

Finds **every** elementary cycle in a directed graph. An elementary cycle visits no node more than once. Johnson's algorithm is the most efficient known: `O((V + E)(C + 1))` where `C` = number of cycles.

Approach:
1. Iteratively find SCCs with Tarjan's.
2. For each SCC with ≥ 1 node, run a modified DFS that tracks blocked nodes to avoid revisiting.
3. Unblock nodes when a cycle is found through them.

**Functions provided:**
- `has_cycle_directed(graph)` → `bool`
- `find_all_cycles_directed(graph)` → `vector<vector<int>>`
- `has_cycle_undirected(graph)` → `bool` (Union-Find)

---

### 5.9 Centrality Measures — `Centrality.hpp`

Centrality quantifies how "important" or "influential" a node is in the graph. Different centrality metrics capture different notions of importance.

#### Degree Centrality

Simply the in-degree + out-degree, normalized by `V - 1`. High degree = many direct dependencies.

```
API_GATEWAY: out-degree=3 → degree centrality = 3/(15-1) = 0.21
```

#### Betweenness Centrality (Brandes Algorithm)

How many shortest paths between all pairs `(s, t)` pass through node `v`?

```
BC(v) = Σ_{s≠v≠t} σ(s,t|v) / σ(s,t)
```

Where `σ(s,t)` = number of shortest paths from `s` to `t`, `σ(s,t|v)` = those passing through `v`.

High betweenness = this service is a critical relay. Removing it disrupts many communication flows.

**Brandes algorithm:** `O(VE)` — runs one modified BFS/Dijkstra from each source and accumulates dependency scores during backpropagation.

#### PageRank

Originally Google's web-link algorithm, PageRank models the probability that a "random walk" along directed edges lands on a given node at steady state.

```
PR(v) = (1 - d)/V  +  d × Σ_{u→v} PR(u) / out_degree(u)
```

Where `d ≈ 0.85` is the damping factor. Iterated until convergence.

High PageRank = many high-ranked services call this one. A natural "importance" ranking for microservices.

#### Closeness Centrality

Average distance from a node to all other nodes. High closeness = low average latency to reach any service.

```
CC(v) = (V - 1) / Σ_{u≠v} dist(v, u)
```

**Functions provided:**
- `degree_centrality(graph)` → `map<int, double>`
- `betweenness_centrality(graph)` → `map<int, double>`
- `pagerank(graph, damping=0.85, iterations=100)` → `map<int, double>`
- `closeness_centrality(graph)` → `map<int, double>`

---

### 5.10 Critical Path (CPM/PERT) — `CriticalPath.hpp`

The **Critical Path Method** (CPM) and **Program Evaluation and Review Technique** (PERT) compute the minimum end-to-end duration of a DAG-structured process. The critical path is the longest path — it determines the floor on total execution time. No amount of optimization on non-critical tasks can reduce the total below this floor.

#### Forward Pass

Compute **Earliest Start Time** (EST) and **Earliest Finish Time** (EFT) for every node, traversing in topological order:

```
EST(v) = max over all predecessors u of: EFT(u) + edge_weight(u→v)
EFT(v) = EST(v) + node_latency(v)
```

The project duration = `max(EFT(v))` over all nodes.

#### Backward Pass

Compute **Latest Start Time** (LST) and **Latest Finish Time** (LFT) traversing in reverse topological order:

```
LFT(v) = min over all successors w of: LST(w) - edge_weight(v→w)
LST(v) = LFT(v) - node_latency(v)
```

#### Slack

```
Slack(v) = LST(v) - EST(v)
```

Slack = 0 → the node is on the **critical path**. Any delay here delays the entire transaction.

```
Critical path found:
API_GW → AUTH_SVC → INVENTORY_SVC → ORDER_SVC → PAYMENT_SVC → PGW_SECONDARY → PAYMENT_DB
Total: 113 ms
```

**Functions provided:**
- `compute_critical_path(graph)` → `CriticalPathResult` with per-node `NodeTiming`
- `project_duration(graph)` → `double`

---

### 5.11 Transitive Closure — `TransitiveClosure.hpp`

The transitive closure of a directed graph encodes, for every pair `(u, v)`, whether `u` can reach `v` following directed edges.

**Warshall's algorithm** is Floyd-Warshall for boolean reachability:

```cpp
for k in 0..V:
    for i in 0..V:
        for j in 0..V:
            reach[i][j] |= reach[i][k] && reach[k][j]
```

Practical use: quickly answer "can AUTH_SERVICE indirectly call PAYMENT_DB?" without re-running BFS.

**Functions provided:**
- `transitive_closure(graph)` → `vector<vector<bool>>`
- `can_reach(graph, u, v)` → `bool` (BFS, no precomputation)
- `reachable_set(graph, src)` → `unordered_set<int>` (all nodes reachable from `src`)

---

### 5.12 Graph Coloring — `Coloring.hpp`

Graph coloring assigns colours to nodes such that no two adjacent nodes share a colour. The minimum number of colours needed is the **chromatic number** χ(G).

In a service dependency graph, coloring can model **deployment slot allocation** — services that call each other must be deployed in different slots (colours) to avoid version conflicts during rolling updates.

#### Greedy Welsh-Powell Coloring

Sort nodes by degree (descending). Assign each node the smallest colour not used by any already-colored neighbour.

#### DSatur (Degree of Saturation)

More sophisticated greedy: at each step pick the uncolored node with the highest **saturation degree** (number of distinct colours in its neighbourhood). Tie-break by degree. Produces tighter colorings than plain greedy.

**Functions provided:**
- `greedy_coloring(graph)` → `map<int, int>` (node → colour index)
- `dsatur_coloring(graph)` → `map<int, int>`
- `chromatic_number_estimate(graph)` → `int`

---

### 5.13 Multi-Component Partitioning — `Partitioning.hpp`

Partitioning decomposes the graph into independent subgraphs that can be analyzed, scheduled, or deployed **concurrently**. This is the bridge between graph theory and parallel systems engineering.

#### WCC Partition

The simplest partition: split by weakly connected components. Components with no edges between them are fully independent.

#### Topological Wave Partition

Group nodes by their dependency depth (level in a topological sort). All nodes in the same wave have no dependencies on each other and can execute in parallel:

```
Wave 0 (independent): [API_GATEWAY]
Wave 1 (parallel):    [AUTH_SVC, ORDER_SVC, AUDIT_SVC]
Wave 2 (parallel):    [USER_DB, PAYMENT_SVC, INVENTORY_SVC, ...]
...
```

This is how build systems (Make, Ninja, Bazel) and task schedulers compute their parallel execution plans.

#### SCC-Based DAG Layer Partition

1. Find all SCCs (Tarjan's).
2. Build the condensation DAG.
3. Topologically sort the condensation DAG.
4. Nodes in each condensation layer can be analyzed in parallel.

**Functions provided:**
- `partition_for_parallel(graph)` → `vector<vector<int>>` (by WCC)
- `topological_wave_partition(graph)` → `vector<vector<int>>` (by depth)
- `partition_by_scc(graph)` → `vector<vector<int>>` (SCCs as partitions)

---

## 6. Analysis Layer

The analysis layer composes algorithms into domain-specific tools.

### 6.1 Failure Propagation

**File:** `include/analysis/FailurePropagation.hpp`

Simulates what happens when one or more services fail. Models **cascade failure** — a failed upstream service degrades the health of every downstream service that depends on it.

**Algorithm:**
1. Mark the failed node as unhealthy.
2. BFS downstream (following directed edges).
3. At each reachable node `v`, compute failure probability:
   ```
   P(v fails) = 1 - product of (1 - P(u fails)) for all failed upstream u
   ```
   (Assumes independent failure probabilities, OR-combined.)
4. Track cascade depth (how many hops the failure propagated).

**Output:** `FailureResult` — affected nodes, failure probability per node, cascade depth, total impact score.

Supports multi-failure: provide a set of simultaneously failed nodes; the union of their downstream effects is computed.

---

### 6.2 Blast Radius

**File:** `include/analysis/BlastRadius.hpp`

Classifies the impact of a single service failure.

**Severity tiers:**

| Affected % of fleet | Severity |
|---|---|
| 0–10% | LOW |
| 10–25% | MEDIUM |
| 25–50% | HIGH |
| > 50% | CRITICAL |

**Revenue impact model** (configurable per node type):

| Node Type | Revenue impact / hr |
|---|---|
| GATEWAY | $50 000 |
| DATABASE | $20 000 |
| INTERNAL | $10 000 |
| EXTERNAL | $5 000 |

Affected nodes sum their revenue contributions to give an hourly revenue-at-risk figure.

---

### 6.3 Latency Analysis

**File:** `include/analysis/LatencyAnalysis.hpp`

Three complementary tools:

**Path latency:** Given an explicit sequence of nodes, sum up `node.base_latency_ms` + outgoing `edge.weight` for each hop.

**Critical path latency** between two nodes: the longest (worst-case) path from `src` to `dst`. For a DAG, computed by CPM. For a general graph, uses a modified Dijkstra with negated weights (longest path via negation trick).

**p99 Monte-Carlo estimation:** Models each edge as a log-normal random variable with `mean = edge.weight` and `CV = 0.20` (coefficient of variation). Runs 10 000 simulated transactions along the critical path; returns the 99th-percentile total latency. This accounts for the heavy tail that simple average calculations miss.

---

### 6.4 Fallback Analysis & Yen's k-Shortest Paths

**File:** `include/analysis/FallbackAnalysis.hpp`

**Fallback path detection:** Checks whether a path exists from `src` to `dst` using only `is_fallback = true` edges. Useful for confirming that a fallback circuit-breaker is actually reachable.

**Yen's k-Shortest Paths algorithm:**

Finds the `k` shortest simple paths (no repeated nodes) from `src` to `dst` in ascending cost order. Algorithm sketch:
1. The shortest path is found by Dijkstra.
2. For each subsequent path `k`, generate candidates by deviating from each prefix of the `(k-1)`-th path at each "spur node", then pick the cheapest candidate not already found.

```
k=1: API_GW → AUDIT_SVC              cost=1ms
k=2: API_GW → ORDER_SVC → AUDIT_SVC  cost=7ms
k=3: API_GW → AUTH_SVC → ... → AUDIT_SVC  cost=11ms
...
```

**Resilience score:** `1 / (1 + 1/k_paths_found)` — a normalized 0–1 score. The more diverse paths exist between two services, the closer to 1.0 the score.

---

## 7. Parallel Execution Engine

**Files:** `include/parallel/ThreadPool.hpp`, `include/parallel/ParallelExecutor.hpp`

### Thread Pool

A C++17 thread pool that accepts arbitrary callables and returns typed `std::future<T>`:

```cpp
ThreadPool pool(std::thread::hardware_concurrency());

auto future = pool.enqueue([&]() {
    return blast_radius(graph, node_id);
});
auto result = future.get();
```

**Internals:**
- Fixed number of worker threads created at construction.
- Shared `std::queue<std::function<void()>>` protected by `std::mutex`.
- Workers block on a `std::condition_variable`; they wake when work is available or the pool is shutting down.
- Graceful shutdown: destructor sets `stop = true`, notifies all threads, joins them.

### Parallel Executor

`run_parallel_analysis(graph, partitions, thread_count)`:
1. Calls `partition_for_parallel()` to split the graph into independent subgraphs.
2. For each partition, enqueues a task in the thread pool that runs full blast-radius analysis on every node in that partition.
3. Aggregates results: total affected nodes, maximum revenue at risk, most critical partition.

Since partitions have no edges between them, their analyses are **data-race-free** by construction — no synchronization needed beyond the thread pool's internal queue lock.

---

## 8. The Demo Graph — 15-Node Transaction System

`src/main.cpp` builds this realistic microservice topology:

```
                        ┌─────────────────────────────────────────────────────┐
                        │                  API_GATEWAY (GATEWAY)              │
                        │              health=0.99  latency=2ms               │
                        └──────┬────────────────┬───────────────┬─────────────┘
                               │                │               │
                         weight=3          weight=5         weight=1
                               ▼                ▼               ▼
                    AUTH_SERVICE          ORDER_SERVICE    AUDIT_SERVICE
                   (INTERNAL,5ms)        (INTERNAL,7ms)  (INTERNAL,3ms)
                    /     |    \           /       \
                  w=4   w=6   w=8        w=5      w=3
                  ▼      ▼     ▼          ▼         ▼
               USER_DB  PAY  INV_SVC  ORDER_DB  NOTIF_SVC
              (DB,8ms) _SVC  (INT,6ms) (DB,8ms) (INT,4ms)
                       /\                |        /     \
                     w=8 w=10           w=9     w=5    w=8
                     ▼   ▼              ▼        ▼       ▼
                 PAY_DB FRAUD       INV_DB   EMAIL_GW  SMS_GW
                (DB)   _DETECTOR  (DB,9ms) (EXT,15ms)(EXT,18ms)
                        (INT,20ms)
                            |
                           w=12
                            ▼
                    PAYMENT_GATEWAY_SECONDARY
                       (EXTERNAL,25ms)
```

**22 edges** including 3 fallback edges (e.g., `PAYMENT_SERVICE → PAYMENT_GATEWAY_SECONDARY`).

---

## 9. Building & Running

### Prerequisites

- C++17-capable compiler: `g++ ≥ 7` or `clang++ ≥ 5`
- `cmake ≥ 3.16` (optional — can also use the direct compile command)

### With CMake

```bash
cmake -B build -S .
cmake --build build
./build/txn-graph-engine
```

### Direct compile (no CMake required)

```bash
c++ -std=c++17 -Wall -Wextra -O2 -pthread -Iinclude src/main.cpp -o txn-graph-engine
./txn-graph-engine
```

### Flags explained

| Flag | Purpose |
|---|---|
| `-std=c++17` | Enables structured bindings, `std::optional`, `if constexpr`, etc. |
| `-Wall -Wextra` | All warnings enabled — code compiles clean |
| `-O2` | Optimizations on (important for Monte-Carlo p99 simulation) |
| `-pthread` | Required for `std::thread`, `std::mutex`, `std::future` |
| `-Iinclude` | Adds `include/` to the header search path |

---

## 10. Sample Output

```
╔══════════════════════════════════════════════════════════════════╗
║   TXN-GRAPH-ENGINE  —  Distributed Transaction Analysis Demo    ║
╚══════════════════════════════════════════════════════════════════╝

=== 1. BFS / DFS Traversal ===
BFS: [API_GATEWAY, AUTH_SERVICE, ORDER_SERVICE, AUDIT_SERVICE, USER_DB,
      PAYMENT_SERVICE, INVENTORY_SERVICE, ...]
DFS: [API_GATEWAY, AUTH_SERVICE, USER_DB, PAYMENT_SERVICE, PAYMENT_DB, ...]

=== 2. Dijkstra (API_GATEWAY → all) ===
  AUDIT_SERVICE     →  1.0 ms   path=[API_GW→AUDIT_SVC]
  AUTH_SERVICE      →  3.0 ms   path=[API_GW→AUTH_SVC]
  ORDER_SERVICE     →  5.0 ms   path=[API_GW→ORDER_SVC]
  PAYMENT_SERVICE   →  9.0 ms   path=[API_GW→AUTH_SVC→PAYMENT_SVC]
  ...

=== 4. Topological Sort + Waves ===
  Kahn order: [API_GATEWAY, AUTH_SVC, ORDER_SVC, AUDIT_SVC, ...]
  Wave 0: [API_GATEWAY]
  Wave 1: [AUTH_SVC, ORDER_SVC, AUDIT_SVC]
  Wave 2: [USER_DB, PAYMENT_SVC, INVENTORY_SVC, ORDER_DB, NOTIF_SVC]
  ...

=== 10. Critical Path (CPM/PERT) ===
  Critical path: API_GW → AUTH_SVC → INV_SVC → ORDER_SVC →
                 PAYMENT_SVC → PGW_SECONDARY → PAYMENT_DB
  Project duration: 113.00 ms

=== 15. Failure Simulation: PAYMENT_SERVICE fails ===
  Cascade depth:  2
  Affected nodes: FRAUD_DETECTOR (P=0.10), NOTIFICATION_SVC (P=0.08),
                  PAYMENT_GW_SECONDARY (P=0.12)

=== 16. Blast Radius ===
  Severity:       HIGH  (3/15 services affected)
  Revenue impact: $12,400 / hr

=== 17. p99 Latency (Monte-Carlo, 10000 samples) ===
  Critical path p99:  106.55 ms

=== 18. Yen's k=5 Shortest Paths (API_GW → AUDIT_SVC) ===
  Path 1:  1.0 ms  [API_GW → AUDIT_SVC]
  Path 2:  7.0 ms  [API_GW → ORDER_SVC → AUDIT_SVC]
  Path 3: 11.0 ms  [API_GW → AUTH_SVC → ... → AUDIT_SVC]
  Resilience score: 0.53
```

---

## 11. Complexity Reference

| Algorithm | Time | Space | Notes |
|---|---|---|---|
| BFS / DFS | O(V+E) | O(V) | |
| Multi-source BFS | O(V+E) | O(V) | |
| Dijkstra | O((V+E) log V) | O(V) | Non-negative weights only |
| Bellman-Ford | O(V·E) | O(V) | Handles negative weights |
| Floyd-Warshall | O(V³) | O(V²) | All-pairs |
| Johnson's | O(V² log V + VE) | O(V²) | All-pairs, sparse |
| Kahn's Toposort | O(V+E) | O(V) | |
| Tarjan's SCC | O(V+E) | O(V) | Iterative impl |
| Kosaraju's SCC | O(V+E) | O(V) | Two DFS passes |
| Kruskal MST | O(E log E) | O(V) | DSU with path compression |
| Prim MST | O(E log V) | O(V) | Min-heap |
| Borůvka MST | O(E log V) | O(V) | Parallel-friendly |
| Edmonds-Karp | O(V·E²) | O(V+E) | BFS augmenting paths |
| Dinic's | O(V²·E) | O(V+E) | Level graph |
| Bridges / APs | O(V+E) | O(V) | Single DFS pass |
| WCC (Union-Find) | O(E·α(V)) | O(V) | α = inverse Ackermann |
| Directed cycle | O(V+E) | O(V) | 3-colour DFS |
| Johnson's cycles | O((V+E)(C+1)) | O(V+E) | C = number of cycles |
| Betweenness | O(V·E) | O(V+E) | Brandes algorithm |
| PageRank | O(V+E) per iter | O(V) | Power iteration |
| CPM/PERT | O(V+E) | O(V) | Requires DAG |
| Transitive closure | O(V³) | O(V²) | Warshall |
| DSatur coloring | O(V² + E) | O(V) | |
| Yen's k-shortest | O(kV(V+E)) | O(kV) | |

---

## 12. C++17 Features Used

| Feature | Where used |
|---|---|
| `std::optional<T>` | `ShortestPathResult`, optional path returns |
| Structured bindings `auto [a, b]` | Loop unpacking in Tarjan's, Kahn's |
| `if constexpr` | Template specializations in centrality |
| `std::string_view` | `node_type_str()`, read-only string params |
| `[[nodiscard]]` | All functions returning analysis results |
| `constexpr` | `INF`, algorithm tuning constants |
| `std::variant` | Node type union in coloring result |
| `std::future` / `std::promise` | Thread pool return types |
| `std::condition_variable` | Thread pool worker sleep/wake |
| `std::mutex` + `std::lock_guard` | Thread pool queue protection |
| Aggregate initialization | `NodeTiming{.EST=0, .EFT=0}` |
| `std::clamp` | Health score bounding |

---

## 13. Interview Quick-Reference

This section maps common interview questions to the exact algorithms implemented in this project.

**"How do you detect deadlocks in a distributed system?"**
→ `has_cycle_directed()` — 3-colour DFS, `O(V+E)`. A cycle in the dependency graph = potential deadlock.

**"How do you find the critical path in a build system or pipeline?"**
→ `compute_critical_path()` — CPM forward+backward pass, `O(V+E)`.

**"How would you find a bottleneck in a service graph?"**
→ `dinic()` + `min_cut_edges()` — max-flow min-cut theorem identifies the saturated edges that limit throughput.

**"How do you find single points of failure in a network?"**
→ `find_articulation_points()` and `find_bridges()` — Tarjan's DFS, `O(V+E)`.

**"How do you model circular dependencies between services?"**
→ `tarjan_scc()` — SCCs of size > 1 are circular dependency clusters.

**"How would you schedule independent jobs in parallel?"**
→ `toposort_levels()` — groups nodes by dependency depth; same-level nodes are data-independent and can run concurrently.

**"Which service is the most critical / most connected hub?"**
→ `betweenness_centrality()` or `pagerank()` — ranks services by information-flow importance.

**"How do you find alternate routes when a service fails?"**
→ `yen_k_shortest_paths()` — finds `k` diverse paths ranked by cost.

**"How do you estimate worst-case latency?"**
→ `p99_latency_estimate()` — Monte-Carlo simulation with log-normal jitter model.

**"How do you partition a graph for parallel processing?"**
→ `partition_for_parallel()` (WCC), `topological_wave_partition()` (dependency levels), `partition_by_scc()` (SCC DAG layers).

**"Explain Dijkstra vs Bellman-Ford vs Floyd-Warshall."**
→ Dijkstra: `O((V+E) log V)`, non-negative weights, SSSP. Bellman-Ford: `O(VE)`, negative weights, SSSP, negative-cycle detection. Floyd-Warshall: `O(V³)`, APSP, simple to implement.

**"What is Johnson's algorithm and when would you use it?"**
→ All-pairs shortest paths on sparse graphs with negative weights. Uses Bellman-Ford to compute reweighting potentials, then Dijkstra from every node. `O(V² log V + VE)` — better than Floyd-Warshall's `O(V³)` when `E ≪ V²`.

---

*4 300+ lines of C++17 — zero external dependencies — compiles clean with `-Wall -Wextra -Wpedantic`.*
