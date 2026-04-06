#pragma once

#include "Graph.hpp"
#include <vector>
#include <queue>
#include <stdexcept>
#include <optional>

// ============================================================
// TopologicalSort.hpp — Kahn's (BFS) and DFS-based toposort,
//                       plus level-wise grouping for parallel
//                       transaction scheduling.
//
// Returns an empty vector when a cycle is detected (the graph
// is not a DAG and ordering is undefined).
// ============================================================

namespace txn {

// ----------------------------------------------------------
// Kahn's algorithm — O(V + E)
// Uses in-degree tracking and BFS.  Because we use a queue
// (FIFO), nodes at the same depth are processed together.
// ----------------------------------------------------------
[[nodiscard]] inline std::vector<int>
kahn_toposort(const Graph& g) {
    const int N = g.num_nodes();
    std::vector<int> in_deg(N, 0);

    for (int u = 0; u < N; ++u)
        for (const auto& e : g.neighbors(u))
            ++in_deg[e.dst];

    std::queue<int> q;
    for (int v = 0; v < N; ++v)
        if (in_deg[v] == 0) q.push(v);

    std::vector<int> order;
    order.reserve(N);

    while (!q.empty()) {
        int u = q.front(); q.pop();
        order.push_back(u);
        for (const auto& e : g.neighbors(u)) {
            if (--in_deg[e.dst] == 0)
                q.push(e.dst);
        }
    }
    // If we didn't visit all nodes → cycle exists
    if (static_cast<int>(order.size()) != N) return {};
    return order;
}

// ----------------------------------------------------------
// DFS-based topological sort — O(V + E)
// Post-order DFS; reverse of finish times gives topological
// order. Uses iterative DFS with explicit state to avoid
// stack overflow on deep dependency chains.
// ----------------------------------------------------------
[[nodiscard]] inline std::vector<int>
dfs_toposort(const Graph& g) {
    const int N = g.num_nodes();
    // 0 = WHITE (unvisited), 1 = GRAY (in stack), 2 = BLACK (done)
    std::vector<int> color(N, 0);
    std::vector<int> order;
    order.reserve(N);
    bool has_cycle = false;

    // Iterative DFS with (node, edge_index) pairs
    for (int start = 0; start < N && !has_cycle; ++start) {
        if (color[start] != 0) continue;
        // Stack holds (node, iterator index into neighbors)
        std::vector<std::pair<int,int>> stk;
        stk.push_back({start, 0});
        color[start] = 1; // GRAY

        while (!stk.empty() && !has_cycle) {
            auto& [u, idx] = stk.back();
            const auto& nbrs = g.neighbors(u);
            if (idx < static_cast<int>(nbrs.size())) {
                int v = nbrs[idx++].dst;
                if (color[v] == 1) {
                    // Back edge → cycle
                    has_cycle = true;
                } else if (color[v] == 0) {
                    color[v] = 1;
                    stk.push_back({v, 0});
                }
                // color[v]==2: cross/forward edge, skip
            } else {
                // Finished u — append to post-order
                color[u] = 2;
                order.push_back(u);
                stk.pop_back();
            }
        }
    }
    if (has_cycle) return {};
    // Reverse of post-order = topological order
    std::reverse(order.begin(), order.end());
    return order;
}

// ----------------------------------------------------------
// Level-based topological partitioning — O(V + E)
// Returns nodes grouped by "wave front": all nodes in wave k
// can start executing once all nodes in wave k-1 have finished.
// This models concurrent transaction batch scheduling.
// ----------------------------------------------------------
[[nodiscard]] inline std::vector<std::vector<int>>
toposort_levels(const Graph& g) {
    const int N = g.num_nodes();
    std::vector<int> in_deg(N, 0);

    for (int u = 0; u < N; ++u)
        for (const auto& e : g.neighbors(u))
            ++in_deg[e.dst];

    std::queue<int> q;
    for (int v = 0; v < N; ++v)
        if (in_deg[v] == 0) q.push(v);

    std::vector<std::vector<int>> levels;
    int visited = 0;

    while (!q.empty()) {
        // Drain all nodes at the current frontier in one level
        std::vector<int> level;
        int sz = static_cast<int>(q.size());
        for (int i = 0; i < sz; ++i) {
            int u = q.front(); q.pop();
            level.push_back(u);
            ++visited;
            for (const auto& e : g.neighbors(u))
                if (--in_deg[e.dst] == 0)
                    q.push(e.dst);
        }
        levels.push_back(std::move(level));
    }
    if (visited != N) return {}; // Cycle detected
    return levels;
}

} // namespace txn
