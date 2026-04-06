#pragma once

#include "Graph.hpp"
#include <vector>
#include <unordered_set>
#include <queue>

// ============================================================
// TransitiveClosure.hpp — Warshall's algorithm + helpers
//
// • transitive_closure : full O(V^3) reachability matrix
// • can_reach          : single-pair reachability (BFS, O(V+E))
// • reachable_set      : all nodes reachable from src (BFS)
//
// In transaction analysis, transitive closure determines
// whether a failure in service A can eventually reach service B
// via any chain of dependencies.
// ============================================================

namespace txn {

// ----------------------------------------------------------
// transitive_closure — Warshall's / Floyd-Warshall reachability
// reach[i][j] = true iff there is a directed path i → j.
// O(V^3) time, O(V^2) space.
// ----------------------------------------------------------
[[nodiscard]] inline std::vector<std::vector<bool>>
transitive_closure(const Graph& g) {
    const int N = g.num_nodes();
    // Initialise with direct edges
    std::vector<std::vector<bool>> reach(N, std::vector<bool>(N, false));
    for (int i = 0; i < N; ++i) reach[i][i] = true; // reflexive

    for (int u = 0; u < N; ++u)
        for (const auto& e : g.neighbors(u))
            reach[u][e.dst] = true;

    // Warshall's closure: if i can reach k and k can reach j,
    // then i can reach j.
    for (int k = 0; k < N; ++k)
        for (int i = 0; i < N; ++i) {
            if (!reach[i][k]) continue; // Pruning shortcut
            for (int j = 0; j < N; ++j)
                if (reach[k][j]) reach[i][j] = true;
        }
    return reach;
}

// ----------------------------------------------------------
// can_reach — single-pair BFS reachability, O(V + E).
// Faster than materialising the full closure matrix when
// only one query is needed.
// ----------------------------------------------------------
[[nodiscard]] inline bool
can_reach(const Graph& g, int u, int v) {
    g.range_check(u);
    g.range_check(v);
    if (u == v) return true;

    const int N = g.num_nodes();
    std::vector<bool> visited(N, false);
    std::queue<int>   q;
    visited[u] = true;
    q.push(u);

    while (!q.empty()) {
        int cur = q.front(); q.pop();
        for (const auto& e : g.neighbors(cur)) {
            if (e.dst == v) return true;
            if (!visited[e.dst]) {
                visited[e.dst] = true;
                q.push(e.dst);
            }
        }
    }
    return false;
}

// ----------------------------------------------------------
// reachable_set — BFS from src, returns all reachable nodes
// (excluding src itself unless there is a cycle back to it).
// ----------------------------------------------------------
[[nodiscard]] inline std::unordered_set<int>
reachable_set(const Graph& g, int src) {
    g.range_check(src);
    const int N = g.num_nodes();
    std::vector<bool>     visited(N, false);
    std::unordered_set<int> result;

    std::queue<int> q;
    visited[src] = true;
    q.push(src);

    while (!q.empty()) {
        int u = q.front(); q.pop();
        for (const auto& e : g.neighbors(u)) {
            if (!visited[e.dst]) {
                visited[e.dst] = true;
                result.insert(e.dst);
                q.push(e.dst);
            }
        }
    }
    return result;
}

} // namespace txn
