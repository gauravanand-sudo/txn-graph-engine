#pragma once

#include "Graph.hpp"
#include "MST.hpp"   // DSU
#include <vector>
#include <unordered_set>
#include <algorithm>

// ============================================================
// CycleDetection.hpp
//
// • has_cycle_directed      : O(V+E), DFS with WHITE/GRAY/BLACK
// • find_all_cycles_directed: Johnson's elementary cycle algorithm
// • has_cycle_undirected    : O(E α(V)), Union-Find
//
// Detecting circular dependencies is safety-critical in
// transaction workflow validation — a cycle means a set of
// services are mutually waiting and will deadlock.
// ============================================================

namespace txn {

// ----------------------------------------------------------
// Directed cycle detection — DFS 3-colour algorithm
// WHITE=0 (unvisited), GRAY=1 (in current path), BLACK=2 (done)
// A back edge (u → GRAY vertex) implies a cycle.
// ----------------------------------------------------------
[[nodiscard]] inline bool
has_cycle_directed(const Graph& g) {
    const int N = g.num_nodes();
    std::vector<int> color(N, 0);

    for (int start = 0; start < N; ++start) {
        if (color[start] != 0) continue;
        // Iterative DFS
        std::vector<std::pair<int,int>> stk; // (node, edge_idx)
        stk.push_back({start, 0});
        color[start] = 1; // GRAY

        while (!stk.empty()) {
            auto& [u, idx] = stk.back();
            const auto& nbrs = g.neighbors(u);
            if (idx < static_cast<int>(nbrs.size())) {
                int v = nbrs[idx++].dst;
                if (color[v] == 1) return true; // back edge → cycle
                if (color[v] == 0) {
                    color[v] = 1;
                    stk.push_back({v, 0});
                }
            } else {
                color[u] = 2; // BLACK
                stk.pop_back();
            }
        }
    }
    return false;
}

// ----------------------------------------------------------
// Johnson's elementary cycle algorithm — finds ALL simple
// (elementary) cycles in a directed graph.
//
// Algorithm sketch:
//   For each starting vertex s (in order):
//     DFS from s in the subgraph induced by vertices ≥ s.
//     Maintain a blocked set and a blocking map.
//     When we reach s again, record the current path as a cycle.
//     Unblock vertices on cycle; use blocking map to propagate.
//   Advance s and repeat on subgraph [s+1 .. N-1].
//
// Complexity: O((V + E)(C + 1)) where C = number of cycles.
// ----------------------------------------------------------
namespace detail {

inline void johnson_unblock(int u,
                             std::vector<bool>& blocked,
                             std::vector<std::unordered_set<int>>& B) {
    blocked[u] = false;
    for (int w : B[u]) {
        if (blocked[w]) johnson_unblock(w, blocked, B);
    }
    B[u].clear();
}

inline bool johnson_dfs(int v, int s, const Graph& g,
                         std::vector<bool>& blocked,
                         std::vector<std::unordered_set<int>>& B,
                         std::vector<int>& path,
                         std::vector<std::vector<int>>& cycles) {
    bool found_cycle = false;
    path.push_back(v);
    blocked[v] = true;

    for (const auto& e : g.neighbors(v)) {
        int w = e.dst;
        if (w < s) continue; // Only consider vertices ≥ s
        if (w == s) {
            // Found a cycle
            cycles.push_back(path);
            found_cycle = true;
        } else if (!blocked[w]) {
            if (johnson_dfs(w, s, g, blocked, B, path, cycles))
                found_cycle = true;
        }
    }

    if (found_cycle) {
        johnson_unblock(v, blocked, B);
    } else {
        for (const auto& e : g.neighbors(v)) {
            int w = e.dst;
            if (w < s) continue;
            B[w].insert(v); // v is blocked by w
        }
    }
    path.pop_back();
    return found_cycle;
}

} // namespace detail

[[nodiscard]] inline std::vector<std::vector<int>>
find_all_cycles_directed(const Graph& g) {
    const int N = g.num_nodes();
    std::vector<bool>                        blocked(N, false);
    std::vector<std::unordered_set<int>>     B(N);
    std::vector<int>                         path;
    std::vector<std::vector<int>>            cycles;

    for (int s = 0; s < N; ++s) {
        // Reset blocked status for nodes ≥ s
        std::fill(blocked.begin(), blocked.end(), false);
        for (int i = 0; i < N; ++i) B[i].clear();
        path.clear();
        detail::johnson_dfs(s, s, g, blocked, B, path, cycles);
    }
    return cycles;
}

// ----------------------------------------------------------
// Undirected cycle detection — Union-Find
// If adding edge (u,v) where both u and v are already in the
// same component, a cycle exists.
// ----------------------------------------------------------
[[nodiscard]] inline bool
has_cycle_undirected(const Graph& g) {
    DSU dsu(g.num_nodes());
    for (int u = 0; u < g.num_nodes(); ++u) {
        for (const auto& e : g.neighbors(u)) {
            // Avoid double-counting by only processing u < dst
            if (u < e.dst) {
                if (!dsu.unite(u, e.dst)) return true;
            }
        }
    }
    return false;
}

} // namespace txn
