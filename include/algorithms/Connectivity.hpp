#pragma once

#include "Graph.hpp"
#include "MST.hpp"   // DSU
#include <vector>
#include <algorithm>
#include <numeric>
#include <optional>
#include <unordered_map>

// ============================================================
// Connectivity.hpp — Connectivity analysis on service graphs
//
// • weakly_connected_components : WCC via Union-Find, O(E α(V))
// • find_bridges                : Tarjan's bridge detection O(V+E)
// • find_articulation_points    : Tarjan's AP detection, O(V+E)
// • is_bipartite                : 2-colouring via BFS, O(V+E)
//
// Bridges and articulation points model single-points-of-failure
// in the transaction service topology.
// ============================================================

namespace txn {

// ----------------------------------------------------------
// Weakly Connected Components — treats directed edges as
// undirected and finds connected components with DSU.
// ----------------------------------------------------------
[[nodiscard]] inline std::vector<std::vector<int>>
weakly_connected_components(const Graph& g) {
    const int N = g.num_nodes();
    DSU dsu(N);
    for (int u = 0; u < N; ++u)
        for (const auto& e : g.neighbors(u))
            dsu.unite(u, e.dst);

    std::unordered_map<int, std::vector<int>> comps;
    for (int v = 0; v < N; ++v)
        comps[dsu.find(v)].push_back(v);

    std::vector<std::vector<int>> result;
    result.reserve(comps.size());
    for (auto& [root, members] : comps)
        result.push_back(std::move(members));
    return result;
}

// ----------------------------------------------------------
// Internal: Tarjan's bridge / AP detection on an undirected
// interpretation of the graph.
// ----------------------------------------------------------
struct BridgeAPResult {
    std::vector<std::pair<int,int>> bridges;              ///< bridge edges
    std::vector<int>                articulation_points;  ///< AP node ids
};

[[nodiscard]] inline BridgeAPResult
find_bridges_and_aps(const Graph& g) {
    // Treat graph as undirected: work on to_undirected()
    Graph ug = g.to_undirected();
    const int N = ug.num_nodes();

    std::vector<int>  disc(N, -1);
    std::vector<int>  low(N, -1);
    std::vector<bool> visited(N, false);
    std::vector<bool> is_ap(N, false);
    std::vector<std::pair<int,int>> bridges;
    int timer = 0;

    // Iterative DFS preserving parent info
    // Stack frame: (node, parent, edge_index, child_count)
    struct Frame {
        int node, parent, edge_idx, children;
    };
    std::vector<Frame> stk;

    for (int start = 0; start < N; ++start) {
        if (disc[start] != -1) continue;
        stk.push_back({start, -1, 0, 0});
        disc[start] = low[start] = timer++;

        while (!stk.empty()) {
            auto& f = stk.back();
            int u   = f.node;
            const auto& nbrs = ug.neighbors(u);

            if (f.edge_idx < static_cast<int>(nbrs.size())) {
                int v = nbrs[f.edge_idx++].dst;
                if (v == f.parent) continue; // skip the edge we came from
                if (disc[v] == -1) {
                    // Tree edge
                    ++f.children;
                    disc[v] = low[v] = timer++;
                    stk.push_back({v, u, 0, 0});
                } else {
                    // Back edge
                    low[u] = std::min(low[u], disc[v]);
                }
            } else {
                // Pop and propagate
                stk.pop_back();
                if (!stk.empty()) {
                    auto& pf = stk.back();
                    int p    = pf.node;
                    low[p]   = std::min(low[p], low[u]);

                    // AP check for non-root parent
                    if (pf.parent != -1 && low[u] >= disc[p])
                        is_ap[p] = true;

                    // Bridge check
                    if (low[u] > disc[p])
                        bridges.emplace_back(p, u);
                }
                // Root AP: root is AP if it has ≥ 2 children
                if (stk.empty() && f.parent == -1 && f.children >= 2)
                    is_ap[u] = true;
                // Note: f.parent == -1 and stk is now empty means u was the
                // root. We already handled children count in the push above.
            }
        }
    }

    // Fix root AP: the root's children counter was stashed in the frame that
    // was already popped. Re-check using the outer-loop counter.
    // (The iterative approach above is correct; root is marked via the final
    //  pop where stk becomes empty and parent==-1.)

    std::vector<int> aps;
    for (int v = 0; v < N; ++v)
        if (is_ap[v]) aps.push_back(v);

    return {std::move(bridges), std::move(aps)};
}

/// Convenience wrapper — returns bridges only.
[[nodiscard]] inline std::vector<std::pair<int,int>>
find_bridges(const Graph& g) {
    return find_bridges_and_aps(g).bridges;
}

/// Convenience wrapper — returns articulation points only.
[[nodiscard]] inline std::vector<int>
find_articulation_points(const Graph& g) {
    return find_bridges_and_aps(g).articulation_points;
}

// ----------------------------------------------------------
// Bipartite check — BFS 2-colouring on undirected graph.
// Returns {is_bipartite, coloring} where coloring[v] ∈ {0,1}.
// Useful for detecting resource-conflict cycles in scheduling.
// ----------------------------------------------------------
[[nodiscard]] inline std::pair<bool, std::vector<int>>
is_bipartite(const Graph& g) {
    Graph ug = g.to_undirected();
    const int N = ug.num_nodes();
    std::vector<int> color(N, -1);
    bool bipartite = true;

    for (int start = 0; start < N && bipartite; ++start) {
        if (color[start] != -1) continue;
        std::queue<int> q;
        color[start] = 0;
        q.push(start);
        while (!q.empty() && bipartite) {
            int u = q.front(); q.pop();
            for (const auto& e : ug.neighbors(u)) {
                int v = e.dst;
                if (color[v] == -1) {
                    color[v] = 1 - color[u];
                    q.push(v);
                } else if (color[v] == color[u]) {
                    bipartite = false;
                }
            }
        }
    }
    return {bipartite, std::move(color)};
}

} // namespace txn
