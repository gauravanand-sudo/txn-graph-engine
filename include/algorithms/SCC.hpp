#pragma once

#include "Graph.hpp"
#include <vector>
#include <stack>
#include <algorithm>
#include <unordered_map>

// ============================================================
// SCC.hpp — Strongly Connected Components
//
// • Tarjan's algorithm  : single DFS pass, O(V+E)
// • Kosaraju's algorithm: two DFS passes (original + reversed graph)
// • condensation_graph  : DAG of SCCs (meta-graph)
//
// In a distributed transaction graph, SCCs represent circular
// dependencies (e.g., service A calls B calls C calls A).
// These cycles must be detected before topology-based scheduling
// can be applied.
// ============================================================

namespace txn {

// ----------------------------------------------------------
// Tarjan's SCC — O(V + E)
// Uses discovery time, low-link value, and an explicit stack.
// SCCs are output in reverse topological order.
// ----------------------------------------------------------
[[nodiscard]] inline std::vector<std::vector<int>>
tarjan_scc(const Graph& g) {
    const int N = g.num_nodes();
    std::vector<int>  disc(N, -1);   // discovery time
    std::vector<int>  low(N,  -1);   // low-link value
    std::vector<bool> on_stack(N, false);
    std::vector<int>  stk;
    stk.reserve(N);
    int timer = 0;

    std::vector<std::vector<int>> sccs;

    // Iterative Tarjan using explicit call-stack frames
    // Frame: (node, edge_index)
    std::vector<std::pair<int,int>> call_stk;

    auto visit = [&](int start) {
        call_stk.push_back({start, 0});
        disc[start] = low[start] = timer++;
        stk.push_back(start);
        on_stack[start] = true;

        while (!call_stk.empty()) {
            auto& [u, idx] = call_stk.back();
            const auto& nbrs = g.neighbors(u);

            if (idx < static_cast<int>(nbrs.size())) {
                int v = nbrs[idx++].dst;
                if (disc[v] == -1) {
                    // Tree edge: recurse
                    disc[v] = low[v] = timer++;
                    stk.push_back(v);
                    on_stack[v] = true;
                    call_stk.push_back({v, 0});
                } else if (on_stack[v]) {
                    // Back edge: update low
                    low[u] = std::min(low[u], disc[v]);
                }
            } else {
                // Finished u: propagate low to parent and check SCC root
                call_stk.pop_back();
                if (!call_stk.empty()) {
                    int parent = call_stk.back().first;
                    low[parent] = std::min(low[parent], low[u]);
                }
                if (low[u] == disc[u]) {
                    // u is the root of an SCC
                    std::vector<int> scc;
                    while (true) {
                        int w = stk.back(); stk.pop_back();
                        on_stack[w] = false;
                        scc.push_back(w);
                        if (w == u) break;
                    }
                    sccs.push_back(std::move(scc));
                }
            }
        }
    };

    for (int i = 0; i < N; ++i)
        if (disc[i] == -1) visit(i);

    return sccs;
}

// ----------------------------------------------------------
// Kosaraju's SCC — O(V + E)
// Pass 1: DFS on original graph, record finish order.
// Pass 2: DFS on reversed graph in reverse finish order.
// Each tree in Pass 2 is one SCC.
// ----------------------------------------------------------
[[nodiscard]] inline std::vector<std::vector<int>>
kosaraju_scc(const Graph& g) {
    const int N = g.num_nodes();
    std::vector<bool> visited(N, false);
    std::vector<int>  finish_order;
    finish_order.reserve(N);

    // --- Pass 1: iterative post-order DFS on g ---
    for (int s = 0; s < N; ++s) {
        if (visited[s]) continue;
        std::vector<std::pair<int,int>> stk; // (node, edge_idx)
        stk.push_back({s, 0});
        visited[s] = true;
        while (!stk.empty()) {
            auto& [u, idx] = stk.back();
            const auto& nbrs = g.neighbors(u);
            if (idx < static_cast<int>(nbrs.size())) {
                int v = nbrs[idx++].dst;
                if (!visited[v]) {
                    visited[v] = true;
                    stk.push_back({v, 0});
                }
            } else {
                finish_order.push_back(u);
                stk.pop_back();
            }
        }
    }

    // --- Pass 2: DFS on reversed graph in reverse finish order ---
    Graph rev = g.reverse_graph();
    std::fill(visited.begin(), visited.end(), false);
    std::vector<std::vector<int>> sccs;

    for (int i = N - 1; i >= 0; --i) {
        int s = finish_order[i];
        if (visited[s]) continue;
        // BFS/DFS from s on reversed graph → one SCC
        std::vector<int> scc;
        std::vector<int> dfs_stk;
        dfs_stk.push_back(s);
        visited[s] = true;
        while (!dfs_stk.empty()) {
            int u = dfs_stk.back(); dfs_stk.pop_back();
            scc.push_back(u);
            for (const auto& e : rev.neighbors(u)) {
                if (!visited[e.dst]) {
                    visited[e.dst] = true;
                    dfs_stk.push_back(e.dst);
                }
            }
        }
        sccs.push_back(std::move(scc));
    }
    return sccs;
}

// ----------------------------------------------------------
// condensation_graph — O(V + E)
// Builds the DAG of SCCs (each SCC collapsed into one node).
// The resulting graph can be topologically sorted to determine
// the global processing order of component groups.
//
// Returns:
//   • The condensation Graph
//   • comp_id[v] = which SCC node v belongs to
// ----------------------------------------------------------
[[nodiscard]] inline std::pair<Graph, std::vector<int>>
condensation_graph(const Graph& g,
                   const std::vector<std::vector<int>>& sccs)
{
    const int N   = g.num_nodes();
    const int M   = static_cast<int>(sccs.size());

    std::vector<int> comp_id(N, -1);
    for (int i = 0; i < M; ++i)
        for (int v : sccs[i])
            comp_id[v] = i;

    // Build meta-graph; use a set to avoid duplicate edges
    Graph meta(M);
    for (int i = 0; i < M; ++i) {
        // Construct a ServiceNode for the SCC (aggregate name)
        std::string name = "SCC_" + std::to_string(i) + "(";
        for (int idx = 0; idx < static_cast<int>(sccs[i].size()); ++idx) {
            if (idx) name += ",";
            name += g.get_node(sccs[i][idx]).name;
        }
        name += ")";
        ServiceNode mn(i, std::move(name), NodeType::INTERNAL);
        meta.add_node(mn);
    }

    // Add inter-SCC edges (deduplicated)
    // Use a flat hash: src_scc * M + dst_scc
    std::vector<bool> added(M * M, false);
    for (int u = 0; u < N; ++u) {
        for (const auto& e : g.neighbors(u)) {
            int cu = comp_id[u];
            int cv = comp_id[e.dst];
            if (cu != cv && !added[cu * M + cv]) {
                added[cu * M + cv] = true;
                meta.add_edge(cu, cv, e.weight, e.capacity, e.is_fallback);
            }
        }
    }
    return {std::move(meta), std::move(comp_id)};
}

} // namespace txn
