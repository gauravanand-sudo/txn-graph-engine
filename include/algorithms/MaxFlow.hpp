#pragma once

#include "Graph.hpp"
#include <vector>
#include <queue>
#include <algorithm>
#include <limits>
#include <functional>

// ============================================================
// MaxFlow.hpp — Maximum flow algorithms
//
// • Edmonds-Karp : BFS-based augmenting paths, O(VE^2)
// • Dinic's      : level graph + blocking flow,  O(V^2 E)
// • min_cut      : derives min-cut from the max-flow residual
//
// In a transaction system, max-flow models peak throughput
// (req/s) from an API gateway to a backend data store, and
// min-cut identifies the bottleneck service links.
// ============================================================

namespace txn {

// ----------------------------------------------------------
// Internal: residual graph representation
// Each original edge is stored as two entries (forward +
// reverse) so we can push flow back easily.
// ----------------------------------------------------------
struct FlowEdge {
    int    dst;
    double cap;   ///< remaining capacity
    int    rev;   ///< index of reverse edge in adj[dst]
};

struct MaxFlowResult {
    double                              max_flow;
    std::vector<std::vector<FlowEdge>> residual; ///< final residual graph
    std::vector<bool>                  min_cut;  ///< true = source side of cut
};

/// Build a flow network from the graph.  Edge weights are used
/// as capacities (or alternatively the `capacity` field when set).
inline std::vector<std::vector<FlowEdge>>
build_flow_network(const Graph& g, bool use_capacity_field = true) {
    const int N = g.num_nodes();
    std::vector<std::vector<FlowEdge>> adj(N);

    for (int u = 0; u < N; ++u) {
        for (const auto& e : g.neighbors(u)) {
            double cap = use_capacity_field ? e.capacity : e.weight;
            int fwd_idx = static_cast<int>(adj[u].size());
            int rev_idx = static_cast<int>(adj[e.dst].size());
            adj[u].push_back({e.dst, cap, rev_idx});
            adj[e.dst].push_back({u, 0.0, fwd_idx}); // reverse edge, 0 initial cap
        }
    }
    return adj;
}

// ----------------------------------------------------------
// Edmonds-Karp — O(V * E^2)
// BFS finds shortest augmenting path; repeat until none exist.
// ----------------------------------------------------------
[[nodiscard]] inline MaxFlowResult
edmonds_karp(const Graph& g, int src, int sink) {
    g.range_check(src);
    g.range_check(sink);
    const int N = g.num_nodes();

    auto adj = build_flow_network(g);
    double total_flow = 0.0;

    while (true) {
        // BFS to find shortest augmenting path
        std::vector<int> parent(N, -1);
        std::vector<int> parent_edge(N, -1);
        std::queue<int>  q;
        parent[src] = src;
        q.push(src);

        while (!q.empty() && parent[sink] == -1) {
            int u = q.front(); q.pop();
            for (int i = 0; i < static_cast<int>(adj[u].size()); ++i) {
                const auto& fe = adj[u][i];
                if (parent[fe.dst] == -1 && fe.cap > 1e-9) {
                    parent[fe.dst]      = u;
                    parent_edge[fe.dst] = i;
                    q.push(fe.dst);
                }
            }
        }
        if (parent[sink] == -1) break; // No augmenting path

        // Find bottleneck
        double path_flow = std::numeric_limits<double>::infinity();
        for (int v = sink; v != src; ) {
            int u = parent[v];
            path_flow = std::min(path_flow, adj[u][parent_edge[v]].cap);
            v = u;
        }
        // Augment flow
        for (int v = sink; v != src; ) {
            int u  = parent[v];
            int ei = parent_edge[v];
            adj[u][ei].cap            -= path_flow;
            adj[v][adj[u][ei].rev].cap += path_flow;
            v = u;
        }
        total_flow += path_flow;
    }

    // Determine min-cut: BFS on residual from src
    std::vector<bool> reachable(N, false);
    std::queue<int> q2;
    reachable[src] = true;
    q2.push(src);
    while (!q2.empty()) {
        int u = q2.front(); q2.pop();
        for (const auto& fe : adj[u]) {
            if (!reachable[fe.dst] && fe.cap > 1e-9) {
                reachable[fe.dst] = true;
                q2.push(fe.dst);
            }
        }
    }
    return {total_flow, std::move(adj), std::move(reachable)};
}

// ----------------------------------------------------------
// Dinic's algorithm — O(V^2 * E)
// Uses BFS level graph + DFS blocking flow for each phase.
// Significantly faster than Edmonds-Karp on dense graphs.
// ----------------------------------------------------------

/// BFS to build the level graph; returns true if sink is reachable.
inline bool dinic_bfs(const std::vector<std::vector<FlowEdge>>& adj,
                      int src, int sink,
                      std::vector<int>& level) {
    std::fill(level.begin(), level.end(), -1);
    level[src] = 0;
    std::queue<int> q;
    q.push(src);
    while (!q.empty()) {
        int u = q.front(); q.pop();
        for (const auto& fe : adj[u]) {
            if (level[fe.dst] == -1 && fe.cap > 1e-9) {
                level[fe.dst] = level[u] + 1;
                q.push(fe.dst);
            }
        }
    }
    return level[sink] != -1;
}

/// DFS blocking flow on the level graph.
inline double dinic_dfs(std::vector<std::vector<FlowEdge>>& adj,
                        std::vector<int>& level,
                        std::vector<int>& iter,
                        int u, int sink, double pushed) {
    if (u == sink) return pushed;
    for (int& i = iter[u]; i < static_cast<int>(adj[u].size()); ++i) {
        auto& fe = adj[u][i];
        if (level[fe.dst] != level[u] + 1 || fe.cap < 1e-9) continue;
        double d = dinic_dfs(adj, level, iter, fe.dst, sink,
                             std::min(pushed, fe.cap));
        if (d > 1e-9) {
            fe.cap -= d;
            adj[fe.dst][fe.rev].cap += d;
            return d;
        }
    }
    return 0.0;
}

[[nodiscard]] inline MaxFlowResult
dinic(const Graph& g, int src, int sink) {
    g.range_check(src);
    g.range_check(sink);
    const int N = g.num_nodes();

    auto adj = build_flow_network(g);
    std::vector<int> level(N), iter(N);
    double total_flow = 0.0;

    while (dinic_bfs(adj, src, sink, level)) {
        std::fill(iter.begin(), iter.end(), 0);
        while (true) {
            double f = dinic_dfs(adj, level, iter, src, sink,
                                 std::numeric_limits<double>::infinity());
            if (f < 1e-9) break;
            total_flow += f;
        }
    }

    // Min-cut
    std::vector<bool> reachable(N, false);
    std::queue<int> q;
    reachable[src] = true;
    q.push(src);
    while (!q.empty()) {
        int u = q.front(); q.pop();
        for (const auto& fe : adj[u]) {
            if (!reachable[fe.dst] && fe.cap > 1e-9) {
                reachable[fe.dst] = true;
                q.push(fe.dst);
            }
        }
    }
    return {total_flow, std::move(adj), std::move(reachable)};
}

/// Extract the min-cut edges from a MaxFlowResult.
/// Returns pairs (u, v) where u is on source side, v on sink side.
[[nodiscard]] inline std::vector<std::pair<int,int>>
min_cut_edges(const MaxFlowResult& mfr, const Graph& g) {
    std::vector<std::pair<int,int>> cut;
    const int N = g.num_nodes();
    for (int u = 0; u < N; ++u) {
        if (!mfr.min_cut[u]) continue;
        for (const auto& e : g.neighbors(u)) {
            if (!mfr.min_cut[e.dst])
                cut.emplace_back(u, e.dst);
        }
    }
    return cut;
}

} // namespace txn
