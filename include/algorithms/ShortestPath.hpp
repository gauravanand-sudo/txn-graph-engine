#pragma once

#include "Graph.hpp"
#include <vector>
#include <queue>
#include <limits>
#include <optional>
#include <stdexcept>
#include <algorithm>
#include <functional>

// ============================================================
// ShortestPath.hpp — Dijkstra, Bellman-Ford, Floyd-Warshall,
//                    Johnson's all-pairs shortest paths.
//
// These algorithms power latency / blast-radius analysis:
//   • Dijkstra  — fast SSSP on non-negative latency graphs
//   • Bellman-Ford — SSSP with negative-cycle detection
//   • Floyd-Warshall — dense APSP, cycle detection
//   • Johnson's — sparse APSP using reweighting
// ============================================================

namespace txn {

constexpr double INF = std::numeric_limits<double>::infinity();

// ----------------------------------------------------------
// ShortestPathResult — output from SSSP algorithms
// ----------------------------------------------------------
struct ShortestPathResult {
    std::vector<double> dist;  ///< dist[v] = shortest distance from src to v
    std::vector<int>    prev;  ///< prev[v] = predecessor on shortest path
    bool has_negative_cycle{false};

    /// Reconstruct path from src to dst using prev[].
    [[nodiscard]] std::vector<int> path_to(int dst) const {
        if (dist[dst] == INF) return {};
        std::vector<int> p;
        for (int v = dst; v != -1; v = prev[v])
            p.push_back(v);
        std::reverse(p.begin(), p.end());
        return p;
    }
};

// ----------------------------------------------------------
// AllPairsResult — output from APSP algorithms
// ----------------------------------------------------------
struct AllPairsResult {
    std::vector<std::vector<double>> dist; ///< dist[u][v]
    std::vector<std::vector<int>>    next; ///< next[u][v] for path reconstruction
    bool has_negative_cycle{false};

    [[nodiscard]] std::vector<int> path(int u, int v) const {
        if (dist[u][v] == INF) return {};
        std::vector<int> p;
        p.push_back(u);
        while (u != v) {
            u = next[u][v];
            p.push_back(u);
        }
        return p;
    }
};

// ----------------------------------------------------------
// Dijkstra — O((V + E) log V)
// Requires non-negative edge weights (latencies ≥ 0).
// ----------------------------------------------------------
[[nodiscard]] inline ShortestPathResult
dijkstra(const Graph& g, int src) {
    g.range_check(src);
    const int N = g.num_nodes();
    ShortestPathResult res;
    res.dist.assign(N, INF);
    res.prev.assign(N, -1);
    res.dist[src] = 0.0;

    // min-heap: (distance, node)
    using P = std::pair<double, int>;
    std::priority_queue<P, std::vector<P>, std::greater<P>> pq;
    pq.push({0.0, src});

    while (!pq.empty()) {
        auto [d, u] = pq.top(); pq.pop();
        // Stale entry — skip
        if (d > res.dist[u]) continue;
        for (const auto& e : g.neighbors(u)) {
            double nd = d + e.weight;
            if (nd < res.dist[e.dst]) {
                res.dist[e.dst] = nd;
                res.prev[e.dst] = u;
                pq.push({nd, e.dst});
            }
        }
    }
    return res;
}

// ----------------------------------------------------------
// Bellman-Ford — O(V * E)
// Handles negative weights; detects negative-weight cycles.
// Used as the reweighting step inside Johnson's algorithm.
// ----------------------------------------------------------
[[nodiscard]] inline ShortestPathResult
bellman_ford(const Graph& g, int src) {
    g.range_check(src);
    const int N = g.num_nodes();
    ShortestPathResult res;
    res.dist.assign(N, INF);
    res.prev.assign(N, -1);
    res.dist[src] = 0.0;

    const auto edges = g.all_edges();

    // Relax all edges N-1 times
    for (int iter = 0; iter < N - 1; ++iter) {
        bool relaxed = false;
        for (const auto& e : edges) {
            if (res.dist[e.src] != INF &&
                res.dist[e.src] + e.weight < res.dist[e.dst]) {
                res.dist[e.dst] = res.dist[e.src] + e.weight;
                res.prev[e.dst] = e.src;
                relaxed = true;
            }
        }
        if (!relaxed) break; // Early termination
    }

    // N-th relaxation detects negative cycles
    for (const auto& e : edges) {
        if (res.dist[e.src] != INF &&
            res.dist[e.src] + e.weight < res.dist[e.dst]) {
            res.has_negative_cycle = true;
            break;
        }
    }
    return res;
}

// ----------------------------------------------------------
// Floyd-Warshall — O(V^3), APSP via dynamic programming.
// Works with negative weights; detects negative cycles via
// negative diagonal entries.
// ----------------------------------------------------------
[[nodiscard]] inline AllPairsResult
floyd_warshall(const Graph& g) {
    const int N = g.num_nodes();
    AllPairsResult res;
    res.dist.assign(N, std::vector<double>(N, INF));
    res.next.assign(N, std::vector<int>(N, -1));

    // Initialise with direct edges
    for (int u = 0; u < N; ++u) {
        res.dist[u][u] = 0.0;
        for (const auto& e : g.neighbors(u)) {
            // Keep smallest weight if multiple edges exist
            if (e.weight < res.dist[u][e.dst]) {
                res.dist[u][e.dst] = e.weight;
                res.next[u][e.dst] = e.dst;
            }
        }
    }
    // Set next for diagonal (self-loops trivially resolved)
    for (int u = 0; u < N; ++u)
        res.next[u][u] = u;

    // Triple nested relaxation
    for (int k = 0; k < N; ++k) {
        for (int i = 0; i < N; ++i) {
            if (res.dist[i][k] == INF) continue; // Pruning
            for (int j = 0; j < N; ++j) {
                if (res.dist[k][j] == INF) continue;
                double nd = res.dist[i][k] + res.dist[k][j];
                if (nd < res.dist[i][j]) {
                    res.dist[i][j] = nd;
                    res.next[i][j] = res.next[i][k];
                }
            }
        }
    }

    // Negative-cycle check: negative self-distance
    for (int i = 0; i < N; ++i) {
        if (res.dist[i][i] < 0.0) {
            res.has_negative_cycle = true;
            break;
        }
    }
    return res;
}

// ----------------------------------------------------------
// Johnson's algorithm — O(V^2 log V + VE), sparse APSP.
//
// Steps:
//   1. Add a virtual source q with 0-weight edges to all nodes.
//   2. Run Bellman-Ford from q → get potentials h[v].
//   3. Reweight every edge: w'(u,v) = w(u,v) + h[u] - h[v] ≥ 0.
//   4. Run Dijkstra from every vertex on the reweighted graph.
//   5. Correct distances back: dist[u][v] - h[u] + h[v].
// ----------------------------------------------------------
[[nodiscard]] inline AllPairsResult
johnsons(const Graph& g) {
    const int N = g.num_nodes();
    AllPairsResult res;

    // --- Step 1: build augmented graph with virtual source (id = N) ---
    // We don't modify g; instead operate on edge list directly.
    // Potentials vector h[v]
    std::vector<double> h(N + 1, INF);
    h[N] = 0.0; // virtual source

    // Bellman-Ford on augmented graph
    const auto orig_edges = g.all_edges();
    // Relax N times (augmented has N+1 vertices)
    for (int iter = 0; iter < N; ++iter) {
        // Virtual source edges: N → every real vertex, weight 0
        for (int v = 0; v < N; ++v) {
            if (0.0 < h[v]) { // h[N]=0, so 0 + 0 = 0
                h[v] = 0.0;
            }
        }
        // Original edges
        for (const auto& e : orig_edges) {
            if (h[e.src] != INF && h[e.src] + e.weight < h[e.dst])
                h[e.dst] = h[e.src] + e.weight;
        }
    }

    // Negative-cycle detection (N+1-th relaxation)
    for (const auto& e : orig_edges) {
        if (h[e.src] != INF && h[e.src] + e.weight < h[e.dst]) {
            res.has_negative_cycle = true;
            return res;
        }
    }

    // --- Steps 4-5: Dijkstra from every source on reweighted graph ---
    res.dist.assign(N, std::vector<double>(N, INF));
    res.next.assign(N, std::vector<int>(N, -1));

    // Reweight on the fly during Dijkstra:
    // w'(u,v) = w(u,v) + h[u] - h[v]  (always ≥ 0 after Bellman-Ford)
    for (int src = 0; src < N; ++src) {
        // Dijkstra with reweighted edges
        std::vector<double> d(N, INF);
        std::vector<int>    nxt(N, -1);
        d[src] = 0.0;

        using P = std::pair<double, int>;
        std::priority_queue<P, std::vector<P>, std::greater<P>> pq;
        pq.push({0.0, src});

        while (!pq.empty()) {
            auto [du, u] = pq.top(); pq.pop();
            if (du > d[u]) continue;
            for (const auto& e : g.neighbors(u)) {
                double rw_w = e.weight + h[e.src] - h[e.dst];
                double nd   = d[u] + rw_w;
                if (nd < d[e.dst]) {
                    d[e.dst] = nd;
                    nxt[e.dst] = u; // predecessor
                    pq.push({nd, e.dst});
                }
            }
        }
        // Correct distances back and store
        for (int v = 0; v < N; ++v) {
            if (d[v] != INF)
                res.dist[src][v] = d[v] - h[src] + h[v];
            else
                res.dist[src][v] = INF;
        }
        // Build next[][] from predecessor map
        // next[src][v] = first hop on shortest path src → v
        // Reconstruct by walking predecessors backward then taking first step
        for (int v = 0; v < N; ++v) {
            if (v == src) { res.next[src][v] = src; continue; }
            if (d[v] == INF) continue;
            // Walk nxt[] to find the hop right after src
            int cur = v;
            while (nxt[cur] != src && nxt[cur] != -1)
                cur = nxt[cur];
            res.next[src][v] = cur;
        }
    }
    return res;
}

} // namespace txn
