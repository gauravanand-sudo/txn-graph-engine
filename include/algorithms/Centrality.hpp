#pragma once

#include "Graph.hpp"
#include "ShortestPath.hpp"
#include <vector>
#include <unordered_map>
#include <queue>
#include <stack>
#include <cmath>
#include <algorithm>
#include <numeric>

// ============================================================
// Centrality.hpp — Node importance metrics
//
// • degree_centrality    : in/out degree, O(V+E)
// • betweenness_centrality : Brandes' algorithm, O(VE) unweighted
// • pagerank             : power iteration, O(V+E) per step
// • closeness_centrality : BFS-based, O(V(V+E))
//
// These metrics identify which services are most critical to
// the health of the entire transaction workflow.
// ============================================================

namespace txn {

// ----------------------------------------------------------
// Degree Centrality — normalised (in + out) degree
// Normalised by (N-1) so the result is in [0,1].
// ----------------------------------------------------------
[[nodiscard]] inline std::unordered_map<int,double>
degree_centrality(const Graph& g) {
    const int N = g.num_nodes();
    std::vector<int> in_deg(N, 0), out_deg(N, 0);
    for (int u = 0; u < N; ++u) {
        out_deg[u] = static_cast<int>(g.neighbors(u).size());
        for (const auto& e : g.neighbors(u))
            ++in_deg[e.dst];
    }
    std::unordered_map<int,double> result;
    result.reserve(N);
    double norm = (N > 1) ? 2.0 * (N - 1) : 1.0;
    for (int v = 0; v < N; ++v)
        result[v] = (in_deg[v] + out_deg[v]) / norm;
    return result;
}

// ----------------------------------------------------------
// Betweenness Centrality — Brandes' algorithm O(VE + V^2)
//
// For each source s, perform BFS to find shortest paths and
// accumulate pair dependencies.  Normalise by (N-1)(N-2)/2.
// ----------------------------------------------------------
[[nodiscard]] inline std::unordered_map<int,double>
betweenness_centrality(const Graph& g) {
    const int N = g.num_nodes();
    std::vector<double> CB(N, 0.0);

    for (int s = 0; s < N; ++s) {
        std::vector<std::vector<int>> pred(N);  // predecessors on sp
        std::vector<double> sigma(N, 0.0);      // number of SPs
        std::vector<int>    dist(N, -1);
        sigma[s] = 1.0;
        dist[s]  = 0;

        std::queue<int> q;
        std::stack<int> stk;
        q.push(s);

        while (!q.empty()) {
            int v = q.front(); q.pop();
            stk.push(v);
            for (const auto& e : g.neighbors(v)) {
                int w = e.dst;
                if (dist[w] == -1) { // first visit
                    dist[w] = dist[v] + 1;
                    q.push(w);
                }
                if (dist[w] == dist[v] + 1) { // on shortest path
                    sigma[w] += sigma[v];
                    pred[w].push_back(v);
                }
            }
        }

        std::vector<double> delta(N, 0.0);
        while (!stk.empty()) {
            int w = stk.top(); stk.pop();
            for (int v : pred[w])
                delta[v] += (sigma[v] / sigma[w]) * (1.0 + delta[w]);
            if (w != s) CB[w] += delta[w];
        }
    }

    // Normalise: divide by (N-1)(N-2) for directed graphs
    double norm = (N > 2) ? (N - 1.0) * (N - 2.0) : 1.0;
    std::unordered_map<int,double> result;
    result.reserve(N);
    for (int v = 0; v < N; ++v)
        result[v] = CB[v] / norm;
    return result;
}

// ----------------------------------------------------------
// PageRank — power iteration
// Damping factor d (typically 0.85). Convergence after
// `iterations` steps. Models influence propagation.
// ----------------------------------------------------------
[[nodiscard]] inline std::unordered_map<int,double>
pagerank(const Graph& g,
         double damping    = 0.85,
         int    iterations = 100)
{
    const int N = g.num_nodes();
    std::vector<double> rank(N, 1.0 / N);
    std::vector<int>    out_deg(N, 0);
    for (int u = 0; u < N; ++u)
        out_deg[u] = static_cast<int>(g.neighbors(u).size());

    for (int iter = 0; iter < iterations; ++iter) {
        std::vector<double> new_rank(N, (1.0 - damping) / N);
        for (int u = 0; u < N; ++u) {
            if (out_deg[u] == 0) {
                // Dangling node: distribute rank evenly
                double share = damping * rank[u] / N;
                for (int v = 0; v < N; ++v)
                    new_rank[v] += share;
            } else {
                double share = damping * rank[u] / out_deg[u];
                for (const auto& e : g.neighbors(u))
                    new_rank[e.dst] += share;
            }
        }
        // Check convergence (L1 norm)
        double diff = 0.0;
        for (int v = 0; v < N; ++v) diff += std::abs(new_rank[v] - rank[v]);
        rank = std::move(new_rank);
        if (diff < 1e-9) break;
    }

    std::unordered_map<int,double> result;
    result.reserve(N);
    for (int v = 0; v < N; ++v)
        result[v] = rank[v];
    return result;
}

// ----------------------------------------------------------
// Closeness Centrality — inverse of average shortest path length
// Uses BFS (unweighted); a high score means the node can reach
// others quickly → key coordinator service.
// ----------------------------------------------------------
[[nodiscard]] inline std::unordered_map<int,double>
closeness_centrality(const Graph& g) {
    const int N = g.num_nodes();
    std::unordered_map<int,double> result;
    result.reserve(N);

    for (int s = 0; s < N; ++s) {
        std::vector<int> dist(N, -1);
        std::queue<int>  q;
        dist[s] = 0;
        q.push(s);
        long long total_dist = 0;
        int reachable = 0;

        while (!q.empty()) {
            int u = q.front(); q.pop();
            for (const auto& e : g.neighbors(u)) {
                if (dist[e.dst] == -1) {
                    dist[e.dst] = dist[u] + 1;
                    total_dist += dist[e.dst];
                    ++reachable;
                    q.push(e.dst);
                }
            }
        }
        // Normalised closeness (Wasserman-Faust for disconnected graphs)
        if (reachable > 0 && total_dist > 0)
            result[s] = static_cast<double>(reachable) *
                        static_cast<double>(reachable) /
                        (static_cast<double>(N - 1) * total_dist);
        else
            result[s] = 0.0;
    }
    return result;
}

} // namespace txn
