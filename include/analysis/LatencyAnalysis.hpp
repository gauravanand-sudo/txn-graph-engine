#pragma once

#include "Graph.hpp"
#include "algorithms/ShortestPath.hpp"
#include "algorithms/TopologicalSort.hpp"
#include <vector>
#include <algorithm>
#include <random>
#include <cmath>
#include <stdexcept>
#include <numeric>
#include <string>

// ============================================================
// LatencyAnalysis.hpp — End-to-end latency estimation
//
// • compute_path_latency   : sum of edge weights + node latencies
// • critical_path_latency  : Dijkstra-based longest path (negated)
// • p99_latency_estimate   : Monte-Carlo with Gaussian jitter model
//
// The p99 model is based on empirical observation that per-hop
// latency follows a log-normal distribution; we sample N_SAMPLES
// synthetic traces and report the 99th percentile.
// ============================================================

namespace txn {

constexpr int    LATENCY_MC_SAMPLES  = 10'000; ///< Monte-Carlo sample count
constexpr double JITTER_CV           = 0.20;   ///< Coefficient of variation for jitter

// ----------------------------------------------------------
// compute_path_latency — O(path length)
// Sums edge weights (transit latency) + destination node base
// latency for each hop along the given path.
// ----------------------------------------------------------
[[nodiscard]] inline double
compute_path_latency(const Graph& g, const std::vector<int>& path) {
    if (path.empty()) return 0.0;
    double total = g.get_node(path[0]).base_latency_ms; // source node latency

    for (int i = 0; i + 1 < static_cast<int>(path.size()); ++i) {
        int u = path[i], v = path[i + 1];
        g.range_check(u); g.range_check(v);
        // Find edge u → v
        double edge_w = INF;
        for (const auto& e : g.neighbors(u)) {
            if (e.dst == v) {
                edge_w = std::min(edge_w, e.weight);
            }
        }
        if (edge_w == INF)
            throw std::runtime_error("compute_path_latency: edge not found " +
                                     std::to_string(u) + "→" + std::to_string(v));
        total += edge_w + g.get_node(v).base_latency_ms;
    }
    return total;
}

// ----------------------------------------------------------
// critical_path_latency — O((V+E) log V)
// Finds the highest-latency path from src to sink by negating
// edge weights and running Dijkstra (longest path on DAG).
// Returns {path, total_latency}.
//
// For a DAG, we use the DP-based longest path (no negation
// needed — avoids negative cycle issues on non-DAG graphs).
// Falls back to negated-Dijkstra for general graphs.
// ----------------------------------------------------------
[[nodiscard]] inline std::pair<std::vector<int>, double>
critical_path_latency(const Graph& g, int src, int sink) {
    g.range_check(src);
    g.range_check(sink);
    const int N = g.num_nodes();

    // Attempt DAG-based longest path first (more accurate)
    auto order = kahn_toposort(g);
    if (!order.empty()) {
        // DAG longest path via DP
        std::vector<double> dist(N, -INF);
        std::vector<int>    prev(N, -1);
        dist[src] = g.get_node(src).base_latency_ms;

        for (int u : order) {
            if (dist[u] == -INF) continue;
            for (const auto& e : g.neighbors(u)) {
                double nd = dist[u] + e.weight + g.get_node(e.dst).base_latency_ms;
                if (nd > dist[e.dst]) {
                    dist[e.dst] = nd;
                    prev[e.dst] = u;
                }
            }
        }
        if (dist[sink] == -INF) return {{}, 0.0};

        // Reconstruct path
        std::vector<int> path;
        for (int v = sink; v != -1; v = prev[v])
            path.push_back(v);
        std::reverse(path.begin(), path.end());
        return {path, dist[sink]};
    }

    // General graph: use negated Dijkstra (no guarantee for negative cycles,
    // but latencies are always non-negative so this is safe here).
    // Build negated-weight graph on the fly
    Graph neg(N);
    neg = g;
    // Negate latencies for longest-path-as-shortest-path trick
    // We can't modify g; instead run Bellman-Ford on negated weights.
    std::vector<double> dist2(N, INF);
    std::vector<int>    prev2(N, -1);
    dist2[src] = -(g.get_node(src).base_latency_ms);

    auto edges = g.all_edges();
    for (int iter = 0; iter < N - 1; ++iter) {
        for (const auto& e : edges) {
            double neg_w = -(e.weight + g.get_node(e.dst).base_latency_ms);
            if (dist2[e.src] != INF && dist2[e.src] + neg_w < dist2[e.dst]) {
                dist2[e.dst] = dist2[e.src] + neg_w;
                prev2[e.dst] = e.src;
            }
        }
    }
    if (dist2[sink] == INF) return {{}, 0.0};

    std::vector<int> path;
    for (int v = sink; v != -1; v = prev2[v])
        path.push_back(v);
    std::reverse(path.begin(), path.end());
    return {path, -dist2[sink]};
}

// ----------------------------------------------------------
// p99_latency_estimate — Monte-Carlo simulation
//
// For each sample:
//   • For every edge on the critical path, draw latency from
//     LogNormal(mean=weight, sigma=weight*JITTER_CV).
//   • Sum drawn values to get one sample end-to-end latency.
// Report the 99th percentile of the distribution.
// ----------------------------------------------------------
[[nodiscard]] inline double
p99_latency_estimate(const Graph& g, int src, int sink,
                     int seed = 42) {
    auto [path, nominal] = critical_path_latency(g, src, sink);
    if (path.empty()) return 0.0;

    // Build hop latencies (edge_weight + dst_node_latency)
    std::vector<double> hop_means;
    hop_means.push_back(g.get_node(path[0]).base_latency_ms);
    for (int i = 0; i + 1 < static_cast<int>(path.size()); ++i) {
        int u = path[i], v = path[i+1];
        for (const auto& e : g.neighbors(u)) {
            if (e.dst == v) {
                hop_means.push_back(e.weight + g.get_node(v).base_latency_ms);
                break;
            }
        }
    }

    std::mt19937_64 rng(seed);
    std::vector<double> samples;
    samples.reserve(LATENCY_MC_SAMPLES);

    for (int s = 0; s < LATENCY_MC_SAMPLES; ++s) {
        double total = 0.0;
        for (double mu : hop_means) {
            // LogNormal: mu_ln = ln(mu) - 0.5*sigma_ln^2, sigma_ln = sqrt(ln(1 + CV^2))
            double sigma_ln = std::sqrt(std::log(1.0 + JITTER_CV * JITTER_CV));
            double mu_ln    = std::log(mu) - 0.5 * sigma_ln * sigma_ln;
            std::lognormal_distribution<double> dist(mu_ln, sigma_ln);
            total += dist(rng);
        }
        samples.push_back(total);
    }

    std::sort(samples.begin(), samples.end());
    int p99_idx = static_cast<int>(0.99 * LATENCY_MC_SAMPLES);
    return samples[p99_idx];
}

} // namespace txn
