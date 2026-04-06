#pragma once

#include "Graph.hpp"
#include "algorithms/ShortestPath.hpp"
#include <vector>
#include <queue>
#include <algorithm>
#include <unordered_set>
#include <limits>
#include <optional>
#include <cmath>

// ============================================================
// FallbackAnalysis.hpp — Alternate path detection & resilience
//
// • has_fallback_path     : checks if any is_fallback-marked edge
//   lies on a path from src to dst
// • yen_k_shortest_paths  : Yen's algorithm — k-shortest simple
//   paths, O(k * V * (V + E) log V)
// • resilience_score      : 0.0–1.0 based on path diversity
//
// In distributed transactions, fallback paths model redundancy:
// if the primary payment gateway is down, can we route via a
// secondary gateway?  Yen's k-paths enumerates all such options.
// ============================================================

namespace txn {

// Type alias for a (path, cost) pair
using PathCost = std::pair<std::vector<int>, double>;

// ----------------------------------------------------------
// has_fallback_path — O(V + E)
// Returns true if there exists a path src → dst that traverses
// at least one is_fallback-marked edge.
// ----------------------------------------------------------
[[nodiscard]] inline bool
has_fallback_path(const Graph& g, int src, int dst) {
    g.range_check(src);
    g.range_check(dst);
    const int N = g.num_nodes();

    // BFS / DFS restricted to fallback edges only from src,
    // then check if dst is reachable.
    // We build a sub-graph using only fallback edges.
    std::vector<bool> visited(N, false);
    std::queue<int>   q;
    visited[src] = true;
    q.push(src);

    while (!q.empty()) {
        int u = q.front(); q.pop();
        if (u == dst) return true;
        for (const auto& e : g.neighbors(u)) {
            if (e.is_fallback && !visited[e.dst]) {
                visited[e.dst] = true;
                q.push(e.dst);
            }
        }
    }

    // Also allow: fallback-edge reachable from any node on ANY src→dst path
    // Strategy: BFS on the full graph, but mark if we ever traverse a fallback edge
    std::fill(visited.begin(), visited.end(), false);
    std::vector<bool> used_fallback(N, false); // used_fallback[v] = arrived via fallback
    std::queue<std::pair<int,bool>> q2; // (node, used_fallback_so_far)
    visited[src] = true;
    q2.push({src, false});

    while (!q2.empty()) {
        auto [u, fb] = q2.front(); q2.pop();
        if (u == dst && fb) return true;
        for (const auto& e : g.neighbors(u)) {
            bool new_fb = fb || e.is_fallback;
            if (!visited[e.dst]) {
                visited[e.dst] = true;
                used_fallback[e.dst] = new_fb;
                q2.push({e.dst, new_fb});
            } else if (new_fb && !used_fallback[e.dst]) {
                // Found a new fallback path to an already-visited node
                used_fallback[e.dst] = true;
                q2.push({e.dst, true});
            }
        }
    }
    return (visited[dst] && used_fallback[dst]);
}

// ----------------------------------------------------------
// Internal: run Dijkstra and recover the shortest path vector.
// Returns {path, cost}; empty path if unreachable.
// ----------------------------------------------------------
namespace detail {

inline PathCost dijkstra_path(const Graph& g, int src, int dst,
                               const std::unordered_set<int>& banned_nodes,
                               const std::vector<std::pair<int,int>>& banned_edges) {
    const int N = g.num_nodes();
    std::vector<double> dist(N, INF);
    std::vector<int>    prev(N, -1);
    dist[src] = 0.0;

    // Build banned_edges set for O(1) lookup
    std::unordered_set<long long> be_set;
    for (auto [u, v] : banned_edges)
        be_set.insert(static_cast<long long>(u) * N + v);

    using P = std::pair<double, int>;
    std::priority_queue<P, std::vector<P>, std::greater<P>> pq;
    pq.push({0.0, src});

    while (!pq.empty()) {
        auto [d, u] = pq.top(); pq.pop();
        if (d > dist[u]) continue;
        if (u == dst) break;
        for (const auto& e : g.neighbors(u)) {
            if (banned_nodes.count(e.dst)) continue;
            if (be_set.count(static_cast<long long>(u) * N + e.dst)) continue;
            double nd = d + e.weight;
            if (nd < dist[e.dst]) {
                dist[e.dst] = nd;
                prev[e.dst] = u;
                pq.push({nd, e.dst});
            }
        }
    }
    if (dist[dst] == INF) return {{}, INF};

    std::vector<int> path;
    for (int v = dst; v != -1; v = prev[v]) path.push_back(v);
    std::reverse(path.begin(), path.end());
    return {path, dist[dst]};
}

} // namespace detail

// ----------------------------------------------------------
// yen_k_shortest_paths — Yen's algorithm, O(k V (V+E) log V)
//
// Finds the k shortest simple paths from src to dst.
// Uses Dijkstra as the SSSP subroutine.
// Returns up to k paths sorted by cost ascending.
// ----------------------------------------------------------
[[nodiscard]] inline std::vector<PathCost>
yen_k_shortest_paths(const Graph& g, int src, int dst, int k) {
    g.range_check(src);
    g.range_check(dst);

    std::vector<PathCost> A; // confirmed k-shortest paths
    A.reserve(k);

    // Get the 1st shortest path
    auto [p0, c0] = detail::dijkstra_path(g, src, dst, {}, {});
    if (p0.empty()) return {}; // src and dst not connected
    A.push_back({p0, c0});

    // Candidate heap: (cost, path)
    using Cand = std::pair<double, std::vector<int>>;
    auto cmp = [](const Cand& a, const Cand& b){ return a.first > b.first; };
    std::priority_queue<Cand, std::vector<Cand>, decltype(cmp)> B(cmp);

    for (int ki = 1; ki < k; ++ki) {
        const auto& prev_path = A[ki - 1].first;
        const int   prev_len  = static_cast<int>(prev_path.size());

        for (int i = 0; i < prev_len - 1; ++i) {
            int spur_node = prev_path[i];
            std::vector<int> root_path(prev_path.begin(), prev_path.begin() + i + 1);

            // Ban edges used by previous paths with the same root
            std::vector<std::pair<int,int>> banned_edges;
            for (const auto& [ap, ac] : A) {
                if (static_cast<int>(ap.size()) > i) {
                    std::vector<int> root_candidate(ap.begin(), ap.begin() + i + 1);
                    if (root_candidate == root_path && static_cast<int>(ap.size()) > i + 1)
                        banned_edges.emplace_back(ap[i], ap[i + 1]);
                }
            }
            // Ban nodes in root_path (except spur_node) to ensure simple paths
            std::unordered_set<int> banned_nodes;
            for (int j = 0; j < i; ++j)
                banned_nodes.insert(root_path[j]);

            auto [spur_path, spur_cost] = detail::dijkstra_path(
                g, spur_node, dst, banned_nodes, banned_edges);

            if (!spur_path.empty()) {
                // Compute root cost
                double root_cost = 0.0;
                for (int j = 0; j + 1 < static_cast<int>(root_path.size()); ++j) {
                    int u = root_path[j], v = root_path[j+1];
                    for (const auto& e : g.neighbors(u))
                        if (e.dst == v) { root_cost += e.weight; break; }
                }
                // Total path = root_path[0..i] + spur_path[1..]
                std::vector<int> total_path = root_path;
                total_path.insert(total_path.end(),
                                  spur_path.begin() + 1, spur_path.end());
                double total_cost = root_cost + spur_cost;
                B.push({total_cost, std::move(total_path)});
            }
        }

        if (B.empty()) break;

        // Pop best candidate, deduplicate
        while (!B.empty()) {
            auto [bc, bp] = B.top(); B.pop();
            // Check not already in A
            bool dup = false;
            for (const auto& [ap, ac] : A)
                if (ap == bp) { dup = true; break; }
            if (!dup) {
                A.push_back({bp, bc});
                break;
            }
        }
        if (static_cast<int>(A.size()) == ki) break; // no new path found
    }
    return A;
}

// ----------------------------------------------------------
// resilience_score — O(k * V * (V+E) log V)
// Scores how resilient the src→dst path is based on:
//   • Number of distinct paths (diversity)
//   • Whether any fallback edges are on those paths
//   • Ratio of fallback path cost to primary path cost
// Returns a score in [0.0, 1.0] (higher = more resilient).
// ----------------------------------------------------------
[[nodiscard]] inline double
resilience_score(const Graph& g, int src, int dst, int k = 5) {
    auto paths = yen_k_shortest_paths(g, src, dst, k);
    if (paths.empty()) return 0.0;
    if (paths.size() == 1) return 0.1; // Only one path — minimal resilience

    // Score based on: number of available paths, fallback presence, and
    // cost ratio between cheapest and most expensive path (lower spread = good).
    double num_paths = static_cast<double>(paths.size());
    double diversity = std::min(1.0, num_paths / k);

    double min_cost = paths.front().second;
    double max_cost = paths.back().second;
    double cost_spread = (max_cost > 0) ? min_cost / max_cost : 1.0;

    // Check if fallback edges are used in any path
    bool has_fb = false;
    for (const auto& [path, cost] : paths) {
        for (int i = 0; i + 1 < static_cast<int>(path.size()); ++i) {
            int u = path[i], v = path[i+1];
            for (const auto& e : g.neighbors(u))
                if (e.dst == v && e.is_fallback) { has_fb = true; break; }
            if (has_fb) break;
        }
        if (has_fb) break;
    }
    double fallback_bonus = has_fb ? 0.1 : 0.0;

    // Combined score
    return std::min(1.0, 0.5 * diversity + 0.4 * cost_spread + fallback_bonus);
}

} // namespace txn
