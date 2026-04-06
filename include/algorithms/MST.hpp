#pragma once

#include "Graph.hpp"
#include <vector>
#include <algorithm>
#include <queue>
#include <numeric>
#include <limits>

// ============================================================
// MST.hpp — Minimum Spanning Tree algorithms
//
// • Kruskal — Union-Find with path compression + union by rank
// • Prim    — priority_queue-based O((V+E) log V)
// • Borůvka — O(E log V), finds cheapest edge per component
//
// MST reveals the "cheapest backbone" connectivity of the
// service graph — useful for identifying which service links
// are critical for minimal-cost failover routing.
// ============================================================

namespace txn {

// ----------------------------------------------------------
// Union-Find (Disjoint Set Union) — internal helper
// Path compression + union by rank → O(α(N)) per operation
// ----------------------------------------------------------
struct DSU {
    std::vector<int> parent, rank_;

    explicit DSU(int n) : parent(n), rank_(n, 0) {
        std::iota(parent.begin(), parent.end(), 0);
    }

    int find(int x) {
        while (parent[x] != x) {
            parent[x] = parent[parent[x]]; // path halving
            x = parent[x];
        }
        return x;
    }

    /// Returns true if x and y were in different sets (union performed).
    bool unite(int x, int y) {
        x = find(x); y = find(y);
        if (x == y) return false;
        if (rank_[x] < rank_[y]) std::swap(x, y);
        parent[y] = x;
        if (rank_[x] == rank_[y]) ++rank_[x];
        return true;
    }

    bool same(int x, int y) { return find(x) == find(y); }
};

// ----------------------------------------------------------
// Kruskal's MST — O(E log E)
// Sort all edges by weight, greedily add if they don't form
// a cycle (checked via DSU).
// ----------------------------------------------------------
[[nodiscard]] inline std::vector<ServiceEdge>
kruskal_mst(const Graph& g) {
    auto edges = g.all_edges();
    // Sort edges by weight ascending
    std::sort(edges.begin(), edges.end(),
              [](const ServiceEdge& a, const ServiceEdge& b) {
                  return a.weight < b.weight;
              });

    const int N = g.num_nodes();
    DSU dsu(N);
    std::vector<ServiceEdge> mst;
    mst.reserve(N - 1);

    for (const auto& e : edges) {
        if (dsu.unite(e.src, e.dst)) {
            mst.push_back(e);
            if (static_cast<int>(mst.size()) == N - 1) break; // Done
        }
    }
    return mst;
}

// ----------------------------------------------------------
// Prim's MST — O((V + E) log V) using a min-heap
// Starts from `start` and grows the MST greedily by always
// picking the minimum-weight edge crossing the cut.
// ----------------------------------------------------------
[[nodiscard]] inline std::vector<ServiceEdge>
prim_mst(const Graph& g, int start = 0) {
    g.range_check(start);
    const int N = g.num_nodes();
    std::vector<bool> in_mst(N, false);
    // min-heap: (weight, src, dst)
    using T = std::tuple<double, int, int, double>; // weight, src, dst, cap
    std::priority_queue<T, std::vector<T>, std::greater<T>> pq;

    // Seed with edges from start
    in_mst[start] = true;
    for (const auto& e : g.neighbors(start))
        pq.push({e.weight, e.src, e.dst, e.capacity});

    std::vector<ServiceEdge> mst;
    mst.reserve(N - 1);

    while (!pq.empty() && static_cast<int>(mst.size()) < N - 1) {
        auto [w, u, v, cap] = pq.top(); pq.pop();
        if (in_mst[v]) continue;
        in_mst[v] = true;
        mst.emplace_back(u, v, w, cap);
        for (const auto& e : g.neighbors(v)) {
            if (!in_mst[e.dst])
                pq.push({e.weight, e.src, e.dst, e.capacity});
        }
    }
    return mst;
}

// ----------------------------------------------------------
// Borůvka's MST — O(E log V)
// Each round, every component picks its cheapest outgoing edge
// and merges. Rounds continue until the MST is complete.
// Naturally parallelisable (each component acts independently).
// ----------------------------------------------------------
[[nodiscard]] inline std::vector<ServiceEdge>
boruvka_mst(const Graph& g) {
    const int N = g.num_nodes();
    DSU dsu(N);
    std::vector<ServiceEdge> mst;
    mst.reserve(N - 1);

    int num_components = N;

    while (num_components > 1) {
        // cheapest[comp] = cheapest outgoing edge for that component
        std::vector<std::optional<ServiceEdge>> cheapest(N);

        // Scan all edges
        for (int u = 0; u < N; ++u) {
            for (const auto& e : g.neighbors(u)) {
                int cu = dsu.find(u);
                int cv = dsu.find(e.dst);
                if (cu == cv) continue; // same component
                if (!cheapest[cu] || e.weight < cheapest[cu]->weight)
                    cheapest[cu] = e;
                if (!cheapest[cv] || e.weight < cheapest[cv]->weight)
                    cheapest[cv] = e;
            }
        }

        bool merged_any = false;
        for (int c = 0; c < N; ++c) {
            if (!cheapest[c]) continue;
            const auto& e = *cheapest[c];
            if (dsu.unite(e.src, e.dst)) {
                mst.push_back(e);
                --num_components;
                merged_any = true;
            }
        }
        if (!merged_any) break; // Disconnected graph
    }
    return mst;
}

} // namespace txn
