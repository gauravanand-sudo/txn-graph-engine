#pragma once

#include "Graph.hpp"
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <algorithm>
#include <numeric>
#include <set>

// ============================================================
// Coloring.hpp — Graph coloring algorithms
//
// • greedy_coloring       : simple degree-ordered greedy, O(V+E)
// • dsatur_coloring       : DSatur (Degree of SATURation), better
//   greedy using saturation degree — often optimal in practice
// • chromatic_number_estimate : returns the chromatic number from DSatur
//
// In transaction scheduling, coloring assigns a "slot" (color)
// to each service such that two services sharing a resource
// (edge) are never in the same slot — enabling conflict-free
// parallel execution.
// ============================================================

namespace txn {

// ----------------------------------------------------------
// Greedy coloring — O(V + E)
// Colour nodes in order of decreasing degree (Welsh-Powell
// heuristic), assigning the smallest available color.
// ----------------------------------------------------------
[[nodiscard]] inline std::unordered_map<int,int>
greedy_coloring(const Graph& g) {
    const int N = g.num_nodes();
    // Build undirected neighbor sets for color conflict checking
    std::vector<std::unordered_set<int>> undirected_nbr(N);
    for (int u = 0; u < N; ++u) {
        for (const auto& e : g.neighbors(u)) {
            undirected_nbr[u].insert(e.dst);
            undirected_nbr[e.dst].insert(u);
        }
    }

    // Sort by decreasing degree (Welsh-Powell)
    std::vector<int> order(N);
    std::iota(order.begin(), order.end(), 0);
    std::sort(order.begin(), order.end(), [&](int a, int b) {
        return undirected_nbr[a].size() > undirected_nbr[b].size();
    });

    std::vector<int> color(N, -1);
    for (int v : order) {
        // Find the smallest color not used by any neighbor
        std::unordered_set<int> used;
        for (int nb : undirected_nbr[v])
            if (color[nb] != -1) used.insert(color[nb]);
        int c = 0;
        while (used.count(c)) ++c;
        color[v] = c;
    }

    std::unordered_map<int,int> result;
    result.reserve(N);
    for (int v = 0; v < N; ++v) result[v] = color[v];
    return result;
}

// ----------------------------------------------------------
// DSatur coloring — O(V^2) worst case, often near-optimal
//
// At each step, choose the uncolored vertex with the highest
// saturation degree (number of distinct colors in its
// neighborhood). Ties broken by largest degree.
// ----------------------------------------------------------
[[nodiscard]] inline std::unordered_map<int,int>
dsatur_coloring(const Graph& g) {
    const int N = g.num_nodes();

    // Build undirected adjacency sets
    std::vector<std::unordered_set<int>> nbr(N);
    for (int u = 0; u < N; ++u)
        for (const auto& e : g.neighbors(u)) {
            nbr[u].insert(e.dst);
            nbr[e.dst].insert(u);
        }

    std::vector<int> color(N, -1);
    // saturation[v] = number of distinct colors among colored neighbors
    std::vector<int> saturation(N, 0);
    // degree[v] = total undirected degree
    std::vector<int> degree(N);
    for (int v = 0; v < N; ++v)
        degree[v] = static_cast<int>(nbr[v].size());

    std::vector<bool> colored(N, false);

    for (int step = 0; step < N; ++step) {
        // Pick uncolored vertex with max saturation; break ties by degree
        int chosen = -1;
        for (int v = 0; v < N; ++v) {
            if (colored[v]) continue;
            if (chosen == -1) { chosen = v; continue; }
            if (saturation[v] > saturation[chosen] ||
                (saturation[v] == saturation[chosen] &&
                 degree[v] > degree[chosen]))
                chosen = v;
        }

        // Assign smallest available color
        std::unordered_set<int> used;
        for (int nb : nbr[chosen])
            if (colored[nb]) used.insert(color[nb]);
        int c = 0;
        while (used.count(c)) ++c;
        color[chosen] = c;
        colored[chosen] = true;

        // Update saturation of uncolored neighbors
        for (int nb : nbr[chosen]) {
            if (!colored[nb]) {
                // Recompute saturation for nb (count distinct colors among colored neighbors)
                std::unordered_set<int> nb_colors;
                for (int nb2 : nbr[nb])
                    if (colored[nb2]) nb_colors.insert(color[nb2]);
                saturation[nb] = static_cast<int>(nb_colors.size());
            }
        }
    }

    std::unordered_map<int,int> result;
    result.reserve(N);
    for (int v = 0; v < N; ++v) result[v] = color[v];
    return result;
}

// ----------------------------------------------------------
// chromatic_number_estimate — uses DSatur coloring and returns
// the number of distinct colors used as an upper bound on the
// chromatic number.
// ----------------------------------------------------------
[[nodiscard]] inline int
chromatic_number_estimate(const Graph& g) {
    auto coloring = dsatur_coloring(g);
    int max_color = 0;
    for (const auto& [v, c] : coloring)
        max_color = std::max(max_color, c);
    return max_color + 1; // colors are 0-indexed
}

} // namespace txn
