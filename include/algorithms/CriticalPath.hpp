#pragma once

#include "Graph.hpp"
#include "TopologicalSort.hpp"
#include <vector>
#include <algorithm>
#include <limits>
#include <stdexcept>

// ============================================================
// CriticalPath.hpp — CPM / PERT critical path analysis
//
// Computes for every node in a DAG:
//   EST — Earliest Start Time
//   EFT — Earliest Finish Time  (= EST + node.base_latency_ms)
//   LST — Latest Start Time
//   LFT — Latest Finish Time
//   Slack = LST - EST  (0 means the node is on the critical path)
//
// The critical path is the longest path through the DAG, which
// determines the minimum end-to-end transaction latency.
// ============================================================

namespace txn {

struct NodeTiming {
    double EST{0.0};    ///< Earliest Start Time
    double EFT{0.0};    ///< Earliest Finish Time
    double LST{0.0};    ///< Latest Start Time
    double LFT{0.0};    ///< Latest Finish Time
    double slack{0.0};  ///< LFT - EFT (or LST - EST)
    bool   on_critical_path{false};
};

struct CriticalPathResult {
    std::vector<NodeTiming> timings;          ///< per-node timing
    std::vector<int>        critical_path;    ///< nodes on critical path (in order)
    double                  project_duration; ///< total end-to-end duration (ms)
};

// ----------------------------------------------------------
// compute_critical_path — O(V + E)
// Requires the graph to be a DAG (toposort must succeed).
// Edge weights represent the transition latency between nodes.
// Node latency is taken from ServiceNode::base_latency_ms.
// ----------------------------------------------------------
[[nodiscard]] inline CriticalPathResult
compute_critical_path(const Graph& g) {
    const int N = g.num_nodes();

    // Step 0: topological order is required
    auto order = kahn_toposort(g);
    if (order.empty())
        throw std::runtime_error("compute_critical_path: graph contains a cycle");

    std::vector<NodeTiming> T(N);

    // ---- Forward pass: compute EST / EFT ----
    // EST[v] = max over all predecessors u of (EFT[u] + edge_weight(u,v))
    // Note: EFT[u] already includes node u's own latency.

    // Initialise EST to 0 for all nodes (sources start at 0)
    for (int v : order)
        T[v].EST = 0.0;

    // Build incoming edge info for the forward pass
    // We iterate in topological order, propagating EFT to successors.
    for (int v : order) {
        double node_lat_v = g.get_node(v).base_latency_ms;
        T[v].EFT = T[v].EST + node_lat_v;
        for (const auto& e : g.neighbors(v)) {
            double arrive = T[v].EFT + e.weight; // edge weight = transit latency
            if (arrive > T[e.dst].EST)
                T[e.dst].EST = arrive;
        }
    }

    // Project duration = max EFT over all sink nodes (nodes with no outgoing
    // edges in the topological order, or simply the global maximum EFT).
    double duration = 0.0;
    for (int v = 0; v < N; ++v)
        duration = std::max(duration, T[v].EFT);

    // ---- Backward pass: compute LFT / LST ----
    // Initialise LFT of all nodes to project_duration.
    for (int v = 0; v < N; ++v)
        T[v].LFT = duration;

    // First backward pass: propagate LFT constraints from successors
    // (This pass is intentionally left as a placeholder; the clean pass below
    //  overwrites everything — we keep it to document the two-pass intent.)
    for (int i = static_cast<int>(order.size()) - 1; i >= 0; --i) {
        int u = order[i];
        (void)u; // processed in the second clean backward pass below
    }

    // Re-do backward pass cleanly:
    // LFT[v] for a node without successors = project_duration.
    // For internal nodes: LFT[u] = min over successors v of (LST[v] - edge(u,v))
    for (int v = 0; v < N; ++v)
        T[v].LFT = duration;

    for (int i = static_cast<int>(order.size()) - 1; i >= 0; --i) {
        int u = order[i];
        for (const auto& e : g.neighbors(u)) {
            int v       = e.dst;
            double node_lat_v = g.get_node(v).base_latency_ms;
            // LFT[u] <= LST[v] - edge_weight
            double constraint = T[v].LFT - node_lat_v - e.weight;
            T[u].LFT = std::min(T[u].LFT, constraint);
        }
        double node_lat = g.get_node(u).base_latency_ms;
        T[u].LST  = T[u].LFT - node_lat;
        T[u].slack = T[u].LFT - T[u].EFT; // Total float
        T[u].on_critical_path = (T[u].slack <= 1e-9);
    }

    // Extract critical path in order
    std::vector<int> cp;
    for (int v : order)
        if (T[v].on_critical_path) cp.push_back(v);

    return {std::move(T), std::move(cp), duration};
}

/// Convenience: returns just the project duration (ms).
[[nodiscard]] inline double
project_duration(const Graph& g) {
    return compute_critical_path(g).project_duration;
}

} // namespace txn
