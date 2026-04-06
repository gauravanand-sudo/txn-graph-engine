#pragma once

#include "Graph.hpp"
#include "Connectivity.hpp"
#include "TopologicalSort.hpp"
#include "SCC.hpp"
#include <vector>
#include <algorithm>
#include <numeric>

// ============================================================
// Partitioning.hpp — Graph partitioning for parallel execution
//
// • partition_for_parallel      : WCC-based — each component
//   is an independent unit that can run on a separate thread.
// • topological_wave_partition  : toposort level groups — nodes
//   in the same level can start concurrently (dependency wave).
// • partition_by_scc            : SCCs form atomic units;
//   condensation DAG levels give inter-SCC parallelism.
// ============================================================

namespace txn {

// ----------------------------------------------------------
// partition_for_parallel — O(E α(V))
// Splits the graph into weakly connected components.
// Each component is fully independent and can be processed
// by a separate worker thread.
// ----------------------------------------------------------
[[nodiscard]] inline std::vector<std::vector<int>>
partition_for_parallel(const Graph& g) {
    return weakly_connected_components(g);
}

// ----------------------------------------------------------
// topological_wave_partition — O(V + E)
// Groups nodes by their topological "level" (depth from all
// source nodes in the DAG). All nodes in level k can start
// execution simultaneously once level k-1 has completed.
// Returns an empty outer vector if the graph has a cycle.
// ----------------------------------------------------------
[[nodiscard]] inline std::vector<std::vector<int>>
topological_wave_partition(const Graph& g) {
    return toposort_levels(g);
}

// ----------------------------------------------------------
// partition_by_scc — O(V + E)
// 1. Compute SCCs (each SCC = atomic execution unit, possibly
//    containing circular dependencies that must be serialised).
// 2. Build condensation DAG.
// 3. Apply wave-level partitioning on the condensation DAG.
// Returns each "wave" as a list of SCC member node ids.
// ----------------------------------------------------------
[[nodiscard]] inline std::vector<std::vector<int>>
partition_by_scc(const Graph& g) {
    // Step 1: compute SCCs
    auto sccs = tarjan_scc(g);

    // Step 2: condensation graph
    auto [meta, comp_id] = condensation_graph(g, sccs);

    // Step 3: wave partition on condensation DAG
    auto meta_levels = toposort_levels(meta);

    // Expand meta-node ids back to original node ids
    std::vector<std::vector<int>> result;
    result.reserve(meta_levels.size());
    for (const auto& level : meta_levels) {
        std::vector<int> wave;
        for (int scc_id : level)
            for (int v : sccs[scc_id])
                wave.push_back(v);
        std::sort(wave.begin(), wave.end());
        result.push_back(std::move(wave));
    }
    return result;
}

} // namespace txn
