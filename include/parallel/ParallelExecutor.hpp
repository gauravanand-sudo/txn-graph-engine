#pragma once

#include "ThreadPool.hpp"
#include "Graph.hpp"
#include "analysis/FailurePropagation.hpp"
#include "analysis/BlastRadius.hpp"
#include <vector>
#include <future>
#include <mutex>
#include <iostream>
#include <string>
#include <algorithm>

// ============================================================
// ParallelExecutor.hpp — Parallel graph partition analysis
//
// run_parallel_analysis distributes FailurePropagation and
// BlastRadius analysis across graph partitions using the
// ThreadPool.  Each partition runs in its own thread; results
// are collected after all futures resolve.
//
// This mirrors real-world EDA batch simulation where independent
// sub-circuits are analysed concurrently to minimise wall-clock
// time.
// ============================================================

namespace txn {

// ----------------------------------------------------------
// PartitionResult — per-partition analysis output
// ----------------------------------------------------------
struct PartitionResult {
    int                      partition_id;
    std::vector<int>         nodes;           ///< node ids in this partition
    std::vector<FailureResult>   failure_results; ///< one per node in partition
    std::vector<BlastRadiusReport> blast_reports;
    double                   worst_impact_score{0.0};
};

// ----------------------------------------------------------
// AggregatedResult — combined output across all partitions
// ----------------------------------------------------------
struct AggregatedResult {
    std::vector<PartitionResult> partition_results;
    int                          total_affected_nodes{0};
    double                       max_revenue_impact{0.0};
    int                          most_critical_partition{-1};
};

// ----------------------------------------------------------
// run_parallel_analysis — O(partitions * V/partition * (V+E))
//
// For each partition:
//   • Simulate failure of every node in the partition (via
//     the FULL graph, not just the partition subgraph —
//     cross-partition blast radius matters).
//   • Compute BlastRadius for each node.
//   • Collect results.
//
// thread_count: number of worker threads (defaults to HW
//               concurrency).
// ----------------------------------------------------------
[[nodiscard]] inline AggregatedResult
run_parallel_analysis(const Graph& g,
                      const std::vector<std::vector<int>>& partitions,
                      std::size_t thread_count = 0) {
    if (thread_count == 0)
        thread_count = std::max(1u, std::thread::hardware_concurrency());
    thread_count = std::min(thread_count, partitions.size());

    ThreadPool pool(thread_count);
    AggregatedResult aggregated;
    aggregated.partition_results.resize(partitions.size());

    std::vector<std::future<PartitionResult>> futures;
    futures.reserve(partitions.size());

    for (int pid = 0; pid < static_cast<int>(partitions.size()); ++pid) {
        const auto& part = partitions[pid];
        // Submit analysis of this partition to the thread pool
        futures.push_back(
            pool.enqueue([&g, part, pid]() -> PartitionResult {
                PartitionResult pr;
                pr.partition_id = pid;
                pr.nodes        = part;

                for (int node_id : part) {
                    // Failure simulation for this node across entire graph
                    auto fr = simulate_failure(g, node_id, 0.05);
                    pr.failure_results.push_back(fr);
                    pr.worst_impact_score =
                        std::max(pr.worst_impact_score, fr.total_impact_score);

                    // Blast radius
                    auto br = compute_blast_radius(g, node_id, 0.05);
                    pr.blast_reports.push_back(br);
                }
                return pr;
            })
        );
    }

    // Collect results
    double max_revenue = 0.0;
    int    max_partition = -1;
    int    total_affected = 0;

    for (int pid = 0; pid < static_cast<int>(futures.size()); ++pid) {
        aggregated.partition_results[pid] = futures[pid].get();
        const auto& pr = aggregated.partition_results[pid];

        for (const auto& br : pr.blast_reports) {
            total_affected += br.affected_count;
            if (br.estimated_revenue_impact_usd_hr > max_revenue) {
                max_revenue   = br.estimated_revenue_impact_usd_hr;
                max_partition = pid;
            }
        }
    }
    aggregated.total_affected_nodes    = total_affected;
    aggregated.max_revenue_impact      = max_revenue;
    aggregated.most_critical_partition = max_partition;

    return aggregated;
}

} // namespace txn
