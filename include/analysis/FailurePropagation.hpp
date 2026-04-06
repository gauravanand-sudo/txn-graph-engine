#pragma once

#include "Graph.hpp"
#include "algorithms/Traversal.hpp"
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <queue>
#include <algorithm>
#include <string>

// ============================================================
// FailurePropagation.hpp — Cascade failure simulation
//
// Simulates how a failure propagates downstream through the
// service dependency graph.  Each service has a `health` score
// (0.0–1.0); unhealthy services amplify failure probability.
//
// Model:
//   • Failed node f sets propagation_prob[f] = 1.0.
//   • For each downstream neighbour v reachable from f via BFS:
//       P(v fails) = 1 - prod(1 - P(u fails) * (1 - health[v]))
//                   for all failed predecessors u of v
//   • A node is "affected" if its failure probability ≥ threshold.
// ============================================================

namespace txn {

// ----------------------------------------------------------
// FailureResult — output of a failure simulation
// ----------------------------------------------------------
struct FailureResult {
    std::unordered_set<int>   failed_nodes;          ///< seed failures
    std::vector<int>          affected_nodes;         ///< nodes with P(fail) > threshold
    std::unordered_map<int,double> failure_prob;      ///< estimated failure probability per node
    int                       cascade_depth{0};       ///< max BFS depth reached
    double                    total_impact_score{0};  ///< sum of failure probs (severity index)
};

// ----------------------------------------------------------
// simulate_failure — single failed node
// threshold: minimum P(fail) to classify a node as "affected"
// ----------------------------------------------------------
[[nodiscard]] inline FailureResult
simulate_failure(const Graph& g, int failed_node,
                 double threshold = 0.1) {
    g.range_check(failed_node);
    const int N = g.num_nodes();

    FailureResult result;
    result.failed_nodes.insert(failed_node);

    // failure_prob[v]: probability that v will fail given the seed failure
    std::vector<double> fp(N, 0.0);
    fp[failed_node] = 1.0;

    // BFS downstream
    std::vector<int> depth(N, -1);
    std::queue<int>  q;
    depth[failed_node] = 0;
    q.push(failed_node);

    int max_depth = 0;

    while (!q.empty()) {
        int u = q.front(); q.pop();
        max_depth = std::max(max_depth, depth[u]);

        for (const auto& e : g.neighbors(u)) {
            int v = e.dst;
            // Accumulate failure probability for v from u
            // P_new(v fails) = 1 - (1 - P_old(v fails)) * (1 - fp[u] * (1 - health[v]))
            double health_v = g.get_node(v).health;
            double prob_transfer = fp[u] * (1.0 - health_v);
            fp[v] = 1.0 - (1.0 - fp[v]) * (1.0 - prob_transfer);

            if (depth[v] == -1) {
                depth[v] = depth[u] + 1;
                q.push(v);
            }
        }
    }

    // Collect affected nodes
    for (int v = 0; v < N; ++v) {
        if (v == failed_node) continue;
        if (fp[v] >= threshold) {
            result.affected_nodes.push_back(v);
            result.total_impact_score += fp[v];
        }
        result.failure_prob[v] = fp[v];
    }
    result.failure_prob[failed_node] = 1.0;
    result.total_impact_score += 1.0;
    result.cascade_depth = max_depth;

    std::sort(result.affected_nodes.begin(), result.affected_nodes.end());
    return result;
}

// ----------------------------------------------------------
// simulate_multi_failure — multiple failed nodes (simultaneous)
// Runs BFS from all seeds concurrently; failure probabilities
// from different sources combine using the OR rule:
//   P(v fails from any seed) = 1 - prod(1 - P(v|seed_i))
// ----------------------------------------------------------
[[nodiscard]] inline FailureResult
simulate_multi_failure(const Graph& g,
                       const std::vector<int>& failed_nodes,
                       double threshold = 0.1) {
    const int N = g.num_nodes();
    FailureResult combined;
    for (int fn : failed_nodes) {
        g.range_check(fn);
        combined.failed_nodes.insert(fn);
    }

    // Run individual simulations and combine
    std::vector<double> combined_fp(N, 0.0);
    int max_depth = 0;

    for (int fn : failed_nodes) {
        auto single = simulate_failure(g, fn, threshold);
        max_depth = std::max(max_depth, single.cascade_depth);
        for (int v = 0; v < N; ++v) {
            // OR-combine: 1 - (1-p1)(1-p2)...
            double pv = 0.0;
            auto it = single.failure_prob.find(v);
            if (it != single.failure_prob.end()) pv = it->second;
            combined_fp[v] = 1.0 - (1.0 - combined_fp[v]) * (1.0 - pv);
        }
    }

    combined.cascade_depth = max_depth;
    for (int v = 0; v < N; ++v) {
        combined.failure_prob[v] = combined_fp[v];
        if (combined.failed_nodes.count(v)) combined_fp[v] = 1.0;
        if (combined_fp[v] >= threshold &&
            !combined.failed_nodes.count(v))
            combined.affected_nodes.push_back(v);
        combined.total_impact_score += combined_fp[v];
    }
    std::sort(combined.affected_nodes.begin(), combined.affected_nodes.end());
    return combined;
}

} // namespace txn
