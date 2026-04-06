#pragma once

#include "Graph.hpp"
#include "FailurePropagation.hpp"
#include <string>
#include <vector>
#include <unordered_map>
#include <algorithm>

// ============================================================
// BlastRadius.hpp — Impact classification for service failures
//
// Wraps FailurePropagation with business-domain intelligence:
//   • Counts affected services by type (DB, GATEWAY, etc.)
//   • Classifies severity: LOW / MEDIUM / HIGH / CRITICAL
//   • Identifies critical service names in the blast zone
//   • Provides a mock revenue-impact estimate
// ============================================================

namespace txn {

// ----------------------------------------------------------
// Severity thresholds — tunable constants
// ----------------------------------------------------------
constexpr double SEVERITY_LOW_THRESHOLD      = 0.02; ///< < 2% of services
constexpr double SEVERITY_MEDIUM_THRESHOLD   = 0.15; ///< < 15%
constexpr double SEVERITY_HIGH_THRESHOLD     = 0.40; ///< < 40%
// CRITICAL: ≥ 40%

// Revenue impact mock model (USD/hour by node type)
constexpr double REVENUE_INTERNAL_PER_HR  = 10'000.0;
constexpr double REVENUE_EXTERNAL_PER_HR  =  5'000.0;
constexpr double REVENUE_GATEWAY_PER_HR   = 50'000.0;
constexpr double REVENUE_DATABASE_PER_HR  = 20'000.0;

enum class Severity : uint8_t { LOW, MEDIUM, HIGH, CRITICAL };

inline std::string_view severity_str(Severity s) {
    switch (s) {
        case Severity::LOW:      return "LOW";
        case Severity::MEDIUM:   return "MEDIUM";
        case Severity::HIGH:     return "HIGH";
        case Severity::CRITICAL: return "CRITICAL";
    }
    return "UNKNOWN";
}

// ----------------------------------------------------------
// BlastRadiusReport — rich impact report
// ----------------------------------------------------------
struct BlastRadiusReport {
    int                  failed_node;
    int                  affected_count;
    int                  total_nodes;
    Severity             severity;
    std::vector<std::string> affected_critical_services; ///< GATEWAYs and DBs hit
    double               estimated_revenue_impact_usd_hr; ///< mock estimate
    std::unordered_map<int,double> per_node_probability;
    // breakdown by type
    int                  affected_gateways{0};
    int                  affected_databases{0};
    int                  affected_internals{0};
    int                  affected_externals{0};
};

// ----------------------------------------------------------
// compute_blast_radius — O(V + E)
// ----------------------------------------------------------
[[nodiscard]] inline BlastRadiusReport
compute_blast_radius(const Graph& g, int failed_node,
                     double failure_threshold = 0.1) {
    g.range_check(failed_node);
    const int N = g.num_nodes();

    auto fr = simulate_failure(g, failed_node, failure_threshold);

    BlastRadiusReport report;
    report.failed_node           = failed_node;
    report.affected_count        = static_cast<int>(fr.affected_nodes.size());
    report.total_nodes           = N;
    report.per_node_probability  = fr.failure_prob;

    // Classify severity based on affected fraction
    double fraction = static_cast<double>(report.affected_count) / N;
    if (fraction < SEVERITY_LOW_THRESHOLD)
        report.severity = Severity::LOW;
    else if (fraction < SEVERITY_MEDIUM_THRESHOLD)
        report.severity = Severity::MEDIUM;
    else if (fraction < SEVERITY_HIGH_THRESHOLD)
        report.severity = Severity::HIGH;
    else
        report.severity = Severity::CRITICAL;

    // Revenue impact estimate + breakdown
    double revenue = 0.0;
    for (int v : fr.affected_nodes) {
        const auto& node = g.get_node(v);
        double prob = fr.failure_prob.count(v) ? fr.failure_prob.at(v) : 0.0;
        switch (node.type) {
            case NodeType::GATEWAY:
                ++report.affected_gateways;
                revenue += REVENUE_GATEWAY_PER_HR  * prob;
                report.affected_critical_services.push_back(node.name);
                break;
            case NodeType::DATABASE:
                ++report.affected_databases;
                revenue += REVENUE_DATABASE_PER_HR * prob;
                report.affected_critical_services.push_back(node.name);
                break;
            case NodeType::INTERNAL:
                ++report.affected_internals;
                revenue += REVENUE_INTERNAL_PER_HR * prob;
                break;
            case NodeType::EXTERNAL:
                ++report.affected_externals;
                revenue += REVENUE_EXTERNAL_PER_HR * prob;
                break;
        }
    }
    // Also account for the failed node itself
    {
        const auto& fn = g.get_node(failed_node);
        switch (fn.type) {
            case NodeType::GATEWAY:  revenue += REVENUE_GATEWAY_PER_HR;  break;
            case NodeType::DATABASE: revenue += REVENUE_DATABASE_PER_HR; break;
            case NodeType::INTERNAL: revenue += REVENUE_INTERNAL_PER_HR; break;
            case NodeType::EXTERNAL: revenue += REVENUE_EXTERNAL_PER_HR; break;
        }
    }
    report.estimated_revenue_impact_usd_hr = revenue;
    return report;
}

} // namespace txn
