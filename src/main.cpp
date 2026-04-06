// ============================================================
// main.cpp — Transaction Workflow Graph Analysis Demo
//
// Builds a realistic 15-node distributed transaction workflow
// and exercises every algorithm in the txn-graph-engine:
//
//   1.  BFS / DFS traversal
//   2.  Dijkstra shortest path
//   3.  Floyd-Warshall all-pairs
//   4.  Topological sort + dependency levels
//   5.  Tarjan SCC + Kosaraju SCC
//   6.  Kruskal MST + Prim MST + Borůvka MST
//   7.  Edmonds-Karp max flow + Dinic max flow
//   8.  Bridges + Articulation points
//   9.  Cycle detection
//  10.  Betweenness centrality + PageRank
//  11.  Critical path (CPM / PERT)
//  12.  Transitive closure
//  13.  DSatur graph coloring
//  14.  Multi-component partitioning
//  15.  Failure simulation: PAYMENT_SERVICE fails
//  16.  Blast radius report
//  17.  Latency analysis + p99 estimate
//  18.  Yen's k-shortest fallback paths
//  19.  Parallel analysis across partitions
// ============================================================

// Core graph
#include "Graph.hpp"

// Algorithms
#include "algorithms/Traversal.hpp"
#include "algorithms/ShortestPath.hpp"
#include "algorithms/TopologicalSort.hpp"
#include "algorithms/SCC.hpp"
#include "algorithms/MST.hpp"
#include "algorithms/MaxFlow.hpp"
#include "algorithms/Connectivity.hpp"
#include "algorithms/CycleDetection.hpp"
#include "algorithms/Centrality.hpp"
#include "algorithms/CriticalPath.hpp"
#include "algorithms/TransitiveClosure.hpp"
#include "algorithms/Coloring.hpp"
#include "algorithms/Partitioning.hpp"

// Analysis
#include "analysis/FailurePropagation.hpp"
#include "analysis/BlastRadius.hpp"
#include "analysis/LatencyAnalysis.hpp"
#include "analysis/FallbackAnalysis.hpp"

// Parallel
#include "parallel/ThreadPool.hpp"
#include "parallel/ParallelExecutor.hpp"

#include <iostream>
#include <iomanip>
#include <string>
#include <vector>
#include <sstream>

using namespace txn;

// ============================================================
// Pretty-print helpers
// ============================================================

static void section(const std::string& title) {
    constexpr int WIDTH = 68;
    std::cout << "\n";
    std::cout << std::string(WIDTH, '=') << "\n";
    std::cout << "  " << title << "\n";
    std::cout << std::string(WIDTH, '=') << "\n";
}

static void subsection(const std::string& title) {
    std::cout << "\n--- " << title << " ---\n";
}

static void print_node_list(const Graph& g, const std::vector<int>& ids,
                             const std::string& label) {
    std::cout << label << ": [";
    for (int i = 0; i < static_cast<int>(ids.size()); ++i) {
        if (i) std::cout << ", ";
        std::cout << g.get_node(ids[i]).name;
    }
    std::cout << "]\n";
}

static void print_edge_list(const Graph& g,
                             const std::vector<ServiceEdge>& edges,
                             const std::string& label) {
    std::cout << label << ":\n";
    for (const auto& e : edges) {
        std::cout << "  " << g.get_node(e.src).name
                  << " -> " << g.get_node(e.dst).name
                  << "  weight=" << std::fixed << std::setprecision(1) << e.weight
                  << "ms" << (e.is_fallback ? " [FALLBACK]" : "") << "\n";
    }
}

// ============================================================
// Build the 15-node transaction workflow graph
//
// Node IDs (0-based):
//   0  API_GATEWAY
//   1  AUTH_SERVICE
//   2  USER_DB
//   3  PAYMENT_SERVICE
//   4  PAYMENT_DB
//   5  FRAUD_DETECTOR
//   6  NOTIFICATION_SERVICE
//   7  EMAIL_GATEWAY
//   8  SMS_GATEWAY
//   9  INVENTORY_SERVICE
//  10  INVENTORY_DB
//  11  ORDER_SERVICE
//  12  ORDER_DB
//  13  AUDIT_SERVICE
//  14  PAYMENT_GATEWAY_SECONDARY  (fallback)
// ============================================================

static Graph build_txn_graph() {
    Graph g;

    // Add nodes (id, name, type, health, base_latency_ms, has_fallback)
    g.add_node({0,  "API_GATEWAY",               NodeType::GATEWAY,  0.99, 2.0,  false});
    g.add_node({1,  "AUTH_SERVICE",               NodeType::INTERNAL, 0.98, 5.0,  false});
    g.add_node({2,  "USER_DB",                    NodeType::DATABASE,  0.97, 8.0,  false});
    g.add_node({3,  "PAYMENT_SERVICE",            NodeType::INTERNAL, 0.95, 12.0, true });
    g.add_node({4,  "PAYMENT_DB",                 NodeType::DATABASE,  0.96, 10.0, false});
    g.add_node({5,  "FRAUD_DETECTOR",             NodeType::INTERNAL, 0.90, 20.0, false});
    g.add_node({6,  "NOTIFICATION_SERVICE",       NodeType::INTERNAL, 0.92, 4.0,  false});
    g.add_node({7,  "EMAIL_GATEWAY",              NodeType::EXTERNAL,  0.85, 15.0, true });
    g.add_node({8,  "SMS_GATEWAY",                NodeType::EXTERNAL,  0.80, 18.0, true });
    g.add_node({9,  "INVENTORY_SERVICE",          NodeType::INTERNAL, 0.94, 6.0,  false});
    g.add_node({10, "INVENTORY_DB",               NodeType::DATABASE,  0.96, 9.0,  false});
    g.add_node({11, "ORDER_SERVICE",              NodeType::INTERNAL, 0.97, 7.0,  false});
    g.add_node({12, "ORDER_DB",                   NodeType::DATABASE,  0.98, 8.0,  false});
    g.add_node({13, "AUDIT_SERVICE",              NodeType::INTERNAL, 0.99, 3.0,  false});
    g.add_node({14, "PAYMENT_GATEWAY_SECONDARY",  NodeType::EXTERNAL,  0.88, 25.0, true });

    // Primary transaction flow edges (src, dst, weight_ms, capacity_rps, is_fallback)
    // API Gateway → downstream services
    g.add_edge(0,  1,  3.0,  5000.0, false);  // API_GATEWAY → AUTH_SERVICE
    g.add_edge(0,  11, 5.0,  3000.0, false);  // API_GATEWAY → ORDER_SERVICE
    g.add_edge(0,  13, 1.0, 10000.0, false);  // API_GATEWAY → AUDIT_SERVICE

    // Auth flow
    g.add_edge(1,  2,  4.0,  2000.0, false);  // AUTH_SERVICE → USER_DB
    g.add_edge(1,  3,  6.0,  1500.0, false);  // AUTH_SERVICE → PAYMENT_SERVICE
    g.add_edge(1,  9,  5.0,  2500.0, false);  // AUTH_SERVICE → INVENTORY_SERVICE

    // Payment flow
    g.add_edge(3,  4,  8.0,  1000.0, false);  // PAYMENT_SERVICE → PAYMENT_DB
    g.add_edge(3,  5,  10.0,  800.0, false);  // PAYMENT_SERVICE → FRAUD_DETECTOR
    g.add_edge(3,  6,  3.0,  1200.0, false);  // PAYMENT_SERVICE → NOTIFICATION_SERVICE
    g.add_edge(3,  13, 2.0,  5000.0, false);  // PAYMENT_SERVICE → AUDIT_SERVICE

    // Fallback: PAYMENT_SERVICE can route via PAYMENT_GATEWAY_SECONDARY
    g.add_edge(3,  14, 15.0,  500.0, true );  // PAYMENT_SERVICE → PAYMENT_GATEWAY_SEC (fallback)
    g.add_edge(14, 4,  12.0,  500.0, true );  // PAYMENT_GATEWAY_SEC → PAYMENT_DB (fallback)

    // Fraud detector → audit
    g.add_edge(5,  13, 2.0,  3000.0, false);  // FRAUD_DETECTOR → AUDIT_SERVICE

    // Notification flow
    g.add_edge(6,  7,  5.0,  2000.0, false);  // NOTIFICATION_SERVICE → EMAIL_GATEWAY
    g.add_edge(6,  8,  8.0,  1500.0, false);  // NOTIFICATION_SERVICE → SMS_GATEWAY

    // Fallback: EMAIL_GATEWAY can failover to SMS
    g.add_edge(7,  8,  3.0,   500.0, true );  // EMAIL_GATEWAY → SMS_GATEWAY (fallback)

    // Inventory flow
    g.add_edge(9,  10, 6.0,  2000.0, false);  // INVENTORY_SERVICE → INVENTORY_DB
    g.add_edge(9,  11, 4.0,  1500.0, false);  // INVENTORY_SERVICE → ORDER_SERVICE

    // Order flow
    g.add_edge(11, 12, 5.0,  1500.0, false);  // ORDER_SERVICE → ORDER_DB
    g.add_edge(11, 3,  7.0,   800.0, false);  // ORDER_SERVICE → PAYMENT_SERVICE
    g.add_edge(11, 13, 2.0,  4000.0, false);  // ORDER_SERVICE → AUDIT_SERVICE
    g.add_edge(11, 6,  3.0,  1000.0, false);  // ORDER_SERVICE → NOTIFICATION_SERVICE

    return g;
}

// ============================================================
// main
// ============================================================

int main() {
    std::cout << std::fixed << std::setprecision(3);

    std::cout << "╔══════════════════════════════════════════════════════════════════╗\n";
    std::cout << "║   TXN-GRAPH-ENGINE  —  Distributed Transaction Analysis Demo    ║\n";
    std::cout << "║   C++17  •  Graph Algorithms  •  Failure Simulation             ║\n";
    std::cout << "╚══════════════════════════════════════════════════════════════════╝\n";

    Graph g = build_txn_graph();
    std::cout << "\nGraph: " << g.num_nodes() << " nodes, "
              << g.num_edges() << " edges\n";

    // Print node listing
    subsection("Service Nodes");
    for (int i = 0; i < g.num_nodes(); ++i) {
        const auto& n = g.get_node(i);
        std::cout << "  [" << std::setw(2) << n.id << "] "
                  << std::left << std::setw(30) << n.name
                  << std::right
                  << node_type_str(n.type)
                  << "  health=" << std::setprecision(2) << n.health
                  << "  latency=" << std::setprecision(1) << n.base_latency_ms << "ms"
                  << (n.has_fallback ? "  [HAS_FALLBACK]" : "")
                  << "\n";
    }

    // ================================================================
    //  1. BFS / DFS traversal from API_GATEWAY (node 0)
    // ================================================================
    section("1. BFS / DFS Traversal from API_GATEWAY");

    auto bfs_order = bfs(g, 0);
    print_node_list(g, bfs_order, "BFS order");

    auto dfs_order = dfs(g, 0);
    print_node_list(g, dfs_order, "DFS order");

    // Multi-source BFS from PAYMENT_SERVICE (3) and ORDER_SERVICE (11)
    auto ms_dist = multi_source_bfs(g, {3, 11});
    subsection("Multi-source BFS from PAYMENT_SERVICE + ORDER_SERVICE");
    for (auto& [node, dist] : ms_dist)
        std::cout << "  " << std::setw(28) << std::left
                  << g.get_node(node).name
                  << " → distance=" << dist << "\n";

    // BFS with callback: print first 5 nodes discovered
    subsection("BFS with callback (depth-tagged, first 7 nodes)");
    int cb_count = 0;
    bfs_with_callback(g, 0, [&](int node, int depth) -> bool {
        std::cout << "  depth=" << depth
                  << "  " << g.get_node(node).name << "\n";
        return (++cb_count < 7);
    });

    // ================================================================
    //  2. Dijkstra from API_GATEWAY
    // ================================================================
    section("2. Dijkstra Shortest Path (API_GATEWAY → all)");

    auto dijk = dijkstra(g, 0);
    for (int v = 0; v < g.num_nodes(); ++v) {
        if (dijk.dist[v] == INF) {
            std::cout << "  " << std::setw(30) << std::left
                      << g.get_node(v).name << " → UNREACHABLE\n";
        } else {
            auto path = dijk.path_to(v);
            std::cout << "  " << std::setw(30) << std::left
                      << g.get_node(v).name
                      << " dist=" << std::setw(7) << std::right
                      << std::setprecision(1) << dijk.dist[v] << "ms"
                      << "  path=[";
            for (int i = 0; i < static_cast<int>(path.size()); ++i) {
                if (i) std::cout << "→";
                std::cout << g.get_node(path[i]).name;
            }
            std::cout << "]\n";
        }
    }

    // Bellman-Ford from AUTH_SERVICE
    subsection("Bellman-Ford from AUTH_SERVICE (node 1)");
    auto bf = bellman_ford(g, 1);
    std::cout << "  Negative cycle detected: "
              << (bf.has_negative_cycle ? "YES" : "NO") << "\n";
    for (int v = 0; v < g.num_nodes(); ++v) {
        if (bf.dist[v] != INF)
            std::cout << "  " << std::setw(30) << std::left
                      << g.get_node(v).name
                      << " dist=" << std::setprecision(1) << bf.dist[v] << "ms\n";
    }

    // ================================================================
    //  3. Floyd-Warshall all-pairs (show excerpt)
    // ================================================================
    section("3. Floyd-Warshall All-Pairs Shortest Paths (excerpt)");

    auto fw = floyd_warshall(g);
    std::cout << "  Negative cycle detected: "
              << (fw.has_negative_cycle ? "YES" : "NO") << "\n";
    // Print distance from API_GATEWAY to all nodes
    int src_fw = 0;
    for (int v = 0; v < g.num_nodes(); ++v) {
        double d = fw.dist[src_fw][v];
        std::cout << "  " << g.get_node(src_fw).name
                  << " -> " << std::setw(30) << std::left
                  << g.get_node(v).name
                  << std::right << " dist="
                  << (d == INF ? std::string("INF") : std::to_string(static_cast<int>(d)) + "ms")
                  << "\n";
    }

    // Johnson's all-pairs
    subsection("Johnson's All-Pairs (API_GATEWAY row)");
    auto jh = johnsons(g);
    std::cout << "  Negative cycle: " << (jh.has_negative_cycle ? "YES" : "NO") << "\n";
    if (!jh.has_negative_cycle) {
        for (int v = 0; v < g.num_nodes(); ++v) {
            double d = jh.dist[0][v];
            std::cout << "  API_GATEWAY -> " << std::setw(30) << std::left
                      << g.get_node(v).name
                      << std::right << " = "
                      << (d >= INF/2 ? std::string("INF") : std::to_string(static_cast<int>(d)) + "ms")
                      << "\n";
        }
    }

    // ================================================================
    //  4. Topological Sort + Dependency Levels
    // ================================================================
    section("4. Topological Sort + Dependency Levels");

    auto kahn_order = kahn_toposort(g);
    if (kahn_order.empty()) {
        std::cout << "  Kahn's toposort: CYCLE DETECTED — not a DAG\n";
    } else {
        print_node_list(g, kahn_order, "  Kahn's order");
    }

    auto dfs_order2 = dfs_toposort(g);
    if (dfs_order2.empty()) {
        std::cout << "  DFS toposort: CYCLE DETECTED — not a DAG\n";
    } else {
        print_node_list(g, dfs_order2, "  DFS order  ");
    }

    auto levels = toposort_levels(g);
    if (levels.empty()) {
        std::cout << "  Level partition: CYCLE DETECTED\n";
    } else {
        std::cout << "  Dependency levels (parallel scheduling waves):\n";
        for (int l = 0; l < static_cast<int>(levels.size()); ++l) {
            std::cout << "  Level " << l << ": [";
            for (int i = 0; i < static_cast<int>(levels[l].size()); ++i) {
                if (i) std::cout << ", ";
                std::cout << g.get_node(levels[l][i]).name;
            }
            std::cout << "]\n";
        }
    }

    // ================================================================
    //  5. Tarjan SCC + Kosaraju SCC
    // ================================================================
    section("5. Strongly Connected Components");

    subsection("Tarjan's SCC");
    auto tarjan_sccs = tarjan_scc(g);
    for (int i = 0; i < static_cast<int>(tarjan_sccs.size()); ++i) {
        std::cout << "  SCC " << std::setw(2) << i << ": [";
        for (int j = 0; j < static_cast<int>(tarjan_sccs[i].size()); ++j) {
            if (j) std::cout << ", ";
            std::cout << g.get_node(tarjan_sccs[i][j]).name;
        }
        std::cout << "]" << (tarjan_sccs[i].size() > 1 ? " ← CYCLE" : "") << "\n";
    }

    subsection("Kosaraju's SCC");
    auto kosar_sccs = kosaraju_scc(g);
    for (int i = 0; i < static_cast<int>(kosar_sccs.size()); ++i) {
        std::cout << "  SCC " << std::setw(2) << i << ": [";
        for (int j = 0; j < static_cast<int>(kosar_sccs[i].size()); ++j) {
            if (j) std::cout << ", ";
            std::cout << g.get_node(kosar_sccs[i][j]).name;
        }
        std::cout << "]\n";
    }

    subsection("Condensation Graph (meta-nodes)");
    auto [cond_g, comp_id] = condensation_graph(g, tarjan_sccs);
    std::cout << "  Condensation: " << cond_g.num_nodes()
              << " meta-nodes, " << cond_g.num_edges() << " meta-edges\n";

    // ================================================================
    //  6. MST — Kruskal + Prim + Borůvka
    // ================================================================
    section("6. Minimum Spanning Tree");

    subsection("Kruskal's MST");
    auto kruskal_edges = kruskal_mst(g);
    print_edge_list(g, kruskal_edges, "  Kruskal MST");
    double kruskal_total = 0.0;
    for (const auto& e : kruskal_edges) kruskal_total += e.weight;
    std::cout << "  Total MST weight: " << kruskal_total << "ms\n";

    subsection("Prim's MST (from API_GATEWAY)");
    auto prim_edges = prim_mst(g, 0);
    print_edge_list(g, prim_edges, "  Prim MST");
    double prim_total = 0.0;
    for (const auto& e : prim_edges) prim_total += e.weight;
    std::cout << "  Total MST weight: " << prim_total << "ms\n";

    subsection("Borůvka's MST");
    auto boruvka_edges = boruvka_mst(g);
    print_edge_list(g, boruvka_edges, "  Borůvka MST");
    double boruvka_total = 0.0;
    for (const auto& e : boruvka_edges) boruvka_total += e.weight;
    std::cout << "  Total MST weight: " << boruvka_total << "ms\n";

    // ================================================================
    //  7. Max Flow — Edmonds-Karp + Dinic
    // ================================================================
    section("7. Maximum Flow (API_GATEWAY → AUDIT_SERVICE)");

    constexpr int FLOW_SRC  = 0;  // API_GATEWAY
    constexpr int FLOW_SINK = 13; // AUDIT_SERVICE

    subsection("Edmonds-Karp");
    auto ek = edmonds_karp(g, FLOW_SRC, FLOW_SINK);
    std::cout << "  Max flow (capacity): " << ek.max_flow << " req/s\n";
    auto ek_cut = min_cut_edges(ek, g);
    std::cout << "  Min-cut edges (" << ek_cut.size() << "):\n";
    for (auto [u, v] : ek_cut)
        std::cout << "    " << g.get_node(u).name << " -> " << g.get_node(v).name << "\n";

    subsection("Dinic's Algorithm");
    auto din = dinic(g, FLOW_SRC, FLOW_SINK);
    std::cout << "  Max flow (capacity): " << din.max_flow << " req/s\n";
    auto din_cut = min_cut_edges(din, g);
    std::cout << "  Min-cut edges (" << din_cut.size() << "):\n";
    for (auto [u, v] : din_cut)
        std::cout << "    " << g.get_node(u).name << " -> " << g.get_node(v).name << "\n";

    // ================================================================
    //  8. Bridges + Articulation Points
    // ================================================================
    section("8. Bridges & Articulation Points");

    auto bridges = find_bridges(g);
    std::cout << "  Bridges (" << bridges.size() << "):\n";
    for (auto [u, v] : bridges)
        std::cout << "    " << g.get_node(u).name << " — " << g.get_node(v).name << "\n";

    auto aps = find_articulation_points(g);
    std::cout << "  Articulation points (" << aps.size() << "):\n";
    for (int v : aps)
        std::cout << "    " << g.get_node(v).name << "\n";

    auto [bip, bip_color] = is_bipartite(g);
    std::cout << "  Bipartite: " << (bip ? "YES" : "NO") << "\n";

    // ================================================================
    //  9. Cycle Detection
    // ================================================================
    section("9. Cycle Detection");

    bool directed_cycle = has_cycle_directed(g);
    std::cout << "  Directed cycle (DFS coloring): "
              << (directed_cycle ? "YES" : "NO") << "\n";

    bool undirected_cycle = has_cycle_undirected(g);
    std::cout << "  Undirected cycle (Union-Find): "
              << (undirected_cycle ? "YES" : "NO") << "\n";

    auto all_cycles = find_all_cycles_directed(g);
    std::cout << "  All directed simple cycles (" << all_cycles.size() << "):\n";
    for (const auto& cycle : all_cycles) {
        std::cout << "    [";
        for (int i = 0; i < static_cast<int>(cycle.size()); ++i) {
            if (i) std::cout << " → ";
            std::cout << g.get_node(cycle[i]).name;
        }
        std::cout << " → " << g.get_node(cycle[0]).name << "]\n";
    }

    // ================================================================
    // 10. Centrality — Betweenness + PageRank + Degree + Closeness
    // ================================================================
    section("10. Centrality Metrics");

    subsection("Degree Centrality");
    auto deg_cent = degree_centrality(g);
    // Sort by centrality descending
    std::vector<std::pair<double,int>> sorted_deg;
    for (auto& [v, c] : deg_cent) sorted_deg.push_back({c, v});
    std::sort(sorted_deg.rbegin(), sorted_deg.rend());
    for (auto& [c, v] : sorted_deg)
        std::cout << "  " << std::setw(30) << std::left
                  << g.get_node(v).name
                  << " centrality=" << std::setprecision(4) << c << "\n";

    subsection("Betweenness Centrality (top 5)");
    auto btw = betweenness_centrality(g);
    std::vector<std::pair<double,int>> sorted_btw;
    for (auto& [v, c] : btw) sorted_btw.push_back({c, v});
    std::sort(sorted_btw.rbegin(), sorted_btw.rend());
    for (int i = 0; i < std::min(5, static_cast<int>(sorted_btw.size())); ++i)
        std::cout << "  " << std::setw(30) << std::left
                  << g.get_node(sorted_btw[i].second).name
                  << " betweenness=" << std::setprecision(6) << sorted_btw[i].first << "\n";

    subsection("PageRank (damping=0.85, top 5)");
    auto pr = pagerank(g, 0.85, 100);
    std::vector<std::pair<double,int>> sorted_pr;
    for (auto& [v, r] : pr) sorted_pr.push_back({r, v});
    std::sort(sorted_pr.rbegin(), sorted_pr.rend());
    for (int i = 0; i < std::min(5, static_cast<int>(sorted_pr.size())); ++i)
        std::cout << "  " << std::setw(30) << std::left
                  << g.get_node(sorted_pr[i].second).name
                  << " rank=" << std::setprecision(6) << sorted_pr[i].first << "\n";

    subsection("Closeness Centrality (top 5)");
    auto cl = closeness_centrality(g);
    std::vector<std::pair<double,int>> sorted_cl;
    for (auto& [v, c] : cl) sorted_cl.push_back({c, v});
    std::sort(sorted_cl.rbegin(), sorted_cl.rend());
    for (int i = 0; i < std::min(5, static_cast<int>(sorted_cl.size())); ++i)
        std::cout << "  " << std::setw(30) << std::left
                  << g.get_node(sorted_cl[i].second).name
                  << " closeness=" << std::setprecision(6) << sorted_cl[i].first << "\n";

    // ================================================================
    // 11. Critical Path (CPM / PERT)
    // ================================================================
    section("11. Critical Path Analysis (CPM / PERT)");

    try {
        auto cp = compute_critical_path(g);
        std::cout << "  Project duration: " << std::setprecision(2)
                  << cp.project_duration << " ms\n";
        print_node_list(g, cp.critical_path, "  Critical path nodes");

        std::cout << "\n  Per-node timing (EST / EFT / LST / LFT / Slack):\n";
        std::cout << "  " << std::setw(28) << std::left << "Node"
                  << std::right
                  << std::setw(8)  << "EST"
                  << std::setw(8)  << "EFT"
                  << std::setw(8)  << "LST"
                  << std::setw(8)  << "LFT"
                  << std::setw(8)  << "Slack"
                  << "  CP?\n";
        std::cout << "  " << std::string(70, '-') << "\n";
        for (int v = 0; v < g.num_nodes(); ++v) {
            const auto& t = cp.timings[v];
            std::cout << "  " << std::setw(28) << std::left << g.get_node(v).name
                      << std::right << std::setprecision(1)
                      << std::setw(8) << t.EST
                      << std::setw(8) << t.EFT
                      << std::setw(8) << t.LST
                      << std::setw(8) << t.LFT
                      << std::setw(8) << t.slack
                      << (t.on_critical_path ? "  *** CP ***" : "")
                      << "\n";
        }
    } catch (const std::exception& ex) {
        std::cout << "  CPM: " << ex.what() << "\n";
    }

    // ================================================================
    // 12. Transitive Closure
    // ================================================================
    section("12. Transitive Closure (Warshall's)");

    auto tc = transitive_closure(g);
    // Print reachability from API_GATEWAY
    std::cout << "  From API_GATEWAY can reach:\n";
    for (int v = 0; v < g.num_nodes(); ++v) {
        if (tc[0][v] && v != 0)
            std::cout << "    " << g.get_node(v).name << "\n";
    }

    // Spot checks
    std::cout << "\n  Spot checks (can_reach via BFS):\n";
    std::vector<std::pair<int,int>> checks = {{0,13},{0,4},{0,2},{5,12},{7,4}};
    for (auto [u, v] : checks) {
        std::cout << "  " << g.get_node(u).name << " → " << g.get_node(v).name
                  << ": " << (can_reach(g, u, v) ? "YES" : "NO") << "\n";
    }

    subsection("Reachable set from PAYMENT_SERVICE");
    auto reach_set = reachable_set(g, 3);
    std::cout << "  Nodes reachable from PAYMENT_SERVICE: [";
    bool first = true;
    for (int v : reach_set) {
        if (!first) std::cout << ", ";
        std::cout << g.get_node(v).name;
        first = false;
    }
    std::cout << "]\n";

    // ================================================================
    // 13. Graph Coloring — Greedy + DSatur
    // ================================================================
    section("13. Graph Coloring");

    subsection("Greedy Coloring (Welsh-Powell)");
    auto greedy_col = greedy_coloring(g);
    int greedy_num = 0;
    for (auto& [v, c] : greedy_col) greedy_num = std::max(greedy_num, c + 1);
    std::cout << "  Colors used: " << greedy_num << "\n";
    // Group by color
    std::vector<std::vector<std::string>> color_groups_g(greedy_num);
    for (auto& [v, c] : greedy_col) color_groups_g[c].push_back(g.get_node(v).name);
    for (int c = 0; c < greedy_num; ++c) {
        std::cout << "  Color " << c << ": [";
        for (int i = 0; i < static_cast<int>(color_groups_g[c].size()); ++i) {
            if (i) std::cout << ", ";
            std::cout << color_groups_g[c][i];
        }
        std::cout << "]\n";
    }

    subsection("DSatur Coloring");
    auto dsatur_col = dsatur_coloring(g);
    int dsatur_num = chromatic_number_estimate(g);
    std::cout << "  Chromatic number estimate: " << dsatur_num << "\n";
    std::vector<std::vector<std::string>> color_groups_d(dsatur_num);
    for (auto& [v, c] : dsatur_col)
        if (c < dsatur_num) color_groups_d[c].push_back(g.get_node(v).name);
    for (int c = 0; c < dsatur_num; ++c) {
        std::cout << "  Color " << c << ": [";
        for (int i = 0; i < static_cast<int>(color_groups_d[c].size()); ++i) {
            if (i) std::cout << ", ";
            std::cout << color_groups_d[c][i];
        }
        std::cout << "]\n";
    }

    // ================================================================
    // 14. Multi-Component Partitioning
    // ================================================================
    section("14. Graph Partitioning for Parallel Execution");

    subsection("WCC-based partitioning (independent components)");
    auto wcc_parts = partition_for_parallel(g);
    for (int i = 0; i < static_cast<int>(wcc_parts.size()); ++i) {
        std::cout << "  Partition " << i << " (" << wcc_parts[i].size() << " nodes): [";
        for (int j = 0; j < static_cast<int>(wcc_parts[i].size()); ++j) {
            if (j) std::cout << ", ";
            std::cout << g.get_node(wcc_parts[i][j]).name;
        }
        std::cout << "]\n";
    }

    subsection("Topological wave partition (parallel scheduling layers)");
    auto wave_parts = topological_wave_partition(g);
    if (wave_parts.empty()) {
        std::cout << "  Wave partition: graph has a cycle — not applicable.\n";
    } else {
        for (int i = 0; i < static_cast<int>(wave_parts.size()); ++i) {
            std::cout << "  Wave " << i << ": [";
            for (int j = 0; j < static_cast<int>(wave_parts[i].size()); ++j) {
                if (j) std::cout << ", ";
                std::cout << g.get_node(wave_parts[i][j]).name;
            }
            std::cout << "]\n";
        }
    }

    subsection("SCC-based partition (DAG layers on condensation graph)");
    auto scc_parts = partition_by_scc(g);
    for (int i = 0; i < static_cast<int>(scc_parts.size()); ++i) {
        std::cout << "  Layer " << i << ": [";
        for (int j = 0; j < static_cast<int>(scc_parts[i].size()); ++j) {
            if (j) std::cout << ", ";
            std::cout << g.get_node(scc_parts[i][j]).name;
        }
        std::cout << "]\n";
    }

    // ================================================================
    // 15. Failure Simulation: PAYMENT_SERVICE (node 3) fails
    // ================================================================
    section("15. Failure Simulation: PAYMENT_SERVICE Fails");

    auto fail_result = simulate_failure(g, 3, 0.05);
    std::cout << "  Failed node: PAYMENT_SERVICE\n";
    std::cout << "  Cascade depth: " << fail_result.cascade_depth << "\n";
    std::cout << "  Total impact score: " << std::setprecision(3)
              << fail_result.total_impact_score << "\n";
    std::cout << "  Affected nodes (" << fail_result.affected_nodes.size() << "):\n";
    for (int v : fail_result.affected_nodes) {
        std::cout << "    " << std::setw(30) << std::left
                  << g.get_node(v).name
                  << " P(fail)=" << std::setprecision(4)
                  << fail_result.failure_prob.at(v) << "\n";
    }

    subsection("Multi-failure: PAYMENT_SERVICE + FRAUD_DETECTOR");
    auto multi_fail = simulate_multi_failure(g, {3, 5}, 0.05);
    std::cout << "  Affected nodes (" << multi_fail.affected_nodes.size() << "):\n";
    for (int v : multi_fail.affected_nodes) {
        std::cout << "    " << std::setw(30) << std::left
                  << g.get_node(v).name
                  << " P(fail)=" << std::setprecision(4)
                  << multi_fail.failure_prob.at(v) << "\n";
    }

    // ================================================================
    // 16. Blast Radius Report
    // ================================================================
    section("16. Blast Radius Report: PAYMENT_SERVICE Fails");

    auto br = compute_blast_radius(g, 3, 0.05);
    std::cout << "  Failed node:          PAYMENT_SERVICE\n";
    std::cout << "  Affected count:       " << br.affected_count
              << " / " << br.total_nodes << " services\n";
    std::cout << "  Severity:             " << severity_str(br.severity) << "\n";
    std::cout << "  Gateways affected:    " << br.affected_gateways << "\n";
    std::cout << "  Databases affected:   " << br.affected_databases << "\n";
    std::cout << "  Internals affected:   " << br.affected_internals << "\n";
    std::cout << "  Externals affected:   " << br.affected_externals << "\n";
    std::cout << "  Revenue impact:       $"
              << std::setprecision(2) << br.estimated_revenue_impact_usd_hr
              << " / hr\n";
    if (!br.affected_critical_services.empty()) {
        std::cout << "  Critical services hit: [";
        for (int i = 0; i < static_cast<int>(br.affected_critical_services.size()); ++i) {
            if (i) std::cout << ", ";
            std::cout << br.affected_critical_services[i];
        }
        std::cout << "]\n";
    }

    // ================================================================
    // 17. Latency Analysis
    // ================================================================
    section("17. Latency Analysis");

    subsection("Path latency for manually specified path");
    std::vector<int> sample_path = {0, 1, 3, 4}; // API_GW → AUTH → PAYMENT → PAYMENT_DB
    try {
        double lat = compute_path_latency(g, sample_path);
        std::cout << "  Path: API_GATEWAY→AUTH_SERVICE→PAYMENT_SERVICE→PAYMENT_DB\n";
        std::cout << "  Total latency: " << std::setprecision(1) << lat << " ms\n";
    } catch (const std::exception& ex) {
        std::cout << "  " << ex.what() << "\n";
    }

    subsection("Critical path latency (API_GATEWAY → PAYMENT_DB)");
    auto [cp_path, cp_lat] = critical_path_latency(g, 0, 4);
    if (cp_path.empty()) {
        std::cout << "  No path found.\n";
    } else {
        std::cout << "  Critical path: [";
        for (int i = 0; i < static_cast<int>(cp_path.size()); ++i) {
            if (i) std::cout << " → ";
            std::cout << g.get_node(cp_path[i]).name;
        }
        std::cout << "]\n";
        std::cout << "  Latency: " << std::setprecision(2) << cp_lat << " ms\n";
    }

    subsection("p99 Latency Estimate (Monte-Carlo, API_GATEWAY → AUDIT_SERVICE)");
    double p99 = p99_latency_estimate(g, 0, 13);
    std::cout << "  p99 end-to-end latency: " << std::setprecision(2) << p99 << " ms\n";
    std::cout << "  (Based on " << LATENCY_MC_SAMPLES << " Monte-Carlo samples,\n"
              << "   log-normal jitter CV=" << JITTER_CV << ")\n";

    // ================================================================
    // 18. Yen's k-shortest fallback paths
    // ================================================================
    section("18. Yen's k-Shortest Fallback Paths");

    subsection("Fallback path detection: PAYMENT_SERVICE → PAYMENT_DB");
    bool fb = has_fallback_path(g, 3, 4);
    std::cout << "  Has fallback path (3→4): " << (fb ? "YES" : "NO") << "\n";

    subsection("Yen's top-5 shortest paths: API_GATEWAY → AUDIT_SERVICE");
    auto k_paths = yen_k_shortest_paths(g, 0, 13, 5);
    std::cout << "  Found " << k_paths.size() << " shortest paths:\n";
    for (int i = 0; i < static_cast<int>(k_paths.size()); ++i) {
        const auto& [path, cost] = k_paths[i];
        std::cout << "  Path " << (i+1) << " (cost=" << std::setprecision(1) << cost << "ms): [";
        for (int j = 0; j < static_cast<int>(path.size()); ++j) {
            if (j) std::cout << " → ";
            std::cout << g.get_node(path[j]).name;
        }
        std::cout << "]\n";
    }

    subsection("Resilience score: API_GATEWAY → AUDIT_SERVICE (k=5 paths)");
    double res = resilience_score(g, 0, 13, 5);
    std::cout << "  Resilience score: " << std::setprecision(4) << res
              << "  (0=fragile, 1=fully resilient)\n";

    subsection("Resilience score: PAYMENT_SERVICE → PAYMENT_DB");
    double res2 = resilience_score(g, 3, 4, 5);
    std::cout << "  Resilience score: " << std::setprecision(4) << res2 << "\n";

    // ================================================================
    // 19. Parallel Analysis Across Partitions
    // ================================================================
    section("19. Parallel Analysis Across Partitions");

    // Use WCC partitions for the parallel run
    std::cout << "  Using " << wcc_parts.size() << " partition(s), "
              << std::thread::hardware_concurrency() << " HW threads available.\n";

    auto parallel_result = run_parallel_analysis(g, wcc_parts, 4);

    std::cout << "\n  Parallel analysis complete:\n";
    std::cout << "  Total affected nodes (sum across all nodes' blast radii): "
              << parallel_result.total_affected_nodes << "\n";
    std::cout << "  Max single-node revenue impact: $"
              << std::setprecision(2) << parallel_result.max_revenue_impact << "/hr\n";
    std::cout << "  Most critical partition: "
              << parallel_result.most_critical_partition << "\n";

    // Detail for each partition
    for (const auto& pr : parallel_result.partition_results) {
        std::cout << "\n  Partition " << pr.partition_id
                  << " (" << pr.nodes.size() << " nodes):\n";
        for (int i = 0; i < static_cast<int>(pr.nodes.size()); ++i) {
            const auto& blast = pr.blast_reports[i];
            const auto& fail  = pr.failure_results[i];
            std::cout << "    Node=" << std::setw(30) << std::left
                      << g.get_node(pr.nodes[i]).name
                      << std::right
                      << "  cascade_depth=" << fail.cascade_depth
                      << "  affected=" << blast.affected_count
                      << "  severity=" << severity_str(blast.severity)
                      << "  revenue=$" << std::setprecision(0)
                      << blast.estimated_revenue_impact_usd_hr << "/hr\n";
        }
    }

    // ================================================================
    // Summary
    // ================================================================
    section("ANALYSIS COMPLETE");
    std::cout << "\n  Engine exercised:\n"
              << "    BFS / DFS / Multi-source BFS / BFS-with-callback\n"
              << "    Dijkstra / Bellman-Ford / Floyd-Warshall / Johnson's\n"
              << "    Kahn Toposort / DFS Toposort / Level partition\n"
              << "    Tarjan SCC / Kosaraju SCC / Condensation graph\n"
              << "    Kruskal MST / Prim MST / Borůvka MST\n"
              << "    Edmonds-Karp max flow / Dinic max flow / Min-cut\n"
              << "    WCC / Bridges / Articulation points / Bipartite check\n"
              << "    Directed cycle / Undirected cycle / Johnson's all-cycles\n"
              << "    Degree / Betweenness / PageRank / Closeness centrality\n"
              << "    CPM/PERT critical path\n"
              << "    Warshall transitive closure / can_reach / reachable_set\n"
              << "    Greedy coloring / DSatur coloring\n"
              << "    WCC partition / Wave partition / SCC partition\n"
              << "    Failure propagation / Multi-failure simulation\n"
              << "    Blast radius classification\n"
              << "    Path latency / Critical-path latency / p99 MC estimate\n"
              << "    Fallback path detection / Yen's k-shortest / Resilience score\n"
              << "    C++17 thread pool + parallel partition executor\n";

    return 0;
}
