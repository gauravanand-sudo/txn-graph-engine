#pragma once

#include <string>
#include <string_view>
#include <vector>
#include <stdexcept>
#include <algorithm>
#include <optional>
#include <unordered_map>

// ============================================================
// Graph.hpp — Core directed weighted graph for transaction
//             workflow dependency modelling.
//
// Design notes:
//   • Adjacency-list representation keeps neighbour iteration
//     O(degree) and is cache-friendly for sparse service graphs.
//   • Every node carries domain metadata (health, latency, etc.)
//     so analysis algorithms can incorporate real-world signals.
// ============================================================

namespace txn {

// ----------------------------------------------------------
// ServiceNode — vertex in the transaction workflow graph
// ----------------------------------------------------------

enum class NodeType : uint8_t {
    INTERNAL  = 0,   // in-house micro-service
    EXTERNAL  = 1,   // third-party API
    GATEWAY   = 2,   // API / load-balancer gateway
    DATABASE  = 3    // data-store
};

inline std::string_view node_type_str(NodeType t) {
    switch (t) {
        case NodeType::INTERNAL:  return "INTERNAL";
        case NodeType::EXTERNAL:  return "EXTERNAL";
        case NodeType::GATEWAY:   return "GATEWAY";
        case NodeType::DATABASE:  return "DATABASE";
    }
    return "UNKNOWN";
}

struct ServiceNode {
    int         id;                  ///< unique numeric identifier (0-based index)
    std::string name;                ///< human-readable service name
    NodeType    type;                ///< service category
    double      health;              ///< 0.0 (down) – 1.0 (fully healthy)
    double      base_latency_ms;     ///< per-call baseline latency in milliseconds
    bool        has_fallback;        ///< true if a fallback path/service exists

    ServiceNode() = default;
    ServiceNode(int id_, std::string name_, NodeType type_,
                double health_ = 1.0, double latency_ = 5.0,
                bool fallback_ = false)
        : id(id_), name(std::move(name_)), type(type_),
          health(health_), base_latency_ms(latency_),
          has_fallback(fallback_) {}
};

// ----------------------------------------------------------
// ServiceEdge — directed weighted edge (dependency / call)
// ----------------------------------------------------------

struct ServiceEdge {
    int    src;           ///< source node id
    int    dst;           ///< destination node id
    double weight;        ///< effective edge latency/cost in ms
    double capacity;      ///< maximum throughput (requests/second)
    bool   is_fallback;   ///< true → only used when primary path is down

    ServiceEdge() = default;
    ServiceEdge(int src_, int dst_, double weight_ = 1.0,
                double cap_ = 1000.0, bool fallback_ = false)
        : src(src_), dst(dst_), weight(weight_),
          capacity(cap_), is_fallback(fallback_) {}
};

// ----------------------------------------------------------
// Graph — directed, weighted adjacency-list graph
// ----------------------------------------------------------

class Graph {
public:
    // ---- construction -----------------------------------------------

    /// Construct a graph, optionally pre-reserving capacity for n nodes.
    /// No nodes are added — call add_node() to populate.
    explicit Graph(int n = 0) {
        adj_.reserve(n);
        nodes_.reserve(n);
    }

    /// Add a new node (appends; id must equal current num_nodes()).
    void add_node(const ServiceNode& node) {
        if (node.id != static_cast<int>(nodes_.size()))
            throw std::invalid_argument("Node id must equal current num_nodes()");
        nodes_.push_back(node);
        adj_.emplace_back();
    }

    /// Add a directed edge src → dst.
    void add_edge(int src, int dst, double weight = 1.0,
                  double capacity = 1000.0, bool is_fallback = false) {
        range_check(src);
        range_check(dst);
        adj_[src].emplace_back(src, dst, weight, capacity, is_fallback);
        ++edge_count_;
    }

    /// Convenience overload accepting a pre-built edge struct.
    void add_edge(const ServiceEdge& e) {
        add_edge(e.src, e.dst, e.weight, e.capacity, e.is_fallback);
    }

    // ---- accessors --------------------------------------------------

    [[nodiscard]] int num_nodes() const noexcept {
        return static_cast<int>(nodes_.size());
    }

    [[nodiscard]] int num_edges() const noexcept {
        return edge_count_;
    }

    [[nodiscard]] const ServiceNode& get_node(int id) const {
        range_check(id);
        return nodes_[id];
    }

    [[nodiscard]] ServiceNode& get_node(int id) {
        range_check(id);
        return nodes_[id];
    }

    [[nodiscard]] const std::vector<ServiceEdge>& neighbors(int id) const {
        range_check(id);
        return adj_[id];
    }

    [[nodiscard]] const std::vector<std::vector<ServiceEdge>>& adj() const noexcept {
        return adj_;
    }

    [[nodiscard]] const std::vector<ServiceNode>& nodes() const noexcept {
        return nodes_;
    }

    // ---- graph transformations -------------------------------------

    /// Returns a new graph with all edge directions reversed.
    /// Useful for Kosaraju's SCC, reachability queries, etc.
    [[nodiscard]] Graph reverse_graph() const {
        const int N = num_nodes();
        Graph rev;
        rev.nodes_ = nodes_;
        rev.adj_.resize(N);
        for (int u = 0; u < N; ++u) {
            for (const auto& e : adj_[u]) {
                rev.adj_[e.dst].emplace_back(e.dst, e.src, e.weight,
                                              e.capacity, e.is_fallback);
                ++rev.edge_count_;
            }
        }
        return rev;
    }

    /// Returns an undirected version: for every edge u→v also add v→u.
    [[nodiscard]] Graph to_undirected() const {
        const int N = num_nodes();
        Graph und;
        und.nodes_ = nodes_;
        und.adj_.resize(N);
        for (int u = 0; u < N; ++u) {
            for (const auto& e : adj_[u]) {
                und.adj_[u].emplace_back(e);
                und.adj_[e.dst].emplace_back(e.dst, u, e.weight,
                                              e.capacity, e.is_fallback);
                und.edge_count_ += 2;
            }
        }
        return und;
    }

    /// Collect all edges as a flat list (useful for MST algorithms).
    [[nodiscard]] std::vector<ServiceEdge> all_edges() const {
        std::vector<ServiceEdge> out;
        out.reserve(edge_count_);
        for (int u = 0; u < num_nodes(); ++u)
            for (const auto& e : adj_[u])
                out.push_back(e);
        return out;
    }

    // ---- utility ----------------------------------------------------

    void range_check(int id) const {
        if (id < 0 || id >= static_cast<int>(nodes_.size()))
            throw std::out_of_range("Node id " + std::to_string(id) +
                                    " out of range [0," +
                                    std::to_string(nodes_.size()) + ")");
    }

private:
    std::vector<std::vector<ServiceEdge>> adj_;   ///< adjacency list
    std::vector<ServiceNode>              nodes_; ///< node metadata
    int                                   edge_count_{0};
};

} // namespace txn
