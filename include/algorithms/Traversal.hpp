#pragma once

#include "Graph.hpp"
#include <vector>
#include <queue>
#include <functional>
#include <unordered_map>
#include <limits>

// ============================================================
// Traversal.hpp — BFS, DFS, multi-source BFS, visitor-BFS
//
// All traversals run on the generic txn::Graph. They return
// visit-order vectors or distance maps so callers can derive
// failure-propagation chains, reachability, etc.
// ============================================================

namespace txn {

constexpr int UNVISITED = -1;

// ----------------------------------------------------------
// BFS — breadth-first traversal from a single source.
// Returns nodes in the order they were first discovered.
// ----------------------------------------------------------
[[nodiscard]] inline std::vector<int> bfs(const Graph& g, int start) {
    g.range_check(start);
    const int N = g.num_nodes();
    std::vector<bool> visited(N, false);
    std::vector<int>  order;
    order.reserve(N);

    std::queue<int> q;
    visited[start] = true;
    q.push(start);

    while (!q.empty()) {
        int u = q.front(); q.pop();
        order.push_back(u);
        for (const auto& e : g.neighbors(u)) {
            if (!visited[e.dst]) {
                visited[e.dst] = true;
                q.push(e.dst);
            }
        }
    }
    return order;
}

// ----------------------------------------------------------
// DFS — depth-first traversal from a single source.
// Uses an explicit stack to avoid recursion-depth issues on
// large service graphs.
// ----------------------------------------------------------
[[nodiscard]] inline std::vector<int> dfs(const Graph& g, int start) {
    g.range_check(start);
    const int N = g.num_nodes();
    std::vector<bool> visited(N, false);
    std::vector<int>  order;
    order.reserve(N);

    // Explicit stack — mirrors the call stack of recursive DFS
    std::vector<int> stk;
    stk.reserve(N);
    stk.push_back(start);

    while (!stk.empty()) {
        int u = stk.back(); stk.pop_back();
        if (visited[u]) continue;
        visited[u] = true;
        order.push_back(u);
        // Push neighbours in reverse so that the lowest-id neighbour
        // is processed first (mimics typical recursive DFS order).
        const auto& nbrs = g.neighbors(u);
        for (int i = static_cast<int>(nbrs.size()) - 1; i >= 0; --i)
            if (!visited[nbrs[i].dst])
                stk.push_back(nbrs[i].dst);
    }
    return order;
}

// ----------------------------------------------------------
// Multi-source BFS — simultaneous BFS from several seeds.
// Returns a map: node_id → distance from nearest source (0
// for source nodes themselves). Unreachable nodes are absent.
// Useful for computing blast radius from a set of failed nodes.
// ----------------------------------------------------------
[[nodiscard]] inline std::unordered_map<int,int>
multi_source_bfs(const Graph& g, const std::vector<int>& sources) {
    const int N = g.num_nodes();
    std::unordered_map<int,int> dist;
    dist.reserve(N);

    std::queue<int> q;
    for (int s : sources) {
        g.range_check(s);
        if (dist.find(s) == dist.end()) {
            dist[s] = 0;
            q.push(s);
        }
    }
    while (!q.empty()) {
        int u = q.front(); q.pop();
        for (const auto& e : g.neighbors(u)) {
            if (dist.find(e.dst) == dist.end()) {
                dist[e.dst] = dist[u] + 1;
                q.push(e.dst);
            }
        }
    }
    return dist;
}

// ----------------------------------------------------------
// BFS with callback — visitor pattern.
// The callback receives (node_id, depth) for every newly
// discovered node. Returning false from the callback aborts
// the traversal early (useful for early-exit reachability).
// ----------------------------------------------------------
inline void bfs_with_callback(const Graph& g, int start,
    std::function<bool(int /*node*/, int /*depth*/)> callback)
{
    g.range_check(start);
    const int N = g.num_nodes();
    std::vector<int> depth(N, -1);
    std::queue<int>  q;
    depth[start] = 0;
    q.push(start);

    while (!q.empty()) {
        int u = q.front(); q.pop();
        // Invoke visitor; abort if it returns false
        if (!callback(u, depth[u])) return;
        for (const auto& e : g.neighbors(u)) {
            if (depth[e.dst] == -1) {
                depth[e.dst] = depth[u] + 1;
                q.push(e.dst);
            }
        }
    }
}

} // namespace txn
