#include "astar.h"
#include "graph.h"
#include <queue>
#include <limits>
#include <cmath>
#include <algorithm>

struct AStarNode {
    int index;
    double g_cost;  // cost from start
    double f_cost;  // g + heuristic
    int parent;

    bool operator>(const AStarNode& other) const {
        return f_cost > other.f_cost;
    }
};

double AStar::heuristic(const Node& a, const Node& b) {
    const double R = 6371000.0; // Earth radius in meters
    double lat1 = a.lat * M_PI / 180.0;
    double lon1 = a.lon * M_PI / 180.0;
    double lat2 = b.lat * M_PI / 180.0;
    double lon2 = b.lon * M_PI / 180.0;

    double dlat = lat2 - lat1;
    double dlon = lon2 - lon1;

    double h = std::sin(dlat/2) * std::sin(dlat/2) +
               std::cos(lat1)*std::cos(lat2)*std::sin(dlon/2)*std::sin(dlon/2);
    return 2 * R * std::atan2(std::sqrt(h), std::sqrt(1-h));
}




AStarResult AStar::shortest_path(const Graph& graph, int start_idx, int goal_idx) {
    const auto& nodes = graph.nodes();
    int N = nodes.size();

    std::vector<double> g(N, std::numeric_limits<double>::infinity());
    std::vector<int> parent(N, -1);
    std::vector<bool> closed(N, false);

    using PQElement = AStarNode;
    std::priority_queue<PQElement, std::vector<PQElement>, std::greater<PQElement>> open;

    g[start_idx] = 0.0;
    open.push({start_idx, 0.0,
               heuristic(nodes[start_idx], nodes[goal_idx]), -1});

    while (!open.empty()) {
        auto current = open.top();
        open.pop();

        if (closed[current.index]) continue;
        closed[current.index] = true;

        if (current.index == goal_idx) break;

        for (const auto& edge : nodes[current.index].edges) {
            int neighbor = edge->to;
            if (closed[neighbor]) continue;

            double tentative_g = g[current.index] + edge->weight;
            if (tentative_g < g[neighbor]) {
                g[neighbor] = tentative_g;
                parent[neighbor] = current.index;
                double f = tentative_g +
                           heuristic(nodes[neighbor], nodes[goal_idx]);
                open.push({neighbor, tentative_g, f, current.index});
            }
        }
    }

    AStarResult result;

    if (g[goal_idx] == std::numeric_limits<double>::infinity()) {
        // No path
        result.total_cost = std::numeric_limits<double>::infinity();
        return result;
    }

    // Reconstruct path
    int curr = goal_idx;
    while (curr != -1) {
        result.path.push_back(curr);
        curr = parent[curr];
    }
    std::reverse(result.path.begin(), result.path.end());

    result.total_cost = g[goal_idx];
    return result;
}

