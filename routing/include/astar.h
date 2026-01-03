#pragma once

#include "graph.h"
#include <vector>


struct AStarResult {
    std::vector<int> path;
    double total_cost;
};


class AStar {
public:
    // Compute shortest path from start to goal (graph indices)
    // Returns a vector of graph node indices representing the path
    static AStarResult shortest_path(const Graph& graph, int start_idx, int goal_idx);

private:
    // Heuristic: haversine distance between two nodes
    static double heuristic(const Node& a, const Node& b);
};
