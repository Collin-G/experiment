#include "graphbuilder.h"
#include "graph.h"
#include <iostream>
#include <queue>
#include <unordered_set>
#include <cmath>
#include <algorithm>

// Helper to count directed edges using public Graph API
size_t count_edges(const Graph& g) {
    size_t total = 0;
    for (size_t i = 0; i < g.nodes().size(); ++i) {
        total += g.neighbors(i).size();
    }
    return total;
}

std::unordered_map<int64_t, int> GraphBuilder::find_intersections() {
    std::unordered_map<int64_t, int> table;
    for (const OSMWay& way : ways_) {
        for (int64_t node_id : way.node_ids) {
            table[node_id]++;
        }
    }
    return table;
}

double GraphBuilder::haversine(OSMNode& n1, OSMNode& n2) {
    const double R = 6371000.0;
    double lat1 = n1.lat * M_PI / 180.0;
    double lon1 = n1.lon * M_PI / 180.0;
    double lat2 = n2.lat * M_PI / 180.0;
    double lon2 = n2.lon * M_PI / 180.0;
    double dlat = lat2 - lat1;
    double dlon = lon2 - lon1;
    double a = std::sin(dlat / 2) * std::sin(dlat / 2) +
               std::cos(lat1) * std::cos(lat2) * std::sin(dlon / 2) * std::sin(dlon / 2);
    double c = 2 * std::atan2(std::sqrt(a), std::sqrt(1 - a));
    return R * c;
}

bool GraphBuilder::is_endpoint(int node_id, OSMWay& way) {
    return node_id == way.node_ids.front() || node_id == way.node_ids.back();
}

Graph GraphBuilder::build_graph() {
    // ------------------------------------------------------------------
    // STAGE 1: Count intersections and identify routing nodes
    // ------------------------------------------------------------------
    auto usage = find_intersections();

    std::unordered_set<int64_t> routing_nodes;
    for (const auto& way : ways_) {
        if (!way.node_ids.empty()) {
            routing_nodes.insert(way.node_ids.front());
            routing_nodes.insert(way.node_ids.back());
        }
    }
    for (const auto& [node_id, count] : usage) {
        if (count > 1) routing_nodes.insert(node_id);
    }

    // ------------------------------------------------------------------
    // STAGE 2: Assign graph indices (nodes must exist in nodes_ map)
    // ------------------------------------------------------------------
    std::unordered_map<int64_t, int> id_to_index;
    Graph graph;
    int idx = 0;
    for (int64_t node_id : routing_nodes) {
        auto it = nodes_.find(node_id);
        if (it == nodes_.end()) continue;
        id_to_index[node_id] = idx;
        graph.add_node(idx, it->second.lat, it->second.lon);
        idx++;
    }

    // ------------------------------------------------------------------
    // STAGE 3: Build edges along ways between routing nodes
    // ------------------------------------------------------------------
    for (const OSMWay& way : ways_) {
        if (way.node_ids.size() < 2) continue;

        double speed_kmh = way.maxspeed > 0 ? way.maxspeed : 30.0;
        double speed_mps = speed_kmh * 1000.0 / 3600.0;

        int64_t prev_routing_node = -1;
        double acc_distance = 0.0;

        for (size_t i = 1; i < way.node_ids.size(); ++i) {
            int64_t prev_id = way.node_ids[i - 1];
            int64_t curr_id = way.node_ids[i];

            auto prev_it = nodes_.find(prev_id);
            auto curr_it = nodes_.find(curr_id);
            if (prev_it == nodes_.end() || curr_it == nodes_.end()) continue;

            acc_distance += haversine(prev_it->second, curr_it->second);

            if (routing_nodes.count(curr_id)) {
                if (prev_routing_node != -1) {
                    double eta = acc_distance / speed_mps;

                    auto from_it = id_to_index.find(prev_routing_node);
                    auto to_it   = id_to_index.find(curr_id);
                    if (from_it != id_to_index.end() && to_it != id_to_index.end()) {
                        int from = from_it->second;
                        int to   = to_it->second;

                        if (way.oneway == OneWay::Forward) {
                            graph.add_edge(way.id, from, to, eta);
                        } else if (way.oneway == OneWay::Backward) {
                            graph.add_edge(way.id, to, from, eta);
                        } else {
                            graph.add_edge(way.id, from, to, eta);
                            graph.add_edge(way.id, to, from, eta);
                        }
                    }
                }
                prev_routing_node = curr_id;
                acc_distance = 0.0;
            }
        }
    }

    // ------------------------------------------------------------------
    // STAGE 4: Optional debug output (remove in production)
    // ------------------------------------------------------------------
    std::cout << "\n=== GRAPH BUILDER ===\n";
    std::cout << "Raw graph nodes: " << graph.nodes().size() << "\n";
    std::cout << "Raw graph edges: " << count_edges(graph) << "\n";

    // ------------------------------------------------------------------
    // STAGE 5: Keep only the largest connected component
    // ------------------------------------------------------------------
    Graph filtered = filter_largest_connected_component(graph);
    
    std::cout << "Filtered graph nodes: " << filtered.nodes().size() << "\n";
    std::cout << "Filtered graph edges: " << count_edges(filtered) << "\n";
    std::cout << "========================\n";

    return filtered;
}

Graph GraphBuilder::filter_largest_connected_component(const Graph& original) {
    int N = original.nodes().size();
    if (N == 0) return Graph();

    std::vector<bool> visited(N, false);
    std::vector<int> largest_component;

    for (int i = 0; i < N; ++i) {
        if (visited[i]) continue;

        std::vector<int> component;
        std::queue<int> q;
        q.push(i);
        visited[i] = true;

        while (!q.empty()) {
            int curr = q.front(); q.pop();
            component.push_back(curr);

            for (const auto& [neighbor, _] : original.neighbors(curr)) {
                if (!visited[neighbor]) {
                    visited[neighbor] = true;
                    q.push(neighbor);
                }
            }
        }

        if (component.size() > largest_component.size()) {
            largest_component = std::move(component);
        }
    }

    // Build filtered graph
    Graph filtered;
    std::unordered_map<int, int> old_to_new;
    old_to_new.reserve(largest_component.size());

    int new_idx = 0;
    for (int old_idx : largest_component) {
        old_to_new[old_idx] = new_idx;
        filtered.add_node(new_idx,
                          original.get_node_lat(old_idx),
                          original.get_node_lon(old_idx));
        new_idx++;
    }

    int edge_id = 0;
    for (int old_idx : largest_component) {
        int new_from = old_to_new[old_idx];

        for (const auto& [old_to, eta] : original.neighbors(old_idx)) {
            auto it = old_to_new.find(old_to);
            if (it != old_to_new.end()) {
                filtered.add_edge(edge_id++, new_from, it->second, eta);
            }
        }
    }

    return filtered;
}
