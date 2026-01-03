#include "graphbuilder.h"
#include "osm_parser.h"
#include "graph.h"

#include <iostream>
#include <cstdlib>
#include <cmath>  // for sin, cos, atan2, sqrt

#include <unordered_set>

std::unordered_map<int64_t, int> GraphBuilder::find_intersections(){
    std::unordered_map<int64_t, int> table;
    for (OSMWay& way : ways_){
        for (int64_t node_id: way.node_ids){
            table[node_id]++;
        }
    }
    
    return table;
}

double GraphBuilder::haversine(OSMNode& n1, OSMNode& n2) {
    const double R = 6371000.0; // radius of Earth in meters

    // Convert degrees to radians
    double lat1 = n1.lat * M_PI / 180.0;
    double lon1 = n1.lon * M_PI / 180.0;
    double lat2 = n2.lat * M_PI / 180.0;
    double lon2 = n2.lon * M_PI / 180.0;

    double dlat = lat2 - lat1;
    double dlon = lon2 - lon1;

    double a = std::sin(dlat / 2) * std::sin(dlat / 2) +
               std::cos(lat1) * std::cos(lat2) *
               std::sin(dlon / 2) * std::sin(dlon / 2);

    double c = 2 * std::atan2(std::sqrt(a), std::sqrt(1 - a));

    double distance = R * c;
    return distance; // in meters


}

bool GraphBuilder::is_endpoint(int node_id, OSMWay& way){
    return node_id ==  way.node_ids.front() || node_id == way.node_ids.back();   
}




Graph GraphBuilder::build_graph() {
    Graph graph;

    // 1. Count how many ways reference each node
    std::unordered_map<int64_t, int> usage = find_intersections();

    // 2. Identify routing nodes (endpoints or shared nodes)
    std::unordered_set<int64_t> routing_nodes;
    for (const auto& way : ways_) {
        if (!way.node_ids.empty()) {
            routing_nodes.insert(way.node_ids.front());
            routing_nodes.insert(way.node_ids.back());
        }
    }
    for (const auto& [node_id, count] : usage) {
        if (count > 1) {
            routing_nodes.insert(node_id);
        }
    }

    // 3. Assign graph indices
    std::unordered_map<int64_t, int> id_to_index;
    int idx = 0;
    for (int64_t node_id : routing_nodes) {
        const OSMNode& osm = nodes_.at(node_id);
        id_to_index[node_id] = idx;
        graph.add_node(idx, osm.lat, osm.lon);
        idx++;
    }

    // 4. Build edges
    for (const OSMWay& way : ways_) {
        if (way.node_ids.size() < 2) continue;

        double speed_kmh = way.maxspeed > 0 ? way.maxspeed : 30.0;
        double speed_mps = speed_kmh * 1000.0 / 3600.0;

        int64_t prev_routing_node = -1;
        double acc_distance = 0.0;

        for (size_t i = 1; i < way.node_ids.size(); ++i) {
            int64_t prev_id = way.node_ids[i - 1];
            int64_t curr_id = way.node_ids[i];

            // Always accumulate distance
            auto& prev_node = nodes_.at(prev_id);
            auto& curr_node = nodes_.at(curr_id);
            acc_distance += haversine(prev_node, curr_node);

            // If current node is a routing node, create an edge
            if (routing_nodes.count(curr_id)) {
                if (prev_routing_node != -1) {
                    double eta = acc_distance / speed_mps;

                    int from = id_to_index.at(prev_routing_node);
                    int to   = id_to_index.at(curr_id);

                    if (way.oneway == OneWay::Forward) {
                        graph.add_edge(way.id, from, to, eta);
                    }
                    else if (way.oneway == OneWay::Backward) {
                        graph.add_edge(way.id, to, from, eta);
                    }
                    else {
                        graph.add_edge(way.id, from, to, eta);
                        graph.add_edge(way.id, to, from, eta);
                    }
                }

                // Reset for next segment
                prev_routing_node = curr_id;
                acc_distance = 0.0;
            }
        }
    }

    graph = filter_largest_connected_component(graph);

    return graph;
}

Graph GraphBuilder::filter_largest_connected_component(const Graph& original) {
    int N = original.nodes().size();
    std::vector<bool> visited(N, false);
    std::vector<std::vector<int>> components;

    // 1. BFS to find components
    for (int i = 0; i < N; ++i) {
        if (visited[i]) continue;

        std::vector<int> component;
        std::queue<int> q;
        q.push(i);
        visited[i] = true;

        while (!q.empty()) {
            int curr = q.front(); q.pop();
            component.push_back(curr);

            for (const auto& neighbor_pair : original.neighbors(curr)) {
                int neighbor = neighbor_pair.first;
                if (!visited[neighbor]) {
                    visited[neighbor] = true;
                    q.push(neighbor);
                }
            }
        }

        components.push_back(component);
    }

    // 2. Keep largest component
    size_t max_size = 0;
    int main_idx = -1;
    for (size_t i = 0; i < components.size(); ++i) {
        if (components[i].size() > max_size) {
            max_size = components[i].size();
            main_idx = i;
        }
    }

    if (main_idx == -1) return Graph(); // empty graph

    const auto& main_component = components[main_idx];
    std::unordered_set<int> main_nodes(main_component.begin(), main_component.end());

    // 3. Build filtered graph
    Graph filtered_graph;
    std::unordered_map<int, int> old_to_new;
    int new_idx = 0;

    for (int old_idx : main_component) {
        old_to_new[old_idx] = new_idx;
        filtered_graph.add_node(
            new_idx,
            original.get_node_lat(old_idx),
            original.get_node_lon(old_idx)
        );
        new_idx++;
    }

    int edge_ind = 0;

    for (int old_idx : main_component) {
        int new_from = old_to_new[old_idx];
        for (const auto& neighbor_pair : original.neighbors(old_idx)) {
            int old_to = neighbor_pair.first;
            double eta = neighbor_pair.second;
            if (main_nodes.count(old_to)) {
                int new_to = old_to_new[old_to];
                filtered_graph.add_edge(edge_ind, new_from, new_to, eta);
                edge_ind ++;
            }
        }
    }

    return filtered_graph;
}
