#pragma once

#include <unordered_map>
#include <vector>

#include "graph.h"
#include "osm_parser.h"   // OK here, used only for data types


class GraphBuilder {
    public:
        GraphBuilder(std::unordered_map<int64_t, OSMNode>&& nodes, std::vector<OSMWay>&& ways) : nodes_(std::move(nodes)), ways_(std::move(ways)) {}
        // static Graph build();

        std::unordered_map<int64_t, int> find_intersections();
        double haversine(OSMNode& n1, OSMNode& n2);
        Graph build_graph();  
        bool is_endpoint(int node_id, OSMWay& way);
        Graph filter_largest_connected_component(const Graph& original);
    
    private:
        std::unordered_map<int64_t, OSMNode> nodes_;
        std::vector<OSMWay> ways_;
    };