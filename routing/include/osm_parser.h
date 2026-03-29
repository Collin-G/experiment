#pragma once

#include <osmium/handler.hpp>
#include <osmium/osm/node.hpp>   // <-- REQUIRED
#include <osmium/osm/way.hpp>    // <-- REQUIRED

#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <string>

// ----------------------------
// Basic OSM Structures
// ----------------------------

struct OSMNode {
    int64_t id;
    double lat;
    double lon;
};

enum struct OneWay {
    No,
    Forward,
    Backward,
};

struct OSMWay {
    int64_t id;
    std::vector<int64_t> node_ids;
    int maxspeed = 0;
    OneWay oneway = OneWay::No;
};

// ----------------------------
// PASS 1 — Collect relevant ways + node IDs
// ----------------------------

class WayCollector : public osmium::handler::Handler {
public:
    std::unordered_set<int64_t> required_nodes;
    std::vector<OSMWay> relevant_ways;
    std::unordered_map<std::string, size_t> excluded_highway_counts;
    size_t excluded_no_highway = 0;

    void way(const osmium::Way& w);
    void print_stats() const;
};

// ----------------------------
// PASS 2 — Collect only required nodes
// ----------------------------

class NodeCollector : public osmium::handler::Handler {
public:
    const std::unordered_set<int64_t>& required_nodes;
    std::unordered_map<int64_t, OSMNode> nodes;

    NodeCollector(const std::unordered_set<int64_t>& req);

    void node(const osmium::Node& n);
};
