#include "osm_parser.h"
#include <cstring>
#include <iostream>
#include <cstdlib>

static const std::unordered_set<std::string> allowed_highways = {
    "motorway","motorway_link",
    "trunk","trunk_link",
    "primary","primary_link",
    "secondary","secondary_link",
    "tertiary","tertiary_link",
    "unclassified",
    "residential",
    "living_street",
    "service",
    "track",
    "road"
    // "busway"
};

// -----------------------------------------------------
// WayCollector
// -----------------------------------------------------
void WayCollector::way(const osmium::Way& w) {

    if (!w.tags().has_key("highway")) {
        excluded_no_highway++;
        return;
    }

    const char* hw = w.tags()["highway"];
    std::string type = hw;

    if (allowed_highways.find(type) == allowed_highways.end()) {
        excluded_highway_counts[type]++;
        return;
    }

    OSMWay way;
    way.id = w.id();

    if (const char* speed = w.tags()["maxspeed"])
        way.maxspeed = std::atoi(speed);

    if (const char* ow = w.tags()["oneway"]) {
        if (!strcmp(ow,"yes") || !strcmp(ow,"1"))
            way.oneway = OneWay::Forward;
        else if (!strcmp(ow,"-1"))
            way.oneway = OneWay::Backward;
    }

    for (const auto& n : w.nodes()) {
        way.node_ids.push_back(n.ref());
        required_nodes.insert(n.ref());
    }

    if (way.node_ids.size() >= 2)
        relevant_ways.push_back(std::move(way));
}

// -----------------------------------------------------
// NodeCollector
// -----------------------------------------------------

NodeCollector::NodeCollector(const std::unordered_set<int64_t>& req)
    : required_nodes(req) {}

void NodeCollector::node(const osmium::Node& n) {
    if (!n.location().valid()) return;

    if (required_nodes.count(n.id())) {
        OSMNode node;
        node.id = n.id();
        node.lat = n.location().lat();
        node.lon = n.location().lon();
        nodes[node.id] = node;
    }
}

void WayCollector::print_stats() const {
    std::cout << "Excluded (no highway tag): "
              << excluded_no_highway << "\n\n";

    std::cout << "Excluded highway types:\n";
    for (const auto& [type, count] : excluded_highway_counts) {
        std::cout << "  " << type << ": " << count << "\n";
    }
}