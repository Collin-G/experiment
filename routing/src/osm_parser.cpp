#include "osm_parser.h"
#include <iostream>
#include <cstdlib>



#include <unordered_set>

const std::unordered_set<std::string> allowed_highways = {
    "motorway", "motorway_link",
    "trunk", "trunk_link",
    "primary", "primary_link",
    "secondary", "secondary_link",
    "tertiary", "tertiary_link", "unclassified", 
    "residential", "living_street"
};


void OSMHandler::node(const osmium::Node& n) {
    // if (!n.location().valid()) return;

    OSMNode node;
    node.id = n.id();
    node.lat = n.location().lat();
    node.lon = n.location().lon();

    nodes[node.id] = node;
}

void OSMHandler::way(const osmium::Way& w) {
    if (!w.tags().has_key("highway")) return;

    std::string type = w.tags()["highway"];
    if (allowed_highways.find(type) == allowed_highways.end()) return; // skip non-drivable

    OSMWay way;
    way.id = w.id();
    way.highway_type = type;
    way.maxspeed = 0;
    way.oneway = OneWay::No;

    if (const char* speed = w.tags()["maxspeed"]) {
        way.maxspeed = std::atoi(speed);
    }

    if (const char* ow = w.tags()["oneway"]) {
        if (std::strcmp(ow, "yes") == 0 || std::strcmp(ow, "1") == 0) {
            way.oneway = OneWay::Forward;
        } else if (std::strcmp(ow, "-1") == 0) {
            way.oneway = OneWay::Backward;
        }
    }

    for (const auto& n : w.nodes()) {
        // if (!n.location().valid()) continue;
        way.node_ids.push_back(n.ref());
    }

    


    ways.push_back(std::move(way));
}
