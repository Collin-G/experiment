#pragma once

#include <osmium/handler.hpp>
#include <osmium/io/any_input.hpp>
#include <osmium/visitor.hpp>
#include <unordered_map>
#include <vector>
#include <string>

// Struct to store node info
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

// Struct to store way info
struct OSMWay {
    int64_t id;
    std::vector<int64_t> node_ids;   // list of node IDs
    std::string highway_type;        // e.g., "residential", "primary"
    int maxspeed;                    // optional
    OneWay oneway;
};

// Handler class
class OSMHandler : public osmium::handler::Handler {
public:
    // Maps to store nodes and ways
    std::unordered_map<int64_t, OSMNode> nodes;
    std::vector<OSMWay> ways;

    // Called for each node in the OSM file
    void node(const osmium::Node& n);

    // Called for each way in the OSM file
    void way(const osmium::Way& w);
};
