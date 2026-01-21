#include "router.h"
#include "osm_parser.h"
#include "graphbuilder.h"

#include <osmium/io/any_input.hpp>
#include <osmium/visitor.hpp>

#include <memory>
#include <mutex>
#include <limits>

namespace {
    std::unique_ptr<RoutingEngine> engine;
    std::once_flag init_flag;
}

extern "C" {

#include <string>
#include <unordered_map>
#include <algorithm>

static Direction parse_direction(const std::string& input) {
    static const std::unordered_map<std::string, Direction> map = {
        {"N",  Direction::N},
        {"S",  Direction::S},
        {"E",  Direction::E},
        {"W",  Direction::W},
        {"NE", Direction::NE},
        {"NW", Direction::NW},
        {"SE", Direction::SE},
        {"SW", Direction::SW},
        {"BOTH", Direction::BOTH},
        {"NONE", Direction::NONE}
    };

    std::string key = input;
    std::transform(key.begin(), key.end(), key.begin(), ::toupper);

    auto it = map.find(key);
    if (it == map.end()) {
        return Direction::BOTH;   // safe default
    }

    return it->second;
}


bool init_router(const char* osm_file) {
    bool success = true;

    std::call_once(init_flag, [&]() {
        try {
            // 1. Parse OSM
            OSMHandler handler;
            osmium::io::Reader reader(osm_file);
            osmium::apply(reader, handler);
            reader.close();

            if (handler.nodes.empty() || handler.ways.empty()) {
                success = false;
                return;
            }

            // 2. Build graph
            GraphBuilder builder(
                std::move(handler.nodes),
                std::move(handler.ways)
            );

            Graph graph = builder.build_graph();

            if (graph.nodes().empty()) {
                success = false;
                return;
            }

            // 3. Create routing engine
            engine = std::make_unique<RoutingEngine>(std::move(graph));
        }
        catch (...) {
            success = false;
        }
    });

    return success && engine != nullptr;
}

double route_distance(double lat1, double lon1,
                      double lat2, double lon2) {
    if (!engine) {
        return std::numeric_limits<double>::infinity();
    }

    return engine->route(lat1, lon1, lat2, lon2);
}

void update_edge_by_coordinates(double lat,
                                double lon,
                                double weight,
                                const char* dir)
{
    if (!engine) {
        return;
    }

    Direction d = Direction::BOTH;

    if (dir && *dir) {
        d = parse_direction(std::string(dir));
    }

    engine->update_edge(lat, lon, weight, d);
}


void update_edge_by_id(int id, double weight){
      if (!engine) {
        return;
    }
    engine->update_edge(id, weight);

}


void update_edge_by_nodes(int from, int to, double weight) {
      if (!engine) {
        return;
    }
    engine->update_edge(from, to, weight);
}


}

