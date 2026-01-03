#include "osm_parser.h"
#include "graphbuilder.h"
#include "astar.h"
#include "router.h"
#include <osmium/io/any_input.hpp>
#include <osmium/visitor.hpp>
#include <iostream>
#include <vector>


#include <fstream>
#include "graph.h"

void export_to_dot(const Graph& g, const std::string& filename) {
    if (g.nodes().empty()) return;

    std::ofstream file(filename);
    file << "graph G {\n";
    file << "  node [shape=circle, width=0.1];\n";

    // Export nodes using raw lon/lat
    for (const auto& node : g.nodes()) {
        double x = node.lon;
        double y = node.lat;   // north is up; no flip needed unless you prefer screen coords

        file << "  " << node.id
             << " [label=\"" << node.id
             << "\", pos=\"" << x << "," << y << "!\"];\n";
    }

    // Export edges
    for (const auto& node : g.nodes()) {
        for (const auto& edge : node.edges) {
            if (node.id < edge->to) {
                file << "  " << node.id << " -- " << edge->to
                     << " [label=\"" << edge->weight << "\"];\n";
            }
        }
    }

    file << "}\n";
}


int main(int argc, char* argv[]) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <osm_file.osm.pbf>\n";
        return 1;
    }

    std::string osm_file = argv[1];

    // 1. Read OSM file
    OSMHandler handler;
    osmium::io::Reader reader(osm_file);
    osmium::apply(reader, handler);
    reader.close();

    std::cout << "Loaded " << handler.nodes.size() << " nodes and " << handler.ways.size() << " ways.\n";

    // 2. Build the graph
    GraphBuilder builder(std::move(handler.nodes), std::move(handler.ways));
    Graph graph = builder.build_graph();

    std::cout << "Graph has " << graph.nodes().size() << " nodes and " << graph.edges_mut().size() << " ways. \n";

    // 3. Pick start and goal nodes (example: first and last)
    if (graph.nodes().size() < 2) {
        std::cerr << "Not enough nodes to run A*\n";
        return 1;
    }

    int start_idx = 0;
    int goal_idx = graph.nodes().size() - 1;

    // 4. Run A* search
    AStar astar;
    std::vector<int> path = astar.shortest_path(graph, start_idx, goal_idx).path;

    std::cout << "Path length: " << path.size() << "\n";
    for (int idx : path) {
        const Node& n = graph.nodes()[idx];
        std::cout << n.id << " (" << n.lat << ", " << n.lon << ")\n";
    }

    std::cout << "distance: " << astar.shortest_path(graph, start_idx, goal_idx).total_cost << "\n";

    export_to_dot(graph, "/home/collin/programming/flux/routing/files/graph_scaled.dot");


    RoutingEngine routing_engine = RoutingEngine(graph);
    double lat1 = 43.69;
    double lon1 = -79.32;
    double lat2 = 43.6845;
    double lon2 = -79.339;

    // int begin = routing_engine.find_nearest_node(lat1, lon1);
    // int end = routing_engine.find_nearest_node(la2, lon2);

    std::cout <<routing_engine.route(lat1,lon1,lat2, lon2) << "\n";

    routing_engine.update_edge(0,3, 999);
    std::cout <<routing_engine.route(lat1,lon1,lat2, lon2) << "\n";




    return 0;
}
