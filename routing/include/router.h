#pragma once
#include "graph.h"
#include "astar.h"


enum struct Direction {
    N, E, S, W,
    NE, NW, SE, SW,
    BOTH, NONE

};

class RoutingEngine {
public:
    RoutingEngine(Graph graph);
    void update_edge(double lat, double lon, double weight, Direction dir = Direction::BOTH);
    void update_edge(int id, double weight);
    void update_edge(int from, int to, double weight);
    bool matches_direction(double from_lat, double from_lon, double to_lat, double to_lon, Direction dir = Direction::BOTH);
    double route(double lat1, double lon1,
                 double lat2, double lon2);
    

    Graph view_graph() {return graph_;}

private:
    Graph graph_;

    int find_nearest_node(double lat, double lon) const;
    std::vector<int> find_nearest_edge(double lat, double lon, Direction dir = Direction::BOTH);


};
