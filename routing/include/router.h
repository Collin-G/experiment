#pragma once
#include "graph.h"
#include "astar.h"

class RoutingEngine {
public:
    RoutingEngine(Graph graph);
    void update_edge(double lat, double lon, double weight);
    void update_edge(int id, double weight);
    void update_edge(int from, int to, double weight);

    double route(double lat1, double lon1,
                 double lat2, double lon2);
    

    Graph view_graph() {return graph_;}

private:
    Graph graph_;

    int find_nearest_node(double lat, double lon) const;
    int find_nearest_edge(double lat, double lon);


};
