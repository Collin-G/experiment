#include "router.h"
#include <cmath>
#include <limits>
#include <unordered_map>
#include <iostream>


std::vector<int> find_nearest_edge(double lat, double lon,
                                   Direction dir = Direction::BOTH);


static double haversine(double lat1, double lon1,
                        double lat2, double lon2) {
    constexpr double R = 6371000.0; // meters
    double dlat = (lat2 - lat1) * M_PI / 180.0;
    double dlon = (lon2 - lon1) * M_PI / 180.0;

    double a = std::sin(dlat/2)*std::sin(dlat/2) +
               std::cos(lat1*M_PI/180.0) *
               std::cos(lat2*M_PI/180.0) *
               std::sin(dlon/2)*std::sin(dlon/2);

    return 2 * R * std::asin(std::sqrt(a));
}

struct Vec2 {
    double x;
    double y;
};

static Vec2 to_xy(double lat, double lon) {
    constexpr double R = 6371000.0;
    double x = lon * M_PI / 180.0 * R * std::cos(lat * M_PI / 180.0);
    double y = lat * M_PI / 180.0 * R;
    return {x, y};
}



double point_to_segment_distance(
    double plat, double plon,
    double alat, double alon,
    double blat, double blon
) {
    Vec2 P = to_xy(plat, plon);
    Vec2 A = to_xy(alat, alon);
    Vec2 B = to_xy(blat, blon);

    Vec2 AB{B.x - A.x, B.y - A.y};
    Vec2 AP{P.x - A.x, P.y - A.y};

    double ab2 = AB.x * AB.x + AB.y * AB.y;
    if (ab2 == 0.0) {
        // A and B are the same point
        double dx = P.x - A.x;
        double dy = P.y - A.y;
        return std::sqrt(dx*dx + dy*dy);
    }

    double t = (AP.x * AB.x + AP.y * AB.y) / ab2;

    if (t < 0.0) t = 0.0;
    else if (t > 1.0) t = 1.0;

    Vec2 C{
        A.x + t * AB.x,
        A.y + t * AB.y
    };

    double dx = P.x - C.x;
    double dy = P.y - C.y;
    return std::sqrt(dx*dx + dy*dy);
}


RoutingEngine::RoutingEngine(Graph graph)
    : graph_(std::move(graph)) {}

int RoutingEngine::find_nearest_node(double lat, double lon) const {
    double best = std::numeric_limits<double>::max();
    int best_id = -1;

    const auto& nodes = graph_.nodes();
    for (int i = 0; i < (int)nodes.size(); ++i) {
        double d = haversine(lat, lon, nodes[i].lat, nodes[i].lon);
        if (d < best) {
            best = d;
            best_id = i;
        }
    }
    return best_id;
}

bool RoutingEngine::matches_direction(
    double from_lat, double from_lon,
    double to_lat,   double to_lon,
    Direction dir
) {
    if (dir == Direction::BOTH || dir == Direction::NONE) {
        return true;
    }

    double dlat = to_lat - from_lat;  // north/south
    double dlon = to_lon - from_lon;  // east/west

    // Degenerate case: same point
    if (dlat == 0.0 && dlon == 0.0) {
        return false;
    }

    switch (dir) {
        case Direction::N:  return dlat > 0;
        case Direction::S:  return dlat < 0;
        case Direction::E:  return dlon > 0;
        case Direction::W:  return dlon < 0;

        case Direction::NE: return dlat > 0 && dlon > 0;
        case Direction::NW: return dlat > 0 && dlon < 0;
        case Direction::SE: return dlat < 0 && dlon > 0;
        case Direction::SW: return dlat < 0 && dlon < 0;

        default:
            return true;
    }
}



std::vector<int> RoutingEngine::find_nearest_edge(double lat, double lon, Direction dir){
    std::unordered_map<int, std::vector<int>> table;


    double best = std::numeric_limits<double>::max();
    int best_id = -1;
    std::vector<std::shared_ptr<Edge>>& edges = graph_.edges_mut();
    const auto& nodes = graph_.nodes();
    // const auto& nodes = graph_.nodes();
    for (int i = 0; i < (int)edges.size(); ++i) {

        double d = point_to_segment_distance(lat, lon, nodes[edges[i]->from].lat, nodes[edges[i]->from].lon, nodes[edges[i]->to].lat, nodes[edges[i]->to].lon);
        if( matches_direction(nodes[edges[i]->from].lat, nodes[edges[i]->from].lon, nodes[edges[i]->to].lat, nodes[edges[i]->to].lon, dir)){
             table[d].push_back(i);
        }
        if (d <= best) {
            best = d;
            best_id = i;
           
        }   
    }
    return table[best];


}

double RoutingEngine::route(double lat1, double lon1,
                            double lat2, double lon2) {
    int start = find_nearest_node(lat1, lon1);
    int goal  = find_nearest_node(lat2, lon2);




    if (start < 0 || goal < 0) return -1.0;

    AStar astar;
    return astar.shortest_path(graph_, start, goal).total_cost;
}

void RoutingEngine::update_edge(double lat, double lon, double weight, Direction dir){
    std::vector<int> closest_edges = find_nearest_edge(lat, lon, dir);

    for (int e : closest_edges){
        if (e < 0) return;
        // std::cout<< e<<"\n";

        graph_.update_edge_weight(e, weight);
    }

}

void RoutingEngine::update_edge(int id, double weight){
    if (id >= graph_.edges_mut().size() || id < 0){
        return;

    }

    graph_.update_edge_weight(id, weight);

}

void RoutingEngine::update_edge(int from, int to, double weight){
    std::vector<std::shared_ptr<Edge>>& edges = graph_.edges_mut();

    for (auto& e : edges){
        if (to == e->to && from == e->from){
            // std::cout<< e->id<<"\n";
            graph_.update_edge_weight(e->id, weight);
            return;
        }
    }
}



