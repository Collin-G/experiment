#pragma once

#include <vector>
#include <cstddef>
#include <memory>

struct Edge {
    int id;
    int from;
    int to;
    double weight;

};

struct Node {
    int id;
    double lat;
    double lon;
    std::vector<std::shared_ptr<Edge>> edges;
};


class Graph {

    public:
        Graph() = default;
        void add_node(int id, double lat, double lon);
        void add_edge(int id, int from, int to, double weight);
        void update_edge_weight(int id, double new_weight);


        const std::vector<Node>& nodes() const;
        std::vector<Node>& nodes_mut();
        std::vector<std::shared_ptr<Edge>>& edges_mut();


        // number of nodes
        int num_nodes() const { return static_cast<int>(nodes_.size()); }

        // neighbors of a node (returns vector of {to, weight})
        std::vector<std::pair<int,double>> neighbors(int idx) const {
            std::vector<std::pair<int,double>> result;
            for (const auto& e : nodes_[idx].edges) {
                result.emplace_back(e->to, e->weight);
            }
            return result;
        }

        // get node coordinates
        double get_node_lat(int idx) const { return nodes_[idx].lat; }
        double get_node_lon(int idx) const { return nodes_[idx].lon; }


    private:
        std::vector<Node> nodes_;
         std::vector<std::shared_ptr<Edge>> edges_;
      
};