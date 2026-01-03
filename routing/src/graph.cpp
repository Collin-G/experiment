#include "graph.h"
#include <iostream>
#include <cstdlib>
#include <cassert>

void Graph::add_node(int id, double lat, double lon){
    Node node;
    node.id = id;
    node.lat = lat;
    node.lon = lon;

    nodes_.push_back(std::move(node));


}

void Graph::add_edge(int id, int from, int to, double weight){
    assert(from >= 0 && from < static_cast<int>(nodes_.size()));
    assert(to   >= 0 && to   < static_cast<int>(nodes_.size()));
    auto eptr = std::make_shared<Edge>();

    eptr->id = id;
    eptr->from = from;
    eptr->to = to;
    eptr->weight = weight;

    // Graph vector owns one shared_ptr
    edges_.push_back(eptr);

    // Node also keeps a shared_ptr to the same edge
    nodes_[from].edges.push_back(eptr);
}

void Graph::update_edge_weight(int id, double new_weight) {

    for (auto& e : edges_) {
        if (e->id == id) {

            e->weight = new_weight;
            return;
        }
    }
}

const std::vector<Node>& Graph::nodes() const {
    return nodes_;
}

std::vector<Node>& Graph::nodes_mut() {
    return nodes_;
}

std::vector<std::shared_ptr<Edge>>& Graph::edges_mut() {
    return edges_;
}
