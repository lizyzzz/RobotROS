#include "Planner/node.h"


namespace rrt_star_Global_planner {

Node::Node() { } 

Node::Node(float px, float py, int node_index, int parent_index) : x(px), y(py), node_id(node_index), parent_id(parent_index) { }

bool Node::operator==(const Node& node) {
    return node_id == node.node_id;
}

bool Node::operator!=(const Node& node) {
    return !(node_id == node.node_id);
}


};