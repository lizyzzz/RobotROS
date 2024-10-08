#include "ImRRTStarFN02/node.h"


namespace rrt_starFN02_Global_planner {

Node::Node() { } 

Node::Node(float px, float py, int node_index, int parent_index) : x(px), y(py), node_id(node_index), parent_id(parent_index) { }

bool Node::operator==(const Node& node) {
    return node_id == node.node_id;
}

bool Node::operator!=(const Node& node) {
    return !(node_id == node.node_id);
}

Node& Node::operator=(const Node& node) {
    x = node.x;
    y = node.y;
    node_id = node.node_id;
    parent_id = node.parent_id;
    cost = node.cost;
    h_cost = node.h_cost;
    leaf = node.leaf;
    child_id = node.child_id;
    return *this;
}


};