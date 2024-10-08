#include "ImRRTStarFN02/ImRRTStarFN.h"


namespace rrt_starFN02_Global_planner {

ImRRTStarFN::ImRRTStarFN(const std::pair<double, double>& start_point, 
            const std::pair<double, double>& goal_point,
            costmap_2d::Costmap2D* costmap, 
            double goal_tolerance, 
            double radius, 
            double epsilon, 
            int fixNode,
            int dynamic_count,
            int max_radius_factor,
            unsigned int max_num_nodes, 
            unsigned int min_num_nodes, 
            double map_width, 
            double map_height) : start_point_(start_point),
                                goal_point_(goal_point),
                                costmap_(costmap),
                                goal_tolerance_(goal_tolerance),
                                radius_(radius),
                                epsilon_(epsilon),
                                fixNode_(fixNode),
                                dynamic_count_(dynamic_count),
                                max_radius_factor_(max_radius_factor),
                                max_num_nodes_(max_num_nodes),
                                min_num_nodes_(min_num_nodes),
                                map_width_(map_width),
                                map_height_(map_height),
                                cd_(costmap)
            { 
                nodes_.reserve(max_num_nodes_ + 1);
                nodes_fn_.reserve(fixNode_ + 1);
                deltaR_ = (euclideanDistance2D(start_point_.first, start_point_.second, goal_point_.first, goal_point_.second) - radius_) / (max_radius_factor_ * dynamic_count_);
                deltaCount_ = std::floor(max_num_nodes_ / dynamic_count_);
                // set range
                // random_double_gen_.setRange(-map_width_, map_width_, 0);
                // random_double_gen_.setRange(-map_height_, map_height_, 1);
                random_double_gen_.setRange(0, map_width_, 0);
                random_double_gen_.setRange(0, map_height_, 1);
            }

// core algorithm
bool ImRRTStarFN::pathPlanning(std::list<std::pair<double, double>> &path) {
    goal_reached_ = false;

    // this goal is collides
    if (cd_.isThisPointCollides(goal_point_.first, goal_point_.second)) {
        ROS_ERROR("This goal is not in Free Space. Choose other goal");
        return false;
    }
    // start point
    creatNewNode(start_point_.first, start_point_.second, -1);

    std::pair<double, double> p_rand;
    std::pair<double, double> p_new;
    Node node_nearest;

    bool found_next;
    int R_count = 1;
    bool goal_reached_cnt = false; // if reach firstly
    while (node_count_ < max_num_nodes_) {
        found_next = false;
        // random sample
        while (!found_next) {
            // generate random point
            if (!goal_reached_) {
                p_rand = sampleFree();
            }
            else {
                p_rand = sampleBall();
            }
            
            // get NearestNode
            node_nearest = nodes_.at(getNearestNodeId(p_rand));
            // one step in random point direction
            p_new = steer(node_nearest.x, node_nearest.y, p_rand.first, p_rand.second);

            // no obstacle
            if (!cd_.isThereObstacleBetween(node_nearest, p_new)) {
                found_next = true;
                // creat New Node which include chooseParent and rewire
                creatNewNode(p_new.first, p_new.second, node_nearest.node_id);
                // dynamic Radius
                R_count++;
                if (R_count == deltaCount_) {
                    radius_ += deltaR_;
                    R_count = 0;
                }
            }
        }

        // goal not reached
        if (!goal_reached_) {
            if (isGoalReached(p_new)) {
                goal_reached_ = true;
                // goal_node_ = nodes_.at(node_count_ - 1); // update goal_node_
            }
        }
        // goal reached firstly
        if (goal_reached_ && node_count_ >= min_num_nodes_) {
            if (goal_reached_cnt == false) {
                // reached firstly
                // only once
                goal_reached_cnt = true;
                // add goalNode (let its leaf == 1 will not be removed)
                Node goal_node = Node(goal_point_.first, goal_point_.second, node_count_, node_count_ - 1);
                goal_node.leaf = 1;
                nodes_.push_back(goal_node); // insert
                nodes_fn_.push_back(node_count_);
                goal_node_id = node_count_; // record goal_node_id

                // chooseParent and rewire
                chooseParent(node_count_, node_count_ - 1);
                rewire(node_count_);
                // current_cost = nodes_.at(goal_node_.node_id).cost; // update current_cost
                node_count_++;
            }
            current_cost = nodes_.at(goal_node_id).cost; // update current_cost
        }

        // remove
        // if no reached, random remove
        // if reached, remove no use node(if has these node)
        force_remove();
        // ROS_WARN("NODES SIZE: %ld , NODES_FN SIZE: %ld", nodes_.size(), nodes_fn_.size());
    }

    if (goal_reached_) {
        // computeFinalPath
        computeFinalPath(path);
        return true;
    }

    // ROS_INFO("nodes size: %d", (int)nodes_.size());
    return false;
}

std::pair<double, double> ImRRTStarFN::sampleFree() {
    // use random_double_gen_ generate two random number
    return random_double_gen_.generate_all();
}
std::pair<double, double> ImRRTStarFN::sampleBall() {
    // use random_double_gen_ generate two random number in a circle
    return random_double_gen_.generate_ball(start_point_, goal_point_, current_cost);
}

int ImRRTStarFN::getNearestNodeId(const std::pair<double, double> &point) {
    double dist_nearest = 0.0 , dist = 0.0;
    Node node_nearest = nodes_.front(); // start_node
    // find
    for (const auto& i : nodes_fn_) {
        Node node = nodes_[i];
        dist_nearest = euclideanDistance2D(node_nearest.x, node_nearest.y, point.first, point.second);
        dist = euclideanDistance2D(node.x, node.y, point.first, point.second);
        if(dist < dist_nearest) {
            node_nearest = node;
        }
    }
    return node_nearest.node_id;
}

std::pair<double, double> ImRRTStarFN::steer(double Node_NearestX, double Node_NearestY, double Node_randomX, double Node_randomY) {
    std::pair<double, double> p_new;
    double dist = euclideanDistance2D(Node_NearestX, Node_NearestY, Node_randomX, Node_randomY);
    // no step
    if (dist < epsilon_) {
        p_new.first = Node_NearestX;
        p_new.second = Node_NearestY;
        return p_new;
    }
    else {
        double theta = atan2(Node_randomY - Node_NearestY, Node_randomX - Node_NearestX);
        p_new.first = Node_NearestX + epsilon_ * cos(theta);
        p_new.second = Node_NearestY + epsilon_ * sin(theta);
        return p_new;
    }
}

void ImRRTStarFN::creatNewNode(double x, double y, int node_nearest_id) {
    Node new_node(x, y, node_count_, node_nearest_id);
    nodes_.push_back(new_node);
    nodes_fn_.push_back(node_count_);

    if(node_nearest_id != -1) {
        // choose parent according to cost
        chooseParent(node_count_, node_nearest_id);
        // rewire
        rewire(node_count_);
    }

    node_count_++;
}

void ImRRTStarFN::chooseParent(int new_node_id, int node_nearest_id) {
    double new_node_cost;
    double other_node_cost;
    double node_dist;

    Node parent_node = nodes_.at(node_nearest_id);

    Node& new_node = nodes_.at(new_node_id); // reference

    for(const auto& i : nodes_fn_) {
        Node node = nodes_[i];
        if(node.node_id == new_node.node_id) continue;

        node_dist = euclideanDistance2D(node.x, node.y, new_node.x, new_node.y);
        // if in circle R
        if (node_dist < radius_) {
            // current cost
            new_node_cost = parent_node.cost + euclideanDistance2D(new_node.x, new_node.y, parent_node.x, parent_node.y);
            // other node cost
            other_node_cost = node.cost + node_dist;
            if (other_node_cost < new_node_cost) {
                if (!cd_.isThereObstacleBetween(node, new_node)) {
                    parent_node = node;
                }
            }
        }
    }

    // update new_node cost, h_cost and parent
    new_node.cost = parent_node.cost + euclideanDistance2D(new_node.x, new_node.y, parent_node.x, parent_node.y);
    new_node.h_cost = euclideanDistance2D(goal_point_.first, goal_point_.second, new_node.x, new_node.y);
    new_node.parent_id = parent_node.node_id;
    // update leaf
    nodes_.at(new_node.parent_id).leaf++;
    nodes_.at(new_node.parent_id).child_id.push_back(new_node.node_id);
}

// parent node's leaf - 1
void ImRRTStarFN::changeParentLeaf(int parent_id, int child_id) {
    nodes_.at(parent_id).leaf--;
    auto iter = std::find(nodes_.at(parent_id).child_id.begin(), nodes_.at(parent_id).child_id.end(), child_id);
    nodes_.at(parent_id).child_id.erase(iter);
}

void ImRRTStarFN::rewire(int new_node_id) {
    double node_dist;
    double new_Node_AsParent_cost;

    Node& new_node = nodes_.at(new_node_id); // reference

    for(const auto& i : nodes_fn_) {
        Node& node = nodes_[i]; //reference
        if (node == new_node) continue;

        node_dist = euclideanDistance2D(node.x, node.y, new_node.x, new_node.y);
        
        // if is not parent node and in circle R
        if (node != nodes_.at(new_node.parent_id) && node_dist < radius_) {
            new_Node_AsParent_cost = new_node.cost + euclideanDistance2D(node.x, node.y, new_node.x, new_node.y);
            // if new_node as parent, cost decrease
            if (new_Node_AsParent_cost < node.cost && !cd_.isThereObstacleBetween(new_node, node)) {
                // // update parent
                changeParentLeaf(node.parent_id, node.node_id);
                // nodes_.at(node.parent_id).leaf--;
                // auto delete_iter = std::find(nodes_.at(node.parent_id).child_id.begin(), nodes_.at(node.parent_id).child_id.end(), node.node_id);
                // // find success and erase it
                // if (delete_iter != nodes_.at(node.parent_id).child_id.end()) {
                //     nodes_.at(node.parent_id).child_id.erase(delete_iter);
                // }
                
                // update node
                node.parent_id = new_node.node_id;
                node.cost = new_Node_AsParent_cost;
                __updateCost(node.node_id); // update tree(this tree root is node) cost

                // update new_node leaf
                new_node.leaf++;
                new_node.child_id.push_back(node.node_id);
            }

        }

    }
}

// update cost function
void ImRRTStarFN::__updateCost(int rootNode_id) {
    // child_id array
    std::vector<int> childs = nodes_.at(rootNode_id).child_id; 
    // recursion condition
    if (childs.size() > 0) {
        // update childs cost
        for (const auto& child : childs) {
            nodes_.at(child).cost = nodes_.at(rootNode_id).cost 
                                + euclideanDistance2D(nodes_.at(child).x, nodes_.at(child).y, nodes_.at(rootNode_id).x, nodes_.at(rootNode_id).y);
            // recursion (child as rootNode)
            __updateCost(child);
        }
    }
}

// remove node
void ImRRTStarFN::force_remove() {
    // don't remove
    if (node_count_ <= fixNode_) {
        return;
    }
    // find leaf node
    std::vector<int> leaf_Node;
    int max_cost_id = -1;
    double max_cost = current_cost;
    for (const auto& i : nodes_fn_) {
        Node node = nodes_[i];
        // is leaf Node ?
        if (node.leaf <= 0) {
            leaf_Node.push_back(node.node_id);
            // find (max cost > current_cost) Node
            if (current_cost > 0 && (node.cost + node.h_cost) > max_cost) {
                max_cost_id = node.node_id;
                max_cost = node.cost + node.h_cost;
            }
        }
    }

    // has max_cost Node and remove it
    if (max_cost_id >= 0) {
        // update its parent
        int parent = nodes_.at(max_cost_id).parent_id;
        changeParentLeaf(parent, max_cost_id);
        // nodes_.at(parent).leaf--;
        // // delete it from its parent
        // auto delete_iter = std::find(nodes_.at(parent).child_id.begin(), nodes_.at(parent).child_id.end(), max_cost_id);
        // // find success and erase it
        // if (delete_iter != nodes_.at(parent).child_id.end()) {
        //     nodes_.at(parent).child_id.erase(delete_iter);
        // }

        // delete it from nodes_fn_
        auto iter = std::find(nodes_fn_.begin(), nodes_fn_.end(), max_cost_id);
        nodes_fn_.erase(iter);
        return;
    }

    // hasn't max_cost Node
    // remove a random leaf Node
    int index = random_double_gen_.generate_int(leaf_Node.size() - 1);
    int delete_key = leaf_Node[index];
    // update its parent
    int parent = nodes_.at(delete_key).parent_id;
    changeParentLeaf(parent, delete_key);
    // nodes_.at(parent).leaf--;
    // // delete it from its parent
    // auto delete_iter = std::find(nodes_.at(parent).child_id.begin(), nodes_.at(parent).child_id.end(), delete_key);
    // // find success and erase it
    // if (delete_iter != nodes_.at(parent).child_id.end()) {
    //     nodes_.at(parent).child_id.erase(delete_iter);
    // }
    // delete it from nodes_fn_
    auto iter = std::find(nodes_fn_.begin(), nodes_fn_.end(), delete_key);
    nodes_fn_.erase(iter);
    return;
}

void ImRRTStarFN::computeFinalPath(std::list<std::pair<double, double>>& path) {
    path.clear();
    // compute path from goal to start
    Node current_node = nodes_.at(goal_node_id);

    std::pair<double, double> pos;
    while (current_node.parent_id != -1) {

        pos.first = current_node.x;
        pos.second = current_node.y;
        path.push_front(pos);

        // update current node
        current_node = nodes_.at(current_node.parent_id);
    }
    // put start point
    pos.first = current_node.x;
    pos.second = current_node.y;
    path.push_front(pos);
}

bool ImRRTStarFN::isGoalReached(const std::pair<double, double>& p_new) {
    return (euclideanDistance2D(p_new.first, 
                                p_new.second, 
                                goal_point_.first, 
                                goal_point_.second) < goal_tolerance_) ? true : false;
}

std::vector<Node> ImRRTStarFN::getNodes() const {
    std::vector<Node> res;
    for (const auto& i : nodes_fn_) {
        res.emplace_back(nodes_[i]);
    }
    return res;
}

};