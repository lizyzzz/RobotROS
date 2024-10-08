#include "Planner/rrt_star.h"


namespace rrt_star_Global_planner {

RRTStar::RRTStar(const std::pair<float, float>& start_point, 
            const std::pair<float, float>& goal_point,
            costmap_2d::Costmap2D* costmap, 
            double goal_tolerance, 
            double radius, 
            double epsilon, 
            unsigned int max_num_nodes, 
            unsigned int min_num_nodes, 
            float map_width, 
            float map_height) : start_point_(start_point),
                                goal_point_(goal_point),
                                costmap_(costmap),
                                goal_tolerance_(goal_tolerance),
                                radius_(radius),
                                epsilon_(epsilon),
                                max_num_nodes_(max_num_nodes),
                                min_num_nodes_(min_num_nodes),
                                map_width_(map_width),
                                map_height_(map_height),
                                cd_(costmap)
            { 
                nodes_.reserve(max_num_nodes_);
                // set range
                // random_double_gen_.setRange(-map_width_, map_width_, 0);
                // random_double_gen_.setRange(-map_height_, map_height_, 1);
                random_double_gen_.setRange(0, map_width_, 0);
                random_double_gen_.setRange(0, map_height_, 1);
            }

// core algorithm
bool RRTStar::pathPlanning(std::list<std::pair<float, float>> &path) {
    goal_reached_ = false;

    // this goal is collides
    if (cd_.isThisPointCollides(goal_point_.first, goal_point_.second)) {
        ROS_ERROR("This goal is not in Free Space. Choose other goal");
        return false;
    }
    // start point
    creatNewNode(start_point_.first, start_point_.second, -1);

    std::pair<float, float> p_rand;
    std::pair<float, float> p_new;
    Node node_nearest;

    bool found_next;
    // while (nodes_.size() < max_num_nodes_) {
    //     found_next = false;
    //     // random sample
    //     while (!found_next) {
    //         // generate random point
    //         p_rand = sampleFree();
    //         // p_rand.first = random_double_gen_.generate();
    //         // p_rand.second = random_double_gen_.generate();
    //         // std::cout << "prand: " << p_rand.first << " " << p_rand.second << std::endl;
            
    //         // get NearestNode
    //         node_nearest = nodes_[getNearestNodeId(p_rand)];
    //         // one step in random point direction
    //         p_new = steer(node_nearest.x, node_nearest.y, p_rand.first, p_rand.second);

    //         // no obstacle
    //         if (!cd_.isThereObstacleBetween(node_nearest, p_new)) {
    //             found_next = true;
    //             // creat New Node which include chooseParent and rewire
    //             creatNewNode(p_new.first, p_new.second, node_nearest.node_id);
    //         }
    //     }

    //     // goal not reached
    //     if (!goal_reached_) {
    //         if (isGoalReached(p_new)) {
    //             goal_reached_ = true;
    //             goal_node_ = nodes_.back(); // update goal_node_
    //         }
    //     }
    //     // goal reached and compute path
    //     if (goal_reached_ && nodes_.size() > min_num_nodes_) {
    //         // for(auto node : nodes_) {
    //         //     std::cout << "node id: " << node.node_id << "  parent id: " << node.parent_id << std::endl; 
    //         // }
    //         computeFinalPath(path);
    //         return true;
    //     }

    // }
    bool goal_reached_cnt = false;
    while (node_count_ < max_num_nodes_) {
        found_next = false;
        // random sample
        while (!found_next) {
            // generate random point
            p_rand = sampleFree();
            // p_rand.first = random_double_gen_.generate();
            // p_rand.second = random_double_gen_.generate();
            // std::cout << "prand: " << p_rand.first << " " << p_rand.second << std::endl;
            
            // get NearestNode
            node_nearest = nodes_[getNearestNodeId(p_rand)];
            // one step in random point direction
            p_new = steer(node_nearest.x, node_nearest.y, p_rand.first, p_rand.second);

            // no obstacle
            if (!cd_.isThereObstacleBetween(node_nearest, p_new)) {
                found_next = true;
                // creat New Node which include chooseParent and rewire
                creatNewNode(p_new.first, p_new.second, node_nearest.node_id);
            }
        }

        // goal not reached
        if (!goal_reached_) {
            if (isGoalReached(p_new)) {
                goal_reached_ = true;
                // goal_node_ = nodes_.back(); // update goal_node_
            }
        }
        // goal reached and compute path
        if (goal_reached_ && nodes_.size() > min_num_nodes_) {
            // for(auto node : nodes_) {
            //     std::cout << "node id: " << node.node_id << "  parent id: " << node.parent_id << std::endl; 
            // }
            if (goal_reached_cnt == false) {
                goal_reached_cnt = true;
                goal_node_ = Node(goal_point_.first, goal_point_.second, node_count_, node_count_ - 1);
                nodes_.push_back(goal_node_);
                chooseParent(node_count_ - 1);
                rewire();
                node_count_++;
            }
        }

    }
    
    if (goal_reached_) {
        // computeFinalPath
        computeFinalPath(path);
        return true;
    }
    // ROS_INFO("nodes size: %d", (int)nodes_.size());
    return false;
}

std::pair<float, float> RRTStar::sampleFree() {
    std::pair<float, float> random_point;
    // use random_double_gen_ generate two random number
    random_point.first = random_double_gen_.generate(0);
    random_point.second = random_double_gen_.generate(1);

    return random_point;
}

int RRTStar::getNearestNodeId(const std::pair<float, float> &point) {
    float dist_nearest = 0.0 , dist = 0.0;
    Node node_nearest = nodes_[0];
    // find
    for (int i = 1; i < nodes_.size(); ++i) {
        dist_nearest = euclideanDistance2D(node_nearest.x, node_nearest.y, point.first, point.second);
        dist = euclideanDistance2D(nodes_[i].x, nodes_[i].y, point.first, point.second);
        if(dist < dist_nearest) {
            node_nearest = nodes_[i];
        }
    }
    return node_nearest.node_id;
}

std::pair<float, float> RRTStar::steer(float Node_NearestX, float Node_NearestY, float Node_randomX, float Node_randomY) {
    std::pair<float, float> p_new;
    float dist = euclideanDistance2D(Node_NearestX, Node_NearestY, Node_randomX, Node_randomY);
    // no step
    if (dist < epsilon_) {
        p_new.first = Node_NearestX;
        p_new.second = Node_NearestY;
        return p_new;
    }
    else {
        float theta = atan2(Node_randomY - Node_NearestY, Node_randomX - Node_NearestX);
        p_new.first = Node_NearestX + epsilon_ * cos(theta);
        p_new.second = Node_NearestY + epsilon_ * sin(theta);
        return p_new;
    }
}

void RRTStar::creatNewNode(float x, float y, int node_nearest_id) {
    Node new_node(x, y, node_count_, node_nearest_id);
    nodes_.emplace_back(new_node);

    if(node_nearest_id != -1) {
        // choose parent according to cost
        chooseParent(node_nearest_id);
        // rewire
        rewire();
    }

    node_count_++;
}

void RRTStar::chooseParent(int node_nearest_id) {
    float new_node_cost;
    float other_node_cost;
    float node_dist;

    Node parent_node = nodes_[node_nearest_id];

    Node& new_node = nodes_.back();

    for(const auto & node : nodes_) {
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

    // update new_node cost and parent
    new_node.cost = parent_node.cost + euclideanDistance2D(new_node.x, new_node.y, parent_node.x, parent_node.y);
    new_node.parent_id = parent_node.node_id;
    // update parent
    nodes_[new_node.parent_id].child_id.push_back(new_node.node_id);
}

void RRTStar::rewire() {
    float node_dist;
    float new_Node_AsParent_cost;

    Node& new_node = nodes_.back();

    for(auto& node : nodes_) {
        if (node == new_node) continue;

        node_dist = euclideanDistance2D(node.x, node.y, new_node.x, new_node.y);
        
        // if is not parent node and in circle R
        if (node != nodes_[new_node.parent_id] && node_dist < radius_) {
            new_Node_AsParent_cost = new_node.cost + euclideanDistance2D(node.x, node.y, new_node.x, new_node.y);
            // if new_node as parent, cost decrease
            if (new_Node_AsParent_cost < node.cost && !cd_.isThereObstacleBetween(new_node, node)) {
                // update parent
                auto iter = std::find(nodes_[node.parent_id].child_id.begin(), nodes_[node.parent_id].child_id.end(), node.node_id);
                nodes_[node.parent_id].child_id.erase(iter);
                // update node
                node.parent_id = new_node.node_id;
                node.cost = new_Node_AsParent_cost;
                __updateCost(node.node_id);

                // update new_code
                new_node.child_id.push_back(node.node_id);
            }

        }

    }
}

// update cost function
void RRTStar::__updateCost(int rootNode_id) {
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

void RRTStar::computeFinalPath(std::list<std::pair<float, float>>& path) {
    path.clear();
    // compute path from goal to start 
    Node current_node = goal_node_;

    std::pair<float, float> pos;
    while (current_node.parent_id != -1) {

        pos.first = current_node.x;
        pos.second = current_node.y;
        path.push_front(pos);

        // update current node
        current_node = nodes_[current_node.parent_id];
    }
    // put start point
    pos.first = current_node.x;
    pos.second = current_node.y;
    path.push_front(pos);
}

bool RRTStar::isGoalReached(const std::pair<float, float>& p_new) {
    return (euclideanDistance2D(p_new.first, 
                                p_new.second, 
                                goal_point_.first, 
                                goal_point_.second) < goal_tolerance_) ? true : false;
}

std::vector<Node> RRTStar::getNodes() const {
    return nodes_;
}

};