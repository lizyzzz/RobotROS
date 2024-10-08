#include "ImRRTStarFN/collision_detector.h"

namespace rrt_starFN_Global_planner {

CollisionDetector::CollisionDetector(costmap_2d::Costmap2D* costmap) : costmap_(costmap) {
    if(costmap_ != nullptr) {
        resolution_ = costmap_->getResolution();
        origin_x_ = costmap_->getOriginX(); // x origin
        origin_y_ = costmap_->getOriginY(); // y origin
    }
}

void CollisionDetector::worldToMap(float wx, float wy, int& mx, int& my){
    if(costmap_ != nullptr) {
        mx = (wx - origin_x_) / resolution_;
        my = (wy - origin_y_) / resolution_;
    }
}

// a point is occupied ? 
bool CollisionDetector::isThisPointCollides(float wx, float wy) {
    // In case of no costmap loaded
    if(costmap_ == nullptr) {
        // no collision
        return false;
    }

    int mx = -1, my = -1;
    worldToMap(wx, wy, mx, my);

    // overstep
    if(mx < 0 || my < 0 || (mx >= costmap_->getSizeInCellsX()) || (my >= costmap_->getSizeInCellsY())) {
        return true;
    }

    unsigned int cost = static_cast<unsigned int>(costmap_->getCost(mx, my));
    // adj
    unsigned int L = static_cast<unsigned int>(costmap_->getCost( mx - 1, my));
    unsigned int LU = static_cast<unsigned int>(costmap_->getCost(mx - 1, my + 1));
    unsigned int U = static_cast<unsigned int>(costmap_->getCost( mx,     my + 1));
    unsigned int RU = static_cast<unsigned int>(costmap_->getCost(mx + 1, my + 1));
    unsigned int R = static_cast<unsigned int>(costmap_->getCost( mx + 1, my));
    unsigned int RD = static_cast<unsigned int>(costmap_->getCost(mx + 1, my - 1));
    unsigned int D = static_cast<unsigned int>(costmap_->getCost( mx,     my - 1));
    unsigned int LD = static_cast<unsigned int>(costmap_->getCost(mx - 1, my - 1));
    

    unsigned int flag = cost + L + LU + U + RU + R + RD + D + LD;
    if(flag > 0) {
        return true;
    }

    return false;
}

// line between node and point is occupied ?
bool CollisionDetector::isThereObstacleBetween(const Node& node, const std::pair<double, double>& point) {
    // In case of no costmap loaded
    if(costmap_ == nullptr) {
        // no collision
        return false;
    }

    float dist = euclideanDistance2D(node.x, node.y, point.first, point.second);

    if (dist < resolution_) {
        return isThisPointCollides(point.first, point.second);
    }
    else {
        // this has some error
        // miss some cell
        int steps_number = static_cast<int>(floor(dist / resolution_));
        // it must be (point - node)
        // (point - node) and (node - point) have different values
        float theta = atan2(point.second - node.y, point.first - node.x);
        
        std::pair<float, float> p_n;
        for(int i = 1; i < steps_number; ++i) {
            p_n.first = node.x + i * resolution_ * cos(theta);
            p_n.second = node.y + i * resolution_ * sin(theta);
            if(isThisPointCollides(p_n.first, p_n.second)) {
                return true;
            }
        }
        return false;
    }

}

// line between node and node is occupied ?
bool CollisionDetector::isThereObstacleBetween(const Node& node1, const Node& node2) {
    return isThereObstacleBetween(node1, std::make_pair(node2.x, node2.y));
}

};