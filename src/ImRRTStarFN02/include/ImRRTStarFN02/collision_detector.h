#ifndef RRT_STARFN_GLOBAL_PLANNER_COLLISION_DETECTOR_H_
#define RRT_STARFN_GLOBAL_PLANNER_COLLISION_DETECTOR_H_

#include <costmap_2d/costmap_2d.h>
#include <utility>
#include "ImRRTStarFN02/node.h"

namespace rrt_starFN02_Global_planner {

class CollisionDetector {
private:
    costmap_2d::Costmap2D* costmap_{nullptr};
    double resolution_{0.1};
    double origin_x_{0.0};
    double origin_y_{0.0};
    // world coordinates to map coordinates
    void worldToMap(float wx, float wy, int& mx, int& my);

public:

    explicit CollisionDetector(costmap_2d::Costmap2D* costmap);
    
    // a point is occupied ? 
    bool isThisPointCollides(float wx, float wy);
    
    // line between node and point is occupied ?
    bool isThereObstacleBetween(const Node& node, const std::pair<double, double>& point);

    // line between node and node is occupied ?
    bool isThereObstacleBetween(const Node& node1, const Node& node2);


};



};

#endif