#ifndef RRT_STAR_GLOBAL_PLANNER_RRT_STAR_H_
#define RRT_STAR_GLOBAL_PLANNER_RRT_STAR_H_

#include <ros/ros.h>
#include <costmap_2d/costmap_2d.h>

#include <cmath>
#include <string>
#include <vector>
#include <list>
#include <utility>

#include "Planner/node.h"
#include "Planner/random_double_generator.h"
#include "Planner/collision_detector.h"


namespace rrt_star_Global_planner {

class RRTStar {
private:
    std::pair<float, float> start_point_;
    std::pair<float, float> goal_point_;
    costmap_2d::Costmap2D* costmap_{nullptr};
    std::vector<Node> nodes_;
    RandomDoubleGenerator random_double_gen_;
    int node_count_{0};
    float map_width_;
    float map_height_;
    double radius_;
    unsigned int max_num_nodes_;
    unsigned int min_num_nodes_;
    double goal_tolerance_;
    double epsilon_; // step length

    bool goal_reached_{false};

    Node goal_node_;

    CollisionDetector cd_;

public:

    RRTStar(const std::pair<float, float>& start_point, 
            const std::pair<float, float>& goal_point,
            costmap_2d::Costmap2D* costmap, 
            double goal_tolerance, 
            double radius, 
            double epsilon, 
            unsigned int max_num_nodes, 
            unsigned int min_num_nodes, 
            float map_width, 
            float map_height);

    // compute the RRT* path planning
    // path : list of positions (x, y)
    bool pathPlanning(std::list<std::pair<float, float>> &path);

    // compute random points
    // return position (x, y)
    std::pair<float, float> sampleFree();

    // Get the Index of the nearest node around the new random point
    // point : random point
    // return nearest node index
    int getNearestNodeId(const std::pair<float, float> &point);

    // Expend in direction (x1, y1) ->> (x2, y2) use epsilon_
    // return a new position in this direction
    std::pair<float, float> steer(float x1, float y1, float x2, float y2);

    // Creat a new node in tree
    // node_nearest_id is new node parent
    void creatNewNode(float x, float y, int node_nearest_id);

    // Selection of the parent node with lowest cost 
    // inside of the circular area of the new node
    void chooseParent(int node_nearest_id);

    // rewire the best parent inside of the circular area of the new node
    void rewire();
    // update cost function
    void __updateCost(int rootNode_id);

    // from goal node to start node , store path
    void computeFinalPath(std::list<std::pair<float, float>>& path);

    bool isGoalReached(const std::pair<float, float>& p_new);

    std::vector<Node> getNodes() const;


};

};




#endif