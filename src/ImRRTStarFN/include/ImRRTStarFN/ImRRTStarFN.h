#ifndef RRT_STARFN_GLOBAL_PLANNER_RRT_STAR_H_
#define RRT_STARFN_GLOBAL_PLANNER_RRT_STAR_H_

#include <ros/ros.h>
#include <costmap_2d/costmap_2d.h>

#include <cmath>
#include <string>
#include <unordered_map>
#include <list>
#include <utility>
#include <algorithm>

#include "ImRRTStarFN/node.h"
#include "ImRRTStarFN/random_double_generator.h"
#include "ImRRTStarFN/collision_detector.h"


namespace rrt_starFN_Global_planner {

class ImRRTStarFN {
private:
    std::pair<double, double> start_point_;
    std::pair<double, double> goal_point_;
    double current_cost{-1};
    costmap_2d::Costmap2D* costmap_{nullptr};
    std::unordered_map<int, Node> nodes_; // fixNode map
    int fixNode_;
    RandomDoubleGenerator random_double_gen_;
    int node_count_{0};
    double map_width_;
    double map_height_;
    double radius_;
    double deltaR_;
    int deltaCount_;
    unsigned int max_num_nodes_;
    unsigned int min_num_nodes_;
    double goal_tolerance_;
    double epsilon_; // step length
    int dynamic_count_; // radius expand times
    int max_radius_factor_; // radius expand max length(max raidus == cost / max_radius_factor_)

    bool goal_reached_{false};

    Node goal_node_;

    CollisionDetector cd_;

public:

    ImRRTStarFN(const std::pair<double, double>& start_point, 
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
            double map_height);

    // compute the RRT* path planning
    // path : list of positions (x, y)
    bool pathPlanning(std::list<std::pair<double, double>> &path);

    // compute random points
    // return position (x, y)
    std::pair<double, double> sampleFree();
    std::pair<double, double> sampleBall();

    // Get the Index of the nearest node around the new random point
    // point : random point
    // return nearest node index
    int getNearestNodeId(const std::pair<double, double> &point);

    // Expend in direction (x1, y1) ->> (x2, y2) use epsilon_
    // return a new position in this direction
    std::pair<double, double> steer(double x1, double y1, double x2, double y2);

    // Creat a new node in tree
    // node_nearest_id is new node parent
    void creatNewNode(double x, double y, int node_nearest_id);

    // Selection of the parent node with lowest cost 
    // inside of the circular area of the new node
    void chooseParent(int new_node_id, int node_nearest_id);

    // parent node's leaf - 1
    void changeParentLeaf(int parent_id, int child_id);

    // rewire the best parent inside of the circular area of the new node
    void rewire(int new_node_id);
    // update cost function
    void __updateCost(int rootNode_id);

    // remove node
    void force_remove();

    // from goal node to start node , store path
    void computeFinalPath(std::list<std::pair<double, double>>& path);

    bool isGoalReached(const std::pair<double, double>& p_new);

    std::vector<Node> getNodes() const;


};

};




#endif