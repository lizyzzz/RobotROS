#ifndef GLOBAL_PLANNER_H
#define GLOBAL_PLANNER_H

#include <ros/ros.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <tf/transform_datatypes.h>

#include <cmath>
#include <vector>
#include <string>
#include <list>
#include <utility>

#include "Planner/rrt_star.h"


namespace rrt_star_Global_planner {

// derive from nav_core::BaseGlobalPlanner
class RRTStar_planner : public nav_core::BaseGlobalPlanner {
public:
    RRTStar_planner();
    RRTStar_planner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
    RRTStar_planner(std::string name, costmap_2d::Costmap2D* costmap, std::string global_frame);
    ~RRTStar_planner();
    // override virtual function
    // param : name  :  the name of the planner
    void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
    void initialize(std::string name, costmap_2d::Costmap2D* costmap, std::string global_frame);

    bool makePlan(const geometry_msgs::PoseStamped& start, 
                  const geometry_msgs::PoseStamped& goal,
                  std::vector<geometry_msgs::PoseStamped>& plan);
    
    // convert path to plan
    void computeFinalPlan(std::vector<geometry_msgs::PoseStamped>& plan, std::list<std::pair<float, float>>& path);
    // publish plan for visualization
    void publishPlan(std::vector<geometry_msgs::PoseStamped>& plan);

private:
    costmap_2d::Costmap2D* costmap_;
    bool initialized_;
    int max_num_nodes_;
    int min_num_nodes_;
    double epsilon_;
    float map_width_;
    float map_height_;
    double radius_;
    double goal_tolerence_;
    bool search_specific_area_{false};
    std::string global_frame_;
    std::shared_ptr<RRTStar> planner_;
    ros::Publisher plan_pub_;
};


};

#endif