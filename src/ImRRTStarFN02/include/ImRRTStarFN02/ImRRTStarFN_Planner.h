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

#include "ImRRTStarFN02/ImRRTStarFN.h"


namespace rrt_starFN02_Global_planner {

// derive from nav_core::BaseGlobalPlanner
class ImRRTStarFN02_planner : public nav_core::BaseGlobalPlanner {
public:
    ImRRTStarFN02_planner();
    ImRRTStarFN02_planner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
    ImRRTStarFN02_planner(std::string name, costmap_2d::Costmap2D* costmap, std::string global_frame);
    ~ImRRTStarFN02_planner();
    // override virtual function
    // param : name  :  the name of the planner
    void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
    void initialize(std::string name, costmap_2d::Costmap2D* costmap, std::string global_frame);

    bool makePlan(const geometry_msgs::PoseStamped& start, 
                  const geometry_msgs::PoseStamped& goal,
                  std::vector<geometry_msgs::PoseStamped>& plan);
    
    // convert path to plan
    void computeFinalPlan(std::vector<geometry_msgs::PoseStamped>& plan, std::list<std::pair<double, double>>& path);
    // publish plan for visualization
    void publishPlan(std::vector<geometry_msgs::PoseStamped>& plan);

private:
    costmap_2d::Costmap2D* costmap_;
    int fixNode_;
    bool initialized_;
    int max_num_nodes_;
    int min_num_nodes_;
    double epsilon_;
    double map_width_;
    double map_height_;
    double radius_;
    double deltaR_;
    double deltaCount_;
    double goal_tolerence_;
    int dynamic_count_; // radius expand times
    int max_radius_factor_; // radius expand max length(max raidus == cost / max_radius_factor_)
    bool search_specific_area_{false};

    std::string global_frame_;
    std::shared_ptr<ImRRTStarFN> planner_;
    ros::Publisher plan_pub_;
};


};

#endif