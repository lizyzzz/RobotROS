#ifndef MYGLOBAL_PLANNER_H
#define MYGLOBAL_PLANNER_H

#include <ros/ros.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <angles/angles.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>
#include <vector>
#include <tf/transform_datatypes.h>


namespace myGlobal_planner {

// 继承 nav_core::BaseGlobalPlanner
class Global_planner : public nav_core::BaseGlobalPlanner {
public:
    Global_planner();
    Global_planner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
    ~Global_planner();
    // 重写 继承来的函数
    void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
    bool makePlan(const geometry_msgs::PoseStamped& start, 
                  const geometry_msgs::PoseStamped& goal,
                  std::vector<geometry_msgs::PoseStamped>& plan);

private:
    costmap_2d::Costmap2DROS* costmap_ros_;
    bool initialized_;
    
};


};

#endif