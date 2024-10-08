#ifndef MYLOCAL_PLANNER_H
#define MYLOCAL_PLANNER_H

#include <ros/ros.h>
#include <nav_core/base_local_planner.h>
#include <tf2_ros/buffer.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>


namespace myLocal_planner {

class Local_planner : public nav_core::BaseLocalPlanner {
public:
    Local_planner();
    Local_planner(std::string name, tf2_ros::Buffer* tf,
                 costmap_2d::Costmap2DROS* costmap_ros);
    ~Local_planner();
    
    void initialize(std::string name, tf2_ros::Buffer* tf, 
                     costmap_2d::Costmap2DROS* costmap_ros);
    bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);
    bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);
    bool isGoalReached();


private:
    costmap_2d::Costmap2DROS* costmap_ros_;
    tf2_ros::Buffer* tf_;
    bool initialized_;
};




};


#endif