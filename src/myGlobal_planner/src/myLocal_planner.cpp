#include <pluginlib/class_list_macros.h>
#include "myGlobal_planner/myLocal_planner.h"



PLUGINLIB_EXPORT_CLASS(myLocal_planner::Local_planner, nav_core::BaseLocalPlanner)


namespace myLocal_planner {

Local_planner::Local_planner() : costmap_ros_(nullptr), tf_(nullptr), initialized_(false) {
    ROS_INFO("This is Local_planner default constructor ");
}

Local_planner::Local_planner(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
     : costmap_ros_(nullptr), tf_(nullptr), initialized_(false) {
    initialize(name, tf, costmap_ros);
}

Local_planner::~Local_planner() {}

void Local_planner::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros) {
    ROS_INFO("This is Local_planner initialize ");

}

bool Local_planner::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan){
    return false;
}

bool Local_planner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel){
    return false;
}

bool Local_planner::isGoalReached(){
    ROS_INFO("This is GoalReached ");
    return true;
}

};