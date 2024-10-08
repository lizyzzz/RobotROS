#include <pluginlib/class_list_macros.h>
#include "myGlobal_planner/myGlobal_planner.h"

// register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(myGlobal_planner::Global_planner, nav_core::BaseGlobalPlanner)


namespace myGlobal_planner {

Global_planner::Global_planner(){
    ROS_INFO("hello, this is Global_planner default constructor ");
}

Global_planner::Global_planner(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
    initialize(name, costmap_ros);
}

void Global_planner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
    ROS_INFO("hello, this is initialize ");
}

bool Global_planner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan){
    plan.push_back(start);
    // ROS_INFO("hello, this is makePlan ");
    // 核心代码
    
    for(int i = 0; i < 20; i++){
        geometry_msgs::PoseStamped new_goal = goal;

        tf::Quaternion goal_quat = tf::createQuaternionFromYaw(1.54);

        new_goal.pose.position.x = -2.5 + (0.05 * i);
        new_goal.pose.position.y = -3.5 + (0.05 * i);

        new_goal.pose.orientation.x = goal_quat.getX();
        new_goal.pose.orientation.y = goal_quat.getY();
        new_goal.pose.orientation.z = goal_quat.getZ();
        new_goal.pose.orientation.w = goal_quat.getW();

        plan.push_back(new_goal);
    }

    plan.push_back(goal);
    return true;
}

Global_planner::~Global_planner() { }

};

