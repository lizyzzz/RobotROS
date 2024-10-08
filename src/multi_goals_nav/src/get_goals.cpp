#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include <fstream>

/*
    问题:从rviz订阅话题/move_base_simple/goal 消息类型是 geometry_msgs/PoseStamped
    并写出到.txt文件

    实现思路:
    1.包含头文件
    2.初始化ROS节点 创建NodeHandle
    3.新建一个txt文件
    4.创建订阅者订阅话题/move_base_simple/goal
    5.处理订阅到的消息
    6.spin()
*/



// 5.处理订阅到的消息
void doMsg(const geometry_msgs::PoseStamped::ConstPtr &goal_pose){
    ROS_INFO("get goal position:x = %.2f,y = %.2f,z = %.2f", goal_pose->pose.position.x, goal_pose->pose.position.y, goal_pose->pose.position.z);
    ROS_INFO("get goal qtn:x = %.2f,y = %.2f,z = %.2f,w = %.2f", goal_pose->pose.orientation.x, goal_pose->pose.orientation.y, goal_pose->pose.orientation.z, goal_pose->pose.orientation.w);

    std::ofstream fout;
    fout.open("/home/lizy/demo_mynav/src/multi_goals_nav/goals.csv", std::ios::app);//以追加方式写入
    if(!fout.is_open()){
        ROS_ERROR("NO TXT");
        ros::shutdown();
    }
    
    //以行为单位写入文件 x y z x y z w
    fout << std::to_string(goal_pose->pose.position.x) <<","
        << std::to_string(goal_pose->pose.position.y) <<","
        << std::to_string(goal_pose->pose.position.z) <<","
        << std::to_string(goal_pose->pose.orientation.x) <<","
        << std::to_string(goal_pose->pose.orientation.y) <<","
        << std::to_string(goal_pose->pose.orientation.z) <<","
        << std::to_string(goal_pose->pose.orientation.w) <<","
        << std::endl;
    
    fout.close();
}

int main(int argc, char *argv[])
{
    // 2.初始化ROS节点 创建NodeHandle
    ros::init(argc, argv, "sub_goals");
    ros::NodeHandle nh;

    // 3.新建txt一个文件存放
    std::ofstream fileout;
    fileout.open("/home/lizy/demo_mynav/src/multi_goals_nav/goals.csv");
    if(!fileout.is_open()){
        ROS_ERROR("new file failed");
        ros::shutdown();
    }
    fileout.close();

    // 4.创建订阅者订阅话题/move_base_simple/goal
    ros::Subscriber sub = nh.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1, doMsg);
    

    // 6.spin()
    ros::spin();

    return 0;
}
