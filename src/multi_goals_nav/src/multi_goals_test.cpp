#include "ros/ros.h"
#include <vector>
#include <string>
#include <math.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Point.h"
#include "move_base_msgs/MoveBaseActionFeedback.h"
#include <fstream>
#include <sstream>
/*
    问题思路：
    1.准备一个数组从txt文件读取多个目标点,消息类型是 geometry_msgs/PoseArray
    2.创建一个节点代替rviz发布话题 /move_base_simple/goal 消息类型是 geometry_msgs/PoseStamped
    3.订阅话题 /move_base/feedback 消息类型是 move_base_msgs/MoveBaseActionFeedback 判断是否到达当前目标点


    实现思路:
    1.包含头文件
    2.初始化ROS节点 创建句柄
    3.创建发布者对象
    4.发布第一个目标点    
    5.创建订阅者对象(回调函数中发布剩下的目标点)
    6.实现逻辑
    7.spin()
*/



geometry_msgs::PoseArray goalsArray;            //目标点位姿数组                    
int goal_id = 0;                                //当前目标点
geometry_msgs::PoseStamped goal_pose;           //当前发布的目标点位姿
double distOfcomplete = 1.0;                    //目标点到达范围

//目标点数组初始化
void goalsArray_INIT(){
    goalsArray.header.frame_id = "map";
    goalsArray.header.stamp = ros::Time::now();
    geometry_msgs::Pose pose;
    
    //从txt文件读取pose数据
    std::ifstream filein;
    filein.open("/home/lizy/demo_mynav/src/multi_goals_nav/goals.csv", std::ios::in);
    if(!filein.is_open()){
        ROS_ERROR("NO TXT");
        return;
    }
    //按行读入
    std::string buff;
    std::vector<std::string> v;
    while(std::getline(filein, buff)){
        // 按逗号分隔每个数
        std::istringstream record(buff);
        std::string word;
        while(std::getline(record, word, ',')){
            v.push_back(word);
        }

        pose.position.x = std::atof(v[0].c_str());
        pose.position.y = std::atof(v[1].c_str());
        pose.position.z = std::atof(v[2].c_str());

        pose.orientation.x = std::atof(v[3].c_str());
        pose.orientation.y = std::atof(v[4].c_str());
        pose.orientation.z = std::atof(v[5].c_str());
        pose.orientation.w = std::atof(v[6].c_str());
        
        goalsArray.poses.push_back(pose);

        v.clear();
    }

    filein.close();
}

//判断与目标点的距离是否小于0.1m
bool distance_arrived(geometry_msgs::Point base_point, geometry_msgs::Point &goal_point){
    double dist = sqrt(pow(abs(base_point.x - goal_point.x), 2) + 
                        pow(abs(base_point.y - goal_point.y), 2) + 
                        pow(abs(base_point.z - goal_point.z), 2));
    
    if(dist <= distOfcomplete){
        return true;
    }
    else{
        return false;
    }
}

void goal_id_Change(const move_base_msgs::MoveBaseActionFeedback::ConstPtr &fb, ros::Publisher &goal_pub){
    bool arrived_ = distance_arrived(fb->feedback.base_position.pose.position, goalsArray.poses[goal_id].position);
    //6.实现逻辑
    if(arrived_){
        ROS_INFO("goal_id:%d is arrived", goal_id + 1);
        goal_id += 1;
        goal_id %= goalsArray.poses.size();//循环发布目标点
        // if(goal_id == goalsArray.poses.size()){
        //     ros::shutdown();
        // }
        
        
        goal_pose.header.frame_id = "map";
        goal_pose.header.stamp = ros::Time::now();
        goal_pose.pose = goalsArray.poses[goal_id];

        int cnt = 1;
        ros::Rate rate(10);
        ROS_INFO("pub next goal");
        while(cnt <= 30 && ros::ok()){
            goal_pub.publish(goal_pose);
            cnt++;
            rate.sleep();
        }
    }
    
}

int main(int argc, char *argv[])
{
    //2.初始化ROS节点 创建句柄
    ros::init(argc, argv, "multi_goals_pub");
    ros::NodeHandle nh;
    //目标点数组初始化
    goalsArray_INIT();
    if(goalsArray.poses.size() == 0){
        ROS_ERROR("NO GOAL");
        ros::shutdown();
    }
    // 3.创建发布者对象
    ros::Publisher goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
    
    // 4.发布第一个目标点
    ros::Rate rate(10);
    //第一个目标点信息
    goal_pose.header.frame_id = "map";
    goal_pose.header.stamp = ros::Time::now();
    goal_pose.pose = goalsArray.poses[goal_id];
    int cnt = 1;
    //ros::Duration(5).sleep();//休眠5s等待move_base准备就绪
    while(ros::ok()){

        goal_pub.publish(goal_pose);

        rate.sleep();
        cnt++;
        if(cnt > 30){
            break;
        }
    }

    // 5.创建订阅者对象
    ros::Subscriber base_state_sub = nh.subscribe<move_base_msgs::MoveBaseActionFeedback>("/move_base/feedback", 1, boost::bind(&goal_id_Change, _1, goal_pub));
    
    // 7.spin()
    ros::spin();
    return 0;
}
