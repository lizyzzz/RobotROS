#include <pluginlib/class_list_macros.h>
#include "Planner/rrt_star_planner.h"

// register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(rrt_star_Global_planner::RRTStar_planner, nav_core::BaseGlobalPlanner)


namespace rrt_star_Global_planner {

RRTStar_planner::RRTStar_planner() : costmap_(nullptr), initialized_(false) {
    ROS_INFO("--- Hello, this is RRTStar_planner default constructor ---");
}

RRTStar_planner::RRTStar_planner(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
                                    : costmap_(nullptr), initialized_(false) {
    // initialize the planner
    initialize(name, costmap_ros);
}

RRTStar_planner::RRTStar_planner(std::string name, costmap_2d::Costmap2D* costmap, std::string global_frame) 
                                    : costmap_(nullptr), initialized_(false) {
    // initialize the planner
    initialize(name, costmap, global_frame);
}

// param : name  :  the name of the planner
void RRTStar_planner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
    initialize(name, costmap_ros->getCostmap(), costmap_ros->getGlobalFrameID());
}

void RRTStar_planner::initialize(std::string name, costmap_2d::Costmap2D* costmap, std::string global_frame) {
    if(!initialized_) {
        costmap_ = costmap;
        global_frame_ = global_frame;

        // ROS_INFO("name: %s", name.c_str());
        // get params from service
        ros::NodeHandle nh_(name);
        nh_.param("goal_tolerence", goal_tolerence_, 0.5);
        nh_.param("radius", radius_, 1.0);
        nh_.param("epsilon", epsilon_, 0.2);
        nh_.param("max_num_nodes", max_num_nodes_, 5000);
        nh_.param("min_num_nodes", min_num_nodes_, 100);
        nh_.param("search_specific_area", search_specific_area_, true);
        
        // get params from service
        // ros::NodeHandle nh_;
        // nh_.param("/" + name + "/goal_tolerence", goal_tolerence_, 0.5);
        // nh_.param("/" + name + "/radius", radius_, 1.0);
        // nh_.param("/" + name + "/epsilon", epsilon_, 0.2);
        // nh_.param("/" + name + "/max_num_nodes", max_num_nodes_, 5000);
        // nh_.param("/" + name + "/min_num_nodes", min_num_nodes_, 100);


        ROS_INFO("max_num_nodes: %d", max_num_nodes_);

        // plan_pub_ initialize
        plan_pub_ = nh_.advertise<nav_msgs::Path>("RRTStar_Global_plan", 1);

        if (search_specific_area_) {
            nh_.param("map_width", map_width_, 5.0f);
            nh_.param("map_height", map_height_, 5.0f);
            // map_width_ = 10.0;
            // map_height_ = 10.0;
            ROS_INFO("map_width and height : ( %.2f, %.2f )", map_width_, map_height_);
        }
        else {
            map_width_ = costmap_->getSizeInMetersX();
            map_height_ = costmap_->getSizeInMetersY();
            ROS_INFO("map_width and height : ( %.2f, %.2f )", map_width_, map_height_);
        }

        ROS_INFO("RRTStar Global Planner initialized successfully.");
        initialized_ = true;
    }
    else {
        ROS_WARN("This RRTStar planner has already been initialized... doing nothing.");
    }
    
}

void RRTStar_planner::computeFinalPlan(std::vector<geometry_msgs::PoseStamped>& plan, std::list<std::pair<float, float>>& path) {
    plan.clear();
    geometry_msgs::PoseStamped pose;
    // header
    pose.header.frame_id = global_frame_;
    pose.header.stamp = ros::Time::now();
    // orientation
    tf::Quaternion quat = tf::createQuaternionFromRPY(0.0, 0.0, 0.0);

    // convert points to pose
    for(const auto& point : path) {
        pose.pose.position.x = point.first;
        pose.pose.position.y = point.second;
        pose.pose.position.z = 0;

        pose.pose.orientation.x = quat.getX();
        pose.pose.orientation.y = quat.getY();
        pose.pose.orientation.z = quat.getZ();
        pose.pose.orientation.w = quat.getW();

        plan.push_back(pose);
    }

}

bool RRTStar_planner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan){
    
    plan.clear();

    ROS_INFO(" --- RRTStar Global planner ---- ");
    ROS_INFO("Start Position: ( %.2f, %.2f )", start.pose.position.x, start.pose.position.y);
    ROS_INFO("GOAL Position: ( %.2f, %.2f )", goal.pose.position.x, goal.pose.position.y);

    std::pair<float, float> start_point = {start.pose.position.x, start.pose.position.y};
    std::pair<float, float> goal_point = {goal.pose.position.x, goal.pose.position.y};
    
    // planner_ initialize
    planner_ = std::shared_ptr<RRTStar>(new RRTStar(start_point,
                                                    goal_point,
                                                    costmap_,
                                                    goal_tolerence_,
                                                    radius_,
                                                    epsilon_,
                                                    max_num_nodes_,
                                                    min_num_nodes_,
                                                    map_width_,
                                                    map_height_));

    std::list<std::pair<float, float>> path;
    // find path
    if (planner_->pathPlanning(path)) {
        ROS_INFO("RRTStar Global Planner: Path found!!!!");
        // convert path to plan
        computeFinalPlan(plan, path);
        plan.push_back(goal);
        // publish plan
        publishPlan(plan);
        return true;
    }
    else {
        ROS_WARN("RRTStar Global Planner fail to find path, choose other goal.");
        return false;
    }
}

void RRTStar_planner::publishPlan(std::vector<geometry_msgs::PoseStamped>& plan) {
    if (!initialized_) {
        ROS_ERROR("This planner has not been initialized yet.");
        return;
    }

    // creat a message for plan
    nav_msgs::Path gui_path;
    gui_path.poses.reserve(plan.size());
    // header
    if (plan.empty()) {
        gui_path.header.frame_id = global_frame_;
        gui_path.header.stamp = ros::Time::now();
    }
    else {
        gui_path.header.frame_id = plan[0].header.frame_id;
        gui_path.header.stamp = plan[0].header.stamp;
    }

    // poses
    for (const auto& pose : plan) {
        gui_path.poses.push_back(pose);
    }
    
    plan_pub_.publish(gui_path);
}

RRTStar_planner::~RRTStar_planner() { }

};

