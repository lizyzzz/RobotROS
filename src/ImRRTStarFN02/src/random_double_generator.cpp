#include "ImRRTStarFN02/random_double_generator.h"
#include <iostream>

namespace rrt_starFN02_Global_planner {

// XorY : 0 for X, others for Y
void RandomDoubleGenerator::setRange(double min, double max, int XorY){
    if(XorY == 0) {
        min_valueX = min;
        max_valueX = max;
    }
    else {
        min_valueY = min;
        max_valueY = max;
    }
    // std::cout << min_value << " " << max_value << std::endl;
}

// generate a point in all map
std::pair<double, double> RandomDoubleGenerator::generate_all(){
    std::mt19937 gen(rd_()); // use rd_ as seed
    
    // uniform_real_distribution does [start, stop)
    // So use nextafter get the next of max_value
    std::pair<double, double> res;
    res.first = std::uniform_real_distribution<double>{min_valueX, std::nextafter(max_valueX, DBL_MAX)}(gen);
    res.second = std::uniform_real_distribution<double>{min_valueY, std::nextafter(max_valueY, DBL_MAX)}(gen);
    return res;
}

// generate a point in ball
std::pair<double, double> RandomDoubleGenerator::generate_ball(std::pair<double, double>& start, std::pair<double, double>& goal, double c_max) {
    // Focal length
    double c_min = std::hypot((start.first - goal.second), (start.second - goal.second));

    // Semi-major axis
    double r1 = c_max / 2;
    // Semi-minor axis
    double r2 = std::sqrt(c_max * c_max - c_min * c_min) / 2;
    // center vector (z == 0)
    Eigen::Vector3d x_center((start.first + goal.first) / 2, (start.second + goal.second) / 2, 0);

    // Rotation matrix
    float theta = atan2(goal.second - start.second, goal.first - start.first); // z theta
    // 绕 z 轴旋转 theta 弧度的旋转矩阵 rotationMatrix
    Eigen::AngleAxisd rotationVector(theta, Eigen::Vector3d(0, 0, 1));
    Eigen::Matrix3d rotationMatrix = rotationVector.toRotationMatrix();

    // diag Matrix
    Eigen::Matrix3d L = Eigen::Matrix3d::Identity();
    L(0, 0) = r1;
    L(1, 1) = r2;
    L(2, 2) = 0;

    // random point in a circle
    std::mt19937 gen(rd_()); // use rd_ as seed
    std::uniform_real_distribution<double> u(0, 1);
    double r_rand = u(gen);
    double theta_rand = 2 * M_PI * u(gen);
    // random point
    Eigen::Vector3d x_ball(r_rand * std::cos(theta_rand), r_rand * std::sin(theta_rand), 0);
    // Linear transformation
    Eigen::Vector3d x_rand = rotationMatrix * L * x_ball + x_center;

    return std::pair<double, double>(x_rand(0), x_rand(1));
}

// generate a rand int number
int RandomDoubleGenerator::generate_int(int upper) {
    std::mt19937 gen(rd_()); // use rd_ as seed
    return std::uniform_int_distribution<int>{0, upper}(gen);
}


};