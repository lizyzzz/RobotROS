#ifndef RRT_STARFN_GLOBAL_PLANNER_RANDOM_DOUBLE_GENERATOR_H_
#define RRT_STARFN_GLOBAL_PLANNER_RANDOM_DOUBLE_GENERATOR_H_


#include <random>
#include <cfloat>
#include <cmath>
#include <utility>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/StdVector>

namespace rrt_starFN02_Global_planner {

class RandomDoubleGenerator {
private:
    std::random_device rd_; // use as seed
    double min_valueX{-1.0};
    double max_valueX{1.0};
    double min_valueY{-1.0};
    double max_valueY{1.0};

public:
    RandomDoubleGenerator() = default;
    // XorY : 0 for X, others for Y
    void setRange(double min, double max, int XorY);
    // generate a point in all map
    std::pair<double, double> generate_all();
    // generate a point in ball
    std::pair<double, double> generate_ball(std::pair<double, double>& start, std::pair<double, double>& goal, double c_max);
    // generate a rand int number
    int generate_int(int upper);
};


};


#endif