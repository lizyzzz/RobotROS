#ifndef RRT_STAR_GLOBAL_PLANNER_RANDOM_DOUBLE_GENERATOR_H_
#define RRT_STAR_GLOBAL_PLANNER_RANDOM_DOUBLE_GENERATOR_H_


#include <random>
#include <cfloat>

namespace rrt_star_Global_planner {

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
    // XorY : 0 for X, others for Y
    double generate(int XorY);

};


};


#endif