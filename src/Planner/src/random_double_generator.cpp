#include "Planner/random_double_generator.h"
#include <iostream>

namespace rrt_star_Global_planner {

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
// XorY : 0 for X, other for Y
double RandomDoubleGenerator::generate(int XorY){
    std::mt19937 gen(rd_()); // use rd_ as seed
    
    // uniform_real_distribution does [start, stop)
    // So use nextafter get the next of max_value
    if(XorY == 0) {
        return std::uniform_real_distribution<double>{min_valueX, std::nextafter(max_valueX, DBL_MAX)}(gen);
    }
    else {
        return std::uniform_real_distribution<double>{min_valueY, std::nextafter(max_valueY, DBL_MAX)}(gen);
    }
}


};