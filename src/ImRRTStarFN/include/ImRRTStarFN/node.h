#ifndef RRT_STARFN_GLOBAL_PLANNER_NODE_H_
#define RRT_STARFN_GLOBAL_PLANNER_NODE_H_

#include <cmath>
#include <vector>

// namespace rrt_starFN_Global_planner
namespace rrt_starFN_Global_planner {
    // Calculate euclidean Distance
    inline float euclideanDistance2D(float x1, float y1, float x2, float y2) {
        return std::hypot((x1 - x2), (y1 - y2));
    }
    // RRTStarFN Node
    class Node {
    public:

        float x;
        float y;
        int node_id;
        int parent_id;
        float cost{0.0}; // cost from root
        float h_cost{0.0}; // cost to goal (straight line)
        int leaf{0}; // leaf number
        std::vector<int> child_id; // child_id array


        Node();
        Node(float px, float py, int node_index, int parent_index);

        bool operator==(const Node& node);
        bool operator!=(const Node& node);

    };

};


#endif