#ifndef RRT_STAR_GLOBAL_PLANNER_NODE_H_
#define RRT_STAR_GLOBAL_PLANNER_NODE_H_
#include <vector>
#include <cmath>

// namespace rrt_star_Global_planner
namespace rrt_star_Global_planner {
    // Calculate euclidean Distance
    inline float euclideanDistance2D(float x1, float y1, float x2, float y2) {
        return std::hypot((x1 - x2), (y1 - y2));
    }
    // RRTStar Node
    class Node {
    public:

        float x;
        float y;
        int node_id;
        int parent_id;
        std::vector<int> child_id;
        float cost{0.0}; // cost from root

        Node();
        Node(float px, float py, int node_index, int parent_index);

        bool operator==(const Node& node);
        bool operator!=(const Node& node);

    };

};


#endif