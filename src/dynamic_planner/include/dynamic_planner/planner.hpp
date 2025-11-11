#pragma once
#include <vector>
#include <queue>
#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <unordered_map>
#include <algorithm>
#include "map.hpp"

namespace dynamic_planner {

// structure for map coordinate
struct Point {
    int x, y;
};

class Planner {
public:
    // Our class hasn't any data but only functions so I just create an empty constructor automatically
    Planner() = default;

    // A* function
    std::vector<Point> plan(const Map& map,
                            const Point& start,
                            const Point& goal);

private:
    // Heuristic for A*
    float heuristic(const Point& a, const Point& b, float resolution) const;
    float distanceToCost(float distance, float resolution) const;
};

} // namespace dynamic_planner
