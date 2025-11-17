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
    Planner() = default;

    // A* function
    std::vector<Point> plan(const Map& map,
                            const Point& start,
                            const Point& goal);

private:
    // Heuristic for A*
    float heuristic(const Point& a, const Point& b, float resolution) const;
    float distanceToCost(float distance) const;
};

} // namespace dynamic_planner
