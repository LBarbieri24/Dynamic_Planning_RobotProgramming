#pragma once
#include <vector>
#include <cmath>
#include <algorithm>   // for std::fill, std::min
#include <limits>      // for infinity
#include <queue>       // for BFS
#include <rclcpp/rclcpp.hpp>

namespace dynamic_planner {

class Map {
public:
    Map(int width, int height, float resolution);

    // Resets map values to infinity (free space)
    void clear();

    // Converts a LaserScan to a local DISTANCE map
    // Each cell contains the distance to the nearest obstacle
    void updateFromScan(const std::vector<float>& ranges,
                        float angle_min,
                        float angle_increment,
                        float range_max_val);

    // Merges the local map into the global map using min()
    void fuse(const Map& local, int robot_gx, int robot_gy);

    // Computes distance transform from occupied cells
    void computeDistanceTransform();

    // Get/set distance values
    float get(int x, int y) const;
    void set(int x, int y, float v);

    // Map properties
    int width() const { return width_; }
    int height() const { return height_; }
    float resolution() const { return resolution_; }

    // Coordinate transformations
    int worldToGridX(float wx) const;
    int worldToGridY(float wy) const;
    float gridToWorldX(int gx) const;
    float gridToWorldY(int gy) const;

    // Check if cell is occupied (distance == 0)
    bool isOccupied(int gx, int gy) const;

    // Check if cell is within bounds
    bool inBounds(int gx, int gy) const;

private:
    int width_, height_;
    float resolution_;
    
    // Distance map: each cell stores distance to nearest obstacle
    // 0.0f = obstacle, positive = free space, infinity = unknown/unobserved
    std::vector<float> data_;
};

} // namespace dynamic_planner