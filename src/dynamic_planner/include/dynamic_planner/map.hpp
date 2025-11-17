#pragma once
#include <vector>
#include <cmath>
#include <algorithm>   
#include <limits>      
#include <queue>       
#include <rclcpp/rclcpp.hpp>

namespace dynamic_planner {

class Map {
public:
    Map(int width, int height, float resolution);

    // Resets map values to infinity (free space)
    void clear();

    // Converts a LaserScan to a local DISTANCE map
    void updateFromScan(const std::vector<float>& ranges,
                        float angle_min,
                        float angle_increment,
                        float range_max_val);

    // Merges the local map into the global map 
    void fuse(const Map& local, int robot_gx, int robot_gy);

    // Compute distance from each free cell to nearest obstacle
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