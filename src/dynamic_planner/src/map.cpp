#include "dynamic_planner/map.hpp"

namespace dynamic_planner {
//Initializes the map with given dimensions-
Map::Map(int width, int height, float resolution)
: width_(width),           // Number of cells horizontally
  height_(height),         // Number of cells vertically
  resolution_(resolution)  // Size of each cell in meters 
{
    // Calculate total number of cells = width × height
    data_.resize(width_ * height_);

    // Initialize all cells to infinity (meaning no obstacle detected yet)
    std::fill(data_.begin(), data_.end(), std::numeric_limits<float>::infinity());
}

//Reset all cells to unknown state (infinity)
void Map::clear() {
    std::fill(data_.begin(), data_.end(), std::numeric_limits<float>::infinity());
}


// Returns true if (gx, gy) is inside the map
bool Map::inBounds(int gx, int gy) const {
    return gx >= 0 && gx < width_ && gy >= 0 && gy < height_;
}


// Returns: Distance to nearest obstacle, or 0.0 if out of bounds (treat as obstacle)
float Map::get(int x, int y) const {
    if (!inBounds(x, y)) return 0.0f;  // Out of bounds = obstacle

    // Convert 2D coordinate to 1D array index
    return data_[y * width_ + x];
}

// set a value at grid position (x, y)
void Map::set(int x, int y, float v) {
    data_[y * width_ + x] = v;
}

//-----------------------------------------------------------
// COORDINATE CONVERSIONS: World ↔ Grid
//-----------------------------------------------------------
// World coordinates: Real-world meters, can be negative
// Grid coordinates: Integer indices starting from 0


// Add width_/2 to center the map at world origin (0,0)
int Map::worldToGridX(float wx) const {
    return static_cast<int>(std::round(wx / resolution_)) + width_ / 2;
    // static_cast<int>: C++ explicit type conversion from float to int
}

int Map::worldToGridY(float wy) const {
    return static_cast<int>(wy / resolution_) + height_ / 2;
}

float Map::gridToWorldX(int gx) const {
    return (gx - width_ / 2) * resolution_;
}

float Map::gridToWorldY(int gy) const {
    return (gy - height_ / 2) * resolution_;
}



// IS OCCUPIED: Check if a cell contains an obstacle
bool Map::isOccupied(int gx, int gy) const {
    if (!inBounds(gx, gy)) return true;  // Out of bounds = treat as occupied
    
    // Value of 0.0 means obstacle (distance to obstacle = 0)
    return data_[gy * width_ + gx] == 0.0f;
}



//Convert raw laser measurements into a local grid map showing obstacles and free space
void Map::updateFromScan(const std::vector<float>& ranges,    // Distance measurements
                         float angle_min,                     // Starting angle
                         float angle_increment,               // Angle between rays
                         float range_max_val)                 // Max sensor range
{
    // The robot is at the center of this map   
    // Process each laser ray
    for (int i = 0; i < (int)ranges.size(); ++i) {
        float r = ranges[i];  // Distance measurement for this ray
        
        // Angle of this ray in robot's frame (base_link)
        float angle_in_base_link = angle_min + i * angle_increment;

        // Determine how far to mark free space along this ray
        float effective_range_for_free_space;
        if (std::isfinite(r) && r > 0.0f && r < range_max_val) {
            effective_range_for_free_space = r;
        } else {
            effective_range_for_free_space = range_max_val;
        }
        
        // RAY TRACING: Walk along the ray marking cells as free
        float current_dist_along_ray = 0.0f;
        float step_size = resolution_ * 0.5;  // Step size = half a cell (for finer resolution)

        // Mark free space along the ray (stop before hitting obstacle)
        while (current_dist_along_ray < effective_range_for_free_space - (step_size / 2.0f))
        {
            // Calculate point along ray in robot's frame
            // Polar to Cartesian: x = r*cos(θ), y = r*sin(θ)
            float px_base_link = current_dist_along_ray * std::cos(angle_in_base_link);
            float py_base_link = current_dist_along_ray * std::sin(angle_in_base_link);

            // Convert to grid coordinates (robot is at center of local map)
            int gx = worldToGridX(px_base_link);
            int gy = worldToGridY(py_base_link);

            if (inBounds(gx, gy)) {
                set(gx, gy, 1.0f);  // Explicitly observed free space
            }
            
            current_dist_along_ray += step_size;  // Move along ray
        }

        // MARK OBSTACLE: If the laser actually hit something
        if (std::isfinite(r) && r > 0.0f && r < range_max_val) {
            // Calculate obstacle position in robot's frame
            float ox_base_link = r * std::cos(angle_in_base_link);
            float oy_base_link = r * std::sin(angle_in_base_link);

            // Convert to grid coordinates
            int ogx = worldToGridX(ox_base_link);
            int ogy = worldToGridY(oy_base_link);

            if (inBounds(ogx, ogy)) {
                set(ogx, ogy, 0.0f);  
            }
        }
    }
    
    // Ensure robot's position is always marked as free
    if (inBounds(width_ / 2, height_ / 2)) {
        set(width_ / 2, height_ / 2, 1.0f);
    }
}



// Combine new observations (local map) with existing knowledge (global map)
// Parameters:
//   - local: The robot-centered observation map
//   - robot_gx, robot_gy: Robot's position in global map coordinates
void Map::fuse(const Map& local, int robot_gx, int robot_gy)
{
    // Calculate local map center offset
    int half_w = local.width_ / 2;
    int half_h = local.height_ / 2;

    // Iterate through all cells in local map
    for (int ly = 0; ly < local.height_; ++ly) {
        for (int lx = 0; lx < local.width_; ++lx) {

            float local_val = local.get(lx, ly);  // Value from local map

            // Calculate corresponding position in global map
            // Offset local coordinates by robot's position
            int gx = robot_gx + (lx - half_w);
            int gy = robot_gy + (ly - half_h);

            if (!inBounds(gx, gy)) continue;  // Skip if outside global map

            float global_val = get(gx, gy);  // Current value in global map
            
            if (local_val == 0.0f) {
                // Local map saw an obstacle → Always update global
                set(gx, gy, 0.0f);
                
            } else if (local_val == 1.0f) {
                // Local map saw free space
                if (global_val == 0.0f) {
                    // If global thinks it's occupied, new free space overrides
                    set(gx, gy, 1.0f);
                } 
            }
        }
    }
}


// Compute distance from each free cell to nearest obstacle
void Map::computeDistanceTransform() {
    // Find all obstacle positions
    std::vector<std::pair<int,int>> obstacles;
    
    for (int y = 0; y < height_; ++y) {
        for (int x = 0; x < width_; ++x) {
            if (get(x, y) == 0.0f) {  
                obstacles.push_back({x, y});
            }
        }
    }
    
    // For each free cell, find distance to nearest obstacle
    for (int y = 0; y < height_; ++y) {
        for (int x = 0; x < width_; ++x) {
            if (get(x, y) == 0.0f) {
                // This is an obstacle, distance stays 0
                continue;
            }
            
            // Initialize minimum distance to infinity
            float min_distance = std::numeric_limits<float>::infinity();
            
            for (auto& obs : obstacles) {
                // Calculate Euclidean distance wrt every obstacle
                int dx = x - obs.first;
                int dy = y - obs.second;
                float distance = std::sqrt(dx * dx + dy * dy) * resolution_;
                
                if (distance < min_distance) { 
                    min_distance = distance; 
                }
            }
            
            set(x, y, min_distance);
        }
    }
}

} // namespace dynamic_planner
