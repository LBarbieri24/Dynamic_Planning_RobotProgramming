#include "dynamic_planner/map.hpp"

namespace dynamic_planner {

//-----------------------------------------------------------
// Constructor
//-----------------------------------------------------------

Map::Map(int width, int height, float resolution)
: width_(width), height_(height), resolution_(resolution)
{
    // total number of cells = width * height
    data_.resize(width_ * height_);

    // initialize with "infinity" distance (meaning no obstacle)
    std::fill(data_.begin(), data_.end(), std::numeric_limits<float>::infinity());
}

//-----------------------------------------------------------
// Clear the map by setting all cells to infinity
//-----------------------------------------------------------

void Map::clear() {
    std::fill(data_.begin(), data_.end(), std::numeric_limits<float>::infinity());
}

//-----------------------------------------------------------
// Safe getters and setters
//-----------------------------------------------------------

// NEW FUNCTION
bool Map::inBounds(int gx, int gy) const {
    return gx >= 0 && gx < width_ && gy >= 0 && gy < height_;
}

float Map::get(int x, int y) const {
    if (!inBounds(x, y)) return 0.0f; // treat out of bounds as obstacle
    return data_[y * width_ + x];
}

void Map::set(int x, int y, float v) {
    data_[y * width_ + x] = v;
}

int Map::worldToGridX(float wx) const {
    return static_cast<int>(std::round(wx / resolution_)) + width_ / 2;
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


bool Map::isOccupied(int gx, int gy) const {
    if (!inBounds(gx, gy)) return true;
    return data_[gy * width_ + gx] == 0.0f;  // ✅ correct
}


//-----------------------------------------------------------
// Convert LaserScan to a local occupancy map
//-----------------------------------------------------------

void Map::updateFromScan(const std::vector<float>& ranges,
                         float angle_min,
                         float angle_increment,
                         float range_max_val)
{
    // The goal of this function is to populate the 'local_map_'
    // with obstacles and free space *relative to the robot's current position*.
    // Therefore, within this local map, the robot is considered to be at
    // the origin (0,0) in world coordinates, which maps to (width_/2, height_/2) in grid coordinates.

    // No need for robot_gx, robot_gy here, as the robot is implicitly at the center of this local map's grid.
    
    for (int i = 0; i < (int)ranges.size(); ++i) {
        float r = ranges[i];
        // The angle is already in the robot's base_link frame (relative to the robot's heading)
        float angle_in_base_link = angle_min + i * angle_increment;

        float effective_range_for_free_space = (std::isfinite(r) && r > 0.0f && r < range_max_val) ? r : range_max_val;
        float current_dist_along_ray = 0.0f;
        float step_size = resolution_ * 0.5; // Use a step size for ray tracing, can be less than resolution for denser tracing

        while (current_dist_along_ray < effective_range_for_free_space - (step_size / 2.0f) )
        {
            // Calculate the point along the ray in the robot's BASE_LINK frame (robot is at 0,0, 0 yaw)
            float px_base_link = current_dist_along_ray * std::cos(angle_in_base_link);
            float py_base_link = current_dist_along_ray * std::sin(angle_in_base_link);

            int gx = worldToGridX(px_base_link);
            int gy = worldToGridY(py_base_link);

            if (inBounds(gx, gy)) {
                set(gx, gy, 1.0f); // Mark as explicitly observed free
            }
            current_dist_along_ray += step_size;
        }

        // --- Mark obstacle point in the LOCAL_MAP ---
        // If the laser beam actually hit an obstacle within its valid range
        if (std::isfinite(r) && r > 0.0f && r < range_max_val) {
            // Laser point (obstacle) is directly in the robot's BASE_LINK frame
            float ox_base_link = r * std::cos(angle_in_base_link);
            float oy_base_link = r * std::sin(angle_in_base_link);

            // Convert this BASE_LINK point to LOCAL_MAP grid coordinates
            int ogx = worldToGridX(ox_base_link);
            int ogy = worldToGridY(oy_base_link);

            if (inBounds(ogx, ogy)) {
                set(ogx, ogy, 0.0f); // Mark as explicitly occupied
            }
        }
    }
    
    // Ensure the robot's own center cell in this LOCAL map is always free
    // The robot's center in its local grid frame is always at (width_/2, height_/2).
    if (inBounds(width_ / 2, height_ / 2)) {
        set(width_ / 2, height_ / 2, 1.0f); 
    }
}


//-----------------------------------------------------------
// Fuse local map into global map using min() operator
//-----------------------------------------------------------

void Map::fuse(const Map& local, int robot_gx, int robot_gy)
{
    int half_w = local.width_ / 2;
    int half_h = local.height_ / 2;

    for (int ly = 0; ly < local.height_; ++ly) {
        for (int lx = 0; lx < local.width_; ++lx) {

            float local_val = local.get(lx, ly);

            int gx = robot_gx + (lx - half_w);
            int gy = robot_gy + (ly - half_h);

            if (!inBounds(gx, gy)) continue; 

            float global_val = get(gx, gy); // Current value in the global map

            // Apply update logic:
            if (local_val == 0.0f) { // Local map explicitly saw an obstacle (highest certainty)
                set(gx, gy, 0.0f); // Global map becomes occupied
            } else if (local_val == 1.0f) { 
                if (global_val == 0.0f) { 
                    set(gx, gy, 1.0f); // Overwrite with free space
                } 
                
                else if (!std::isfinite(global_val)) { 
                    set(gx, gy, 1.0f);
                }

            }
        }
    }
}


//-----------------------------------------------------------
// Distance transform using simple multi-source BFS
//-----------------------------------------------------------

void Map::computeDistanceTransform() {
    std::queue<std::pair<int,int>> q;

    // Initialize
    for (int y = 0; y < height_; ++y) {
        for (int x = 0; x < width_; ++x) {
            if (get(x,y) == 0.0f) {
                q.push({x,y});
            } else {
                set(x,y, std::numeric_limits<float>::infinity());
            }
        }
    }

    // 8-neighborhood
    const int dx[8] = {1, -1, 0, 0, 1, -1, 1, -1};
    const int dy[8] = {0, 0, 1, -1, 1, 1, -1, -1};
    const float distances[8] = {
        resolution_, resolution_, resolution_, resolution_,  // Cardinal
        resolution_ * 1.414f, resolution_ * 1.414f,          // Diagonal
        resolution_ * 1.414f, resolution_ * 1.414f
    };

    while (!q.empty()) {
        auto [x, y] = q.front();
        q.pop();

        float current_dist = get(x,y);

        for (int k = 0; k < 8; ++k) {
            int nx = x + dx[k];
            int ny = y + dy[k];

            if (nx < 0 || nx >= width_ || ny < 0 || ny >= height_) continue;

            float new_dist = current_dist + distances[k];  // Use appropriate distance

            if (new_dist < get(nx, ny)) {
                set(nx, ny, new_dist);
                q.push({nx, ny});
            }
        }
    }
}

} // namespace dynamic_planner
