#include "dynamic_planner/planner.hpp"

namespace dynamic_planner {

//-----------------------------------------------------------
// Heuristic: Euclidean distance (on empty map)
//-----------------------------------------------------------

float Planner::heuristic(const Point& a, const Point& b, float resolution) const {
    float dx = (a.x - b.x) * resolution;
    float dy = (a.y - b.y) * resolution;
    return std::sqrt(dx * dx + dy * dy);
}

//-----------------------------------------------------------
// Convert distance to traversal cost
//-----------------------------------------------------------

float Planner::distanceToCost(float distance, float /*resolution*/) const {
    // If distance is 0 (obstacle), return infinite cost
    if (distance <= 0.0f) {
        return std::numeric_limits<float>::infinity();
    }
    
    // Parameters for cost function
    const float safety_distance = 0.5; // 40cm safety margin
    const float penalty_factor = 2.0f;   // how much to penalize proximity
    
    // Base movement cost
    float base_cost = 1.0f;
    
    // Add exponential penalty for being close to obstacles
    if (distance < safety_distance) {
        float ratio = distance / safety_distance;
        base_cost += penalty_factor * (1.0f - ratio);
    }
    
    return base_cost;
}

//-----------------------------------------------------------
// Hash function for Point (needed for unordered_map)
//-----------------------------------------------------------

struct PointHash {
    std::size_t operator()(const Point& p) const noexcept {
        return ((std::size_t)p.x << 32) ^ (std::size_t)p.y;
    }
};

bool operator==(const Point& a, const Point& b) {
    return a.x == b.x && a.y == b.y;
}


//-----------------------------------------------------------
// A* Planning with 8-connectivity (diagonal neighbors)
//-----------------------------------------------------------

std::vector<Point> Planner::plan(const Map& map,
                                 const Point& start,
                                 const Point& goal)
{
    RCLCPP_INFO(rclcpp::get_logger("Planner"), 
                "PLANNER_DEBUG: Planning from grid=(%d, %d) to goal grid=(%d, %d)",
                start.x, start.y, goal.x, goal.y);

    if (map.isOccupied(start.x, start.y)) {
        RCLCPP_ERROR(rclcpp::get_logger("Planner"), 
                     "PLANNER_ERROR: Start point (%d, %d) is occupied!", 
                     start.x, start.y);
        return {};
    }

    if (map.isOccupied(goal.x, goal.y)) {
        RCLCPP_ERROR(rclcpp::get_logger("Planner"), 
                     "PLANNER_ERROR: Goal point (%d, %d) is occupied!", 
                     goal.x, goal.y);
        return {};
    }

    // Calculate the general direction from start to goal
    float goal_direction_x = goal.x - start.x;
    float goal_direction_y = goal.y - start.y;
    float goal_dist = std::sqrt(goal_direction_x * goal_direction_x + 
                                goal_direction_y * goal_direction_y);
    
    if (goal_dist > 0) {
        goal_direction_x /= goal_dist;  // Normalize
        goal_direction_y /= goal_dist;
    }

    // Priority queue for A*
    struct Node {
        Point p;
        float f;  // f = g + h
        float g;
    };

    auto cmp = [](const Node& a, const Node& b) {
        return a.f > b.f;
    };

    std::priority_queue<Node, std::vector<Node>, decltype(cmp)> open(cmp);

    std::unordered_map<Point, float, PointHash> g_score;
    std::unordered_map<Point, Point, PointHash> came_from;

    g_score[start] = 0.0f;
    open.push({start, heuristic(start, goal, map.resolution()), 0.0f});

    // ✅ 8-CONNECTIVITY: Include diagonal neighbors
    // Order: Right, Left, Down, Up, DownRight, DownLeft, UpRight, UpLeft
    const int dx[8] = {1, -1,  0,  0,  1, -1,  1, -1};
    const int dy[8] = {0,  0,  1, -1,  1,  1, -1, -1};
    
    // Cost multipliers: 1.0 for cardinal, sqrt(2) for diagonal
    const float cost_multiplier[8] = {
        1.0f, 1.0f, 1.0f, 1.0f,           // Cardinal directions
        1.414f, 1.414f, 1.414f, 1.414f    // Diagonal directions (√2)
    };

    std::vector<Point> reconstructed_path;

    while (!open.empty()) {
        Node current = open.top();
        open.pop();

        // Skip if we found a better path already
        if (g_score.count(current.p) && current.g > g_score[current.p]) {
            continue;
        }

        // Goal reached!
        if (current.p.x == goal.x && current.p.y == goal.y) {
            Point p = goal;
            while (!(p.x == start.x && p.y == start.y)) {
                reconstructed_path.push_back(p);
                p = came_from[p];
            }
            reconstructed_path.push_back(start);
            std::reverse(reconstructed_path.begin(), reconstructed_path.end());
            
            RCLCPP_INFO(rclcpp::get_logger("Planner"), 
                        "PLANNER_DEBUG: Path found with %zu waypoints (8-connectivity)",
                        reconstructed_path.size());
            
            return reconstructed_path;  // Without smoothing
        }

        // Explore all 8 neighbors
        for (int k = 0; k < 8; ++k) {
            Point np{current.p.x + dx[k], current.p.y + dy[k]};

            if (!map.inBounds(np.x, np.y)) continue;

            float distance = map.get(np.x, np.y);
            float move_cost = distanceToCost(distance, map.resolution());
            
            if (!std::isfinite(move_cost)) continue;

            // ✅ Scale movement cost by distance (diagonal = √2 × base cost)
            move_cost *= cost_multiplier[k];

            // Direction alignment penalty (optional - reduces zigzagging)
            float move_direction_x = dx[k];
            float move_direction_y = dy[k];
            float move_length = std::sqrt(move_direction_x * move_direction_x + 
                                         move_direction_y * move_direction_y);
            
            if (move_length > 0) {
                move_direction_x /= move_length;
                move_direction_y /= move_length;
            }
            
            float direction_alignment = (move_direction_x * goal_direction_x + 
                                        move_direction_y * goal_direction_y);
            
            float backward_penalty = 0.0f;
            if (direction_alignment < 0) {
                backward_penalty = 1.0f * std::abs(direction_alignment);
            }

            float tentative_g = current.g + move_cost + backward_penalty;

            if (!g_score.count(np) || tentative_g < g_score[np]) {
                g_score[np] = tentative_g;
                came_from[np] = current.p;

                float f = tentative_g + heuristic(np, goal, map.resolution());
                open.push({np, f, tentative_g});
            }
        }
    }

    RCLCPP_WARN(rclcpp::get_logger("Planner"), 
                "PLANNER_WARN: No path found!");
    return {};
}

} // namespace dynamic_planner