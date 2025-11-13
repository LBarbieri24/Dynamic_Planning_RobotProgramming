#include "dynamic_planner/planner.hpp"

namespace dynamic_planner {

// Estimate remaining cost from point A to point B using Dijkstra cost on empty grid
float Planner::heuristic(const Point& a, const Point& b, float resolution) const {

    // Distances in cells
    int dx_cells = std::abs(a.x - b.x);
    int dy_cells = std::abs(a.y - b.y);

    // How many diagonal moves you can make
    int diag = std::min(dx_cells, dy_cells);
    // Remaining moves are straight
    int straight = std::max(dx_cells, dy_cells) - diag;

    const float base_cost = 1.0f; 

    // Distances in meters
    const float Distance_straight = resolution;
    const float Distance_diag = resolution * 1.41421356237f; 

    // Total minimal cost on empty map (meters * base_cost)
    float cost = base_cost * (diag * Distance_diag + straight * Distance_straight);
    return cost;
}



// Assign a cost based on how far the cell is from nearest obstacle
float Planner::distanceToCost(float distance) const {
    
    // If at obstacle (distance = 0), cost is infinite (can't go there)
    if (distance <= 0.0f) {
        return std::numeric_limits<float>::infinity();
    }
    
    // COST FUNCTION PARAMETERS
    const float safety_distance = 0.5;   // 50cm safety margin
    const float penalty_factor = 2.0f;   // How much to penalize being close
    
    // Start with base cost of 1.0
    float base_cost = 1.0f;
    
    // Add extra cost if too close to obstacles
    if (distance < safety_distance) {
        base_cost += penalty_factor;
    }
    
    return base_cost;
}


// C++ unordered_map needs a way to convert keys to hash values
struct PointHash {
    std::size_t operator()(const Point& p) const noexcept {
        // Combine x and y into a single hash value
        // Bit shift x left by 32 bits, then XOR with y
        // XOR (^) = bitwise exclusive OR operator
        return ((std::size_t)p.x << 32) ^ (std::size_t)p.y;
    }
};


// EQUALITY OPERATOR: Check if two points are the same
// Needed for unordered_map to compare keys
bool operator==(const Point& a, const Point& b) {
    return a.x == b.x && a.y == b.y;
}


//-----------------------------------------------------------
// A* PLANNING ALGORITHM: Find optimal path from start to goal
//-----------------------------------------------------------
std::vector<Point> Planner::plan(const Map& map,
                                 const Point& start,
                                 const Point& goal)
{
    RCLCPP_INFO(rclcpp::get_logger("Planner"), 
                "PLANNER_DEBUG: Planning from grid=(%d, %d) to goal grid=(%d, %d)",
                start.x, start.y, goal.x, goal.y);

    // VALIDITY CHECKS: Make sure start and goal are not inside obstacles
    if (map.isOccupied(start.x, start.y)) {
        RCLCPP_ERROR(rclcpp::get_logger("Planner"), 
                     "PLANNER_ERROR: Start point (%d, %d) is occupied!", 
                     start.x, start.y);
        return {};  // {} = empty vector
    }

    if (map.isOccupied(goal.x, goal.y)) {
        RCLCPP_ERROR(rclcpp::get_logger("Planner"), 
                     "PLANNER_ERROR: Goal point (%d, %d) is occupied!", 
                     goal.x, goal.y);
        return {};
    }

    
    //-----------------------------------------------------------
    // A* DATA STRUCTURES
    //----------------------------------------------------------- 
    // NODE: Represents a point during search
    struct Node {
        Point p;   // Grid position
        float f;   // f = g + h (total estimated cost)
        float g;   // g = cost from start to this node
    };

    // COMPARATOR: For priority queue (lower f-score = higher priority)
    // Lambda function: [](params) { body }
    auto cmp = [](const Node& a, const Node& b) {
        return a.f > b.f;  // Return true if a has LOWER priority than b
    };

    // PRIORITY QUEUE (Open Set): Nodes to explore, sorted by f-score
    // std::priority_queue: Heap data structure (always gives smallest f first)
    // decltype(cmp): Get the type of the lambda function
    std::priority_queue<Node, std::vector<Node>, decltype(cmp)> open(cmp);

    // HASH MAPS: Store data associated with each Point
    // unordered_map = hash table (O(1) average lookup time)
    std::unordered_map<Point, float, PointHash> g_score;      // Best known cost to reach each point
    std::unordered_map<Point, Point, PointHash> came_from;    // Parent pointer for path reconstruction

    // Initialize start node
    g_score[start] = 0.0f;
    open.push({start, heuristic(start, goal, map.resolution()), 0.0f});

    //-----------------------------------------------------------
    // 8-CONNECTIVITY: Movement directions (including diagonals)
    //-----------------------------------------------------------
    // Order: Right, Left, Down, Up, DownRight, DownLeft, UpRight, UpLeft
    const int dx[8] = {1, -1,  0,  0,  1, -1,  1, -1};
    const int dy[8] = {0,  0,  1, -1,  1,  1, -1, -1};
    
    // COST MULTIPLIERS: Diagonal movement costs √2 times more than cardinal
    const float cost_multiplier[8] = {
        1.0f, 1.0f, 1.0f, 1.0f,           // Cardinal directions (left/right/up/down)
        1.414f, 1.414f, 1.414f, 1.414f    // Diagonal directions (√2 ≈ 1.414)
    };

    std::vector<Point> reconstructed_path;  // Will store final path

    //-----------------------------------------------------------
    // A* MAIN LOOP
    //-----------------------------------------------------------
    while (!open.empty()) {
        // Get node with lowest f-score
        Node current = open.top();
        open.pop();

        // OPTIMIZATION: Skip if we already found a better path to this point
        if (g_score.count(current.p) && current.g > g_score[current.p]) {
            continue;
        }

        // GOAL CHECK: Have we reached the destination?
        if (current.p.x == goal.x && current.p.y == goal.y) {
            // BACKTRACK: Reconstruct path by following parent pointers
            Point p = goal;
            while (!(p.x == start.x && p.y == start.y)) {
                reconstructed_path.push_back(p);
                p = came_from[p];  // Move to parent
            }
            reconstructed_path.push_back(start);
            
            // Reverse path (we built it backwards)
            std::reverse(reconstructed_path.begin(), reconstructed_path.end());
            
            RCLCPP_INFO(rclcpp::get_logger("Planner"), 
                        "PLANNER_DEBUG: Path found with %zu waypoints",
                        reconstructed_path.size());
            
            return reconstructed_path;
        }

        //-----------------------------------------------------------
        // EXPAND NEIGHBORS: Try all 8 adjacent cells
        //-----------------------------------------------------------
        for (int k = 0; k < 8; ++k) {
            // Calculate neighbor position
            Point np{current.p.x + dx[k], current.p.y + dy[k]};

            // Skip if out of bounds
            if (!map.inBounds(np.x, np.y)) continue;

            // Get distance to nearest obstacle at this neighbor
            float distance = map.get(np.x, np.y);
            float move_cost = distanceToCost(distance);
            
            // Skip if infinite cost (in obstacle)
            if (!std::isfinite(move_cost)) continue;

            // SCALE BY DISTANCE: Diagonal moves cost √2 times more
            move_cost *= cost_multiplier[k];

            // CALCULATE TENTATIVE G-SCORE: Cost to reach neighbor via current
            float tentative_g = current.g + move_cost;

            // UPDATE NEIGHBOR if this path is better
            // .count(): Returns 1 if key exists, 0 otherwise
            if (!g_score.count(np) || tentative_g < g_score[np]) {
                g_score[np] = tentative_g;       // Update best cost
                came_from[np] = current.p;        // Set parent pointer
                
                // Calculate f-score and add to queue
                float f = tentative_g + heuristic(np, goal, map.resolution());
                open.push({np, f, tentative_g});
            }
        }
    }

    // No path found (open set empty without reaching goal)
    RCLCPP_WARN(rclcpp::get_logger("Planner"), 
                "PLANNER_WARN: No path found!");
    return {};
}

} // namespace dynamic_planner