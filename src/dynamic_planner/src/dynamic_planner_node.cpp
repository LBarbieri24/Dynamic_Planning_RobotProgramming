#include "dynamic_planner/dynamic_planner_node.hpp"

using namespace std::chrono_literals; 

namespace dynamic_planner {
DynamicPlannerNode::DynamicPlannerNode() 
: Node("dynamic_planner_node"),              // Initialize the ROS2 node with name "dynamic_planner_node"
  global_map_(500, 500, 0.05f),              // Create 500x500 grid with 0.05m resolution (25m x 25m world)
  local_map_(100, 100, 0.05f),               // Create 100x100 grid with 0.05m resolution (5m x 5m world)
  previous_path_()                            // Initialize empty vector for storing the path from last iteration
{
    // TF (Transform) system: Tracks coordinate transformations between robot frames
    // std::make_shared: Creates a shared pointer (smart pointer that auto-deletes when no longer used)
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // SUBSCRIBER: Listens to laser scan data from the robot
    // std::bind: Creates a callback function that will call scanCallback when data arrives
    // std::placeholders::_1: Placeholder for the message argument that ROS will pass
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan",                                                    
        10,                                                         
        std::bind(&DynamicPlannerNode::scanCallback, this, std::placeholders::_1) //run the callback function when data arrives
    ); 

    // PUBLISHERS: Send data to other ROS nodes
    path_pub_ = create_publisher<nav_msgs::msg::Path>("/planner/path", 10);
    map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/planner/distance_map", 10);

    // TIMER: Triggers planning at regular intervals (every 1500 milliseconds = 1.5 seconds)
    // Slower rate to have a smoother movement for the robot
    planning_timer_ = create_wall_timer(
        1500ms,
        std::bind(&DynamicPlannerNode::planningTimerCallback, this)
    );
}

// FUNCTION: Publishes the distance map as a ROS OccupancyGrid message for visualization
void DynamicPlannerNode::publishDistanceMap()
{   
    // Create the message
    nav_msgs::msg::OccupancyGrid grid;
    grid.header.stamp = this->now();          // Timestamp
    grid.header.frame_id = "map";             // Coordinate frame
    
    // Set map metadata
    grid.info.resolution = global_map_.resolution();
    grid.info.width = global_map_.width();
    grid.info.height = global_map_.height();
    
    // Set the origin (where grid [0,0] is in world coordinates)
    grid.info.origin.position.x = global_map_.gridToWorldX(0);
    grid.info.origin.position.y = global_map_.gridToWorldY(0);
    grid.info.origin.position.z = 0.0;
    
    // Quaternion represents rotation (w=1, x=y=z=0 means no rotation (our robot moves without rotating))
    grid.info.origin.orientation.w = 1.0;
    grid.info.origin.orientation.x = 0.0;
    grid.info.origin.orientation.y = 0.0;
    grid.info.origin.orientation.z = 0.0;

    // Allocate memory for the grid data
    grid.data.resize(grid.info.width * grid.info.height);

    // Convert distance values to occupancy values (0-100 scale)
    for (int y = 0; y < global_map_.height(); ++y) {
        for (int x = 0; x < global_map_.width(); ++x) {
            float distance = global_map_.get(x, y);
            int occupancy_value;  

            // Convert distance to occupancy:
            if (distance <= 0.0f) {
                occupancy_value = 100;      // 100 = Fully occupied (obstacle)
            } else if (!std::isfinite(distance)) {
                occupancy_value = -1;       // -1 = Unknown (never observed)
            } else { 
                occupancy_value = 0;        // 0 = Free space
            }
            // Row-major order: index = y * width + x
            grid.data[y * grid.info.width + x] = occupancy_value; 
        }
    }
    
    // Send the message
    map_pub_->publish(grid);
    RCLCPP_INFO(this->get_logger(), "Map published.");
}


// CALLBACK: Called by the timer every 1.5 seconds to plan a new path
void DynamicPlannerNode::planningTimerCallback()
{
    // ----------------------------------------------------
    // STEP 1: Get robot's current position and orientation
    // ----------------------------------------------------
    geometry_msgs::msg::TransformStamped tf;
    try {
        // Look up the transform from "map" frame to "base_link" (robot) frame
        // tf2::TimePointZero = get the latest available transform
        tf = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN(this->get_logger(), "TF lookup failed in planningTimerCallback: %s", ex.what());
        return;
    }

    // Extract position and orientation from the transform
    float rx  = tf.transform.translation.x;   // Robot X position in meters
    float ry  = tf.transform.translation.y;   // Robot Y position in meters
    float yaw = tf2::getYaw(tf.transform.rotation);  // Robot heading angle in radians

    // ----------------------------------------------------
    // STEP 2: Convert robot position from world to grid coordinates
    // ----------------------------------------------------
    Point robot_pos;
    robot_pos.x = global_map_.worldToGridX(rx);
    robot_pos.y = global_map_.worldToGridY(ry);

    // ----------------------------------------------------
    // STEP 3: Define the goal position (hardcoded for now)
    // ----------------------------------------------------
    float goal_wx = 4.8;  // Goal X in world coordinates (meters)
    float goal_wy = 0.0;  // Goal Y in world coordinates (meters)

    Point goal;
    goal.x = global_map_.worldToGridX(goal_wx);
    goal.y = global_map_.worldToGridY(goal_wy);

    // ----------------------------------------------------
    // STEP 4: PATH REUSE - Find closest waypoint on previous path
    // ----------------------------------------------------
    // Goal: Reuse parts of the old path to avoid replanning from scratch

    // Calculate distance from robot to goal (in meters)
    float dx_to_goal = goal_wx - rx;
    float dy_to_goal = goal_wy - ry;
    float dist_to_goal = std::sqrt(dx_to_goal * dx_to_goal + dy_to_goal * dy_to_goal);

    int KEEP_STEPS;
    if (dist_to_goal > 2.0) {
        KEEP_STEPS = 10;  // Far from goal: keep 10 waypoints
    } else if (dist_to_goal > 1.0) {
        KEEP_STEPS = 5;   // Medium distance: keep 5 waypoints
    }
    else {
        KEEP_STEPS = 0;   // Close to goal: dont keep any waypoints
    }
    
    std::vector<Point> future_path;  // Will store waypoints that are still ahead of robot
    int closest_idx = -1;             // Index of the closest waypoint
    float min_dist = std::numeric_limits<float>::infinity();  // Infinite distance initially
    
    // Search through previous path to find closest waypoint that's ahead of robot
    if (!previous_path_.empty()) {
        int closest_idx_any = -1;      // Closest waypoint (regardless of direction)
        float min_dist_any = std::numeric_limits<float>::infinity();

        int closest_idx_ahead = -1;    // Closest waypoint that's ahead of robot
        float min_dist_ahead = std::numeric_limits<float>::infinity();

        // Loop through all waypoints in previous path
        for (size_t i = 0; i < previous_path_.size(); ++i) {
            // Convert waypoint from grid to world coordinates
            float wx = global_map_.gridToWorldX(previous_path_[i].x);
            float wy = global_map_.gridToWorldY(previous_path_[i].y);
            
            // Calculate distance from robot to waypoint
            float dx = wx - rx;
            float dy = wy - ry;
            float dist = std::sqrt(dx * dx + dy * dy);

            // Track the absolute closest waypoint
            if (dist < min_dist_any) {
                min_dist_any = dist;
                closest_idx_any = i;
            }

            // DOT PRODUCT: Check if waypoint is ahead based on robot's heading
            // If dot product > 0, waypoint is in front of robot
            // cos(yaw) and sin(yaw) give the robot's forward direction vector
            float forward_dot = std::cos(yaw) * dx + std::sin(yaw) * dy;
            if (forward_dot > 0.0f) {
                if (dist < min_dist_ahead) {
                    min_dist_ahead = dist;
                    closest_idx_ahead = i;
                }
            }
        }

        // Prefer waypoint ahead if found, otherwise use closest waypoint
        if (closest_idx_ahead >= 0) {
            closest_idx = closest_idx_ahead;
            min_dist = min_dist_ahead;
            RCLCPP_INFO(this->get_logger(), 
                       "Found closest AHEAD waypoint at index %d (dist=%.3fm), keeping %zu waypoints ahead", 
                       closest_idx, min_dist, previous_path_.size() - closest_idx);
        } else if (closest_idx_any >= 0) {
            closest_idx = closest_idx_any;
            min_dist = min_dist_any;
            RCLCPP_WARN(this->get_logger(), 
                       "No waypoint clearly ahead; using closest at index %d (dist=%.3fm)", 
                       closest_idx, min_dist);
        }

        // Keep all waypoints from closest index onwards
        // C++ vector slicing: assign from iterator range
        if (closest_idx >= 0) {
            future_path.assign(previous_path_.begin() + closest_idx, previous_path_.end());
        }
    }

    // ----------------------------------------------------
    // STEP 5: Determine where to start planning from
    // ----------------------------------------------------
    Point planning_start = robot_pos;  // Default: start from robot position
    std::vector<Point> kept_path;      // Waypoints we're keeping from old path
    
    if (future_path.size() > KEEP_STEPS) {
        // Case 1: Many waypoints ahead - keep first KEEP_STEPS, replan from waypoint #10
        kept_path.assign(future_path.begin(), future_path.begin() + KEEP_STEPS);
        planning_start = future_path[KEEP_STEPS];
        
        RCLCPP_INFO(this->get_logger(), 
                    "Keeping %d waypoints, replanning from grid=(%d,%d) world=(%.2f,%.2f)", 
                    KEEP_STEPS, planning_start.x, planning_start.y,
                    global_map_.gridToWorldX(planning_start.x),
                    global_map_.gridToWorldY(planning_start.y));
    } else if (!future_path.empty()) {
        // Case 2: Few waypoints ahead - keep all, replan from last one
        kept_path = future_path;
        planning_start = future_path.back();  // .back() returns last element
        
        RCLCPP_INFO(this->get_logger(), 
                    "Keeping all %zu future waypoints, replanning from last", kept_path.size());
    } else {
        // Case 3: No valid previous path - plan from robot position
        RCLCPP_INFO(this->get_logger(), 
                    "No previous path. Planning from robot position grid=(%d,%d)", 
                    robot_pos.x, robot_pos.y);
    }

    // ----------------------------------------------------
    // STEP 6: A* PLANNING - Find path from planning_start to goal
    // ----------------------------------------------------
    auto new_path_segment = planner_.plan(global_map_, planning_start, goal);

    // Combine kept waypoints + newly planned path
    std::vector<Point> full_path = kept_path;
    // .insert(): Insert all elements from new_path_segment at the end
    full_path.insert(full_path.end(), new_path_segment.begin(), new_path_segment.end());

    // Store for next iteration
    previous_path_ = full_path;

    // ----------------------------------------------------
    // STEP 7: Publish the path as a ROS message
    // ----------------------------------------------------
    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp = this->now();
    path_msg.header.frame_id = "map";

    // Convert each grid point to a world coordinate pose
    for (auto &pt : full_path) {  
        geometry_msgs::msg::PoseStamped pose;
        pose.header = path_msg.header;

        // Convert grid to world coordinates
        pose.pose.position.x = global_map_.gridToWorldX(pt.x);
        pose.pose.position.y = global_map_.gridToWorldY(pt.y);
        pose.pose.position.z = 0.0;

        path_msg.poses.push_back(pose);  // Add to list
    }

    path_pub_->publish(path_msg);
    
    RCLCPP_INFO(this->get_logger(), 
                "Published: %zu kept + %zu new = %zu total waypoints", 
                kept_path.size(), new_path_segment.size(), full_path.size());
}


// CALLBACK: Called whenever a new laser scan arrives
void DynamicPlannerNode::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    // ----------------------------------------------------
    // STEP 1: Get robot's current position via TF
    // ----------------------------------------------------
    geometry_msgs::msg::TransformStamped tf;
    try {
        tf = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN(this->get_logger(), "TF lookup failed in scanCallback: %s", ex.what());
        return;
    }

    float rx  = tf.transform.translation.x;
    float ry  = tf.transform.translation.y;

    // ----------------------------------------------------
    // STEP 2: Update the local map with new scan data
    // ----------------------------------------------------
    local_map_.clear();  // Reset local map
    
    // Process the laser scan into the local map
    local_map_.updateFromScan(
        msg->ranges,           // Array of distance measurements
        msg->angle_min,        // Starting angle of scan (radians)
        msg->angle_increment,  // Angle between rays (radians)
        msg->range_max         // Maximum sensing range (meters)
    );

    // Convert robot position to grid coordinates
    int robot_gx = global_map_.worldToGridX(rx);
    int robot_gy = global_map_.worldToGridY(ry);

    // ----------------------------------------------------
    // STEP 3: Fuse local map into global map
    // ----------------------------------------------------
    // Combines new observations with existing global map knowledge
    global_map_.fuse(local_map_, robot_gx, robot_gy);
    
    // Recompute distance transform (distance from each cell to nearest obstacle)
    global_map_.computeDistanceTransform();

    // Publish the distance map for visualization
    publishDistanceMap(); 
}

} // namespace dynamic_planner


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);  // Initialize ROS2
    
    rclcpp::spin(std::make_shared<dynamic_planner::DynamicPlannerNode>()); // Keep the node alive to process callbacks
    
    rclcpp::shutdown(); 
    return 0;
}