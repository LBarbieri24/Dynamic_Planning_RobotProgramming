#include "dynamic_planner/dynamic_planner_node.hpp"

namespace dynamic_planner {

using namespace std::chrono_literals; 

DynamicPlannerNode::DynamicPlannerNode() 
: Node("dynamic_planner_node"),
  global_map_(500, 500, 0.05f),
  local_map_(200, 200, 0.05f),
  previous_path_()  // Initialize empty previous path
{
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10,
        std::bind(&DynamicPlannerNode::scanCallback, this, std::placeholders::_1)
    );

    path_pub_ = create_publisher<nav_msgs::msg::Path>("/planner/path", 10);

    map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/planner/distance_map", 10);

    planning_timer_ = create_wall_timer(
        1500ms, // Planning frequency: 1500ms (adjust as needed)
        std::bind(&DynamicPlannerNode::planningTimerCallback, this)
    );
}

// -------------------------------------------------------------------
// Helper: grid → world
// -------------------------------------------------------------------
inline float gridToWorldX(int gx, const Map& map)
{
    return (gx * map.resolution());
}

inline float gridToWorldY(int gy, const Map& map)
{
    return (gy * map.resolution());
}

void DynamicPlannerNode::publishDistanceMap()
{
    RCLCPP_INFO(this->get_logger(), "!!! DEBUG: Entered publishDistanceMap() !!!"); 
    nav_msgs::msg::OccupancyGrid grid;
    grid.header.stamp = this->now();
    grid.header.frame_id = "map";
    
    grid.info.resolution = global_map_.resolution();
    grid.info.width = global_map_.width();
    grid.info.height = global_map_.height();
    
    grid.info.origin.position.x = global_map_.gridToWorldX(0);
    grid.info.origin.position.y = global_map_.gridToWorldY(0);
    grid.info.origin.position.z = 0.0;
    grid.info.origin.orientation.w = 1.0;
    grid.info.origin.orientation.x = 0.0;
    grid.info.origin.orientation.y = 0.0;
    grid.info.origin.orientation.z = 0.0;

    grid.data.resize(grid.info.width * grid.info.height);

    for (int y = 0; y < global_map_.height(); ++y) {
        for (int x = 0; x < global_map_.width(); ++x) {
            float distance = global_map_.get(x, y);
            int8_t occupancy_value;

            if (distance <= 0.0f) {
                occupancy_value = 100; // Fully occupied
            } else if (!std::isfinite(distance)) {
                occupancy_value = -1; // Unknown
            } else { 
                occupancy_value = 0; // Free space
            }
            grid.data[y * grid.info.width + x] = occupancy_value;
        }
    }
    map_pub_->publish(grid);
    RCLCPP_INFO(this->get_logger(), "!!! DEBUG: Exited publishDistanceMap() - Message Published !!!");
}


void DynamicPlannerNode::planningTimerCallback()
{
    // Get the latest robot pose
    geometry_msgs::msg::TransformStamped tf;
    try {
        tf = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN(this->get_logger(), "TF lookup failed in planningTimerCallback: %s", ex.what());
        return;
    }

    float rx  = tf.transform.translation.x;
    float ry  = tf.transform.translation.y;
    float yaw = tf2::getYaw(tf.transform.rotation);

    // ----------------------------------------------------
    // Convert robot world → grid
    // ----------------------------------------------------
    Point robot_pos;
    robot_pos.x = global_map_.worldToGridX(rx);
    robot_pos.y = global_map_.worldToGridY(ry);

    // ----------------------------------------------------
    // Goal (IN WORLD COORDS)
    // ----------------------------------------------------
    float goal_wx = 4.8;
    float goal_wy = 0.0;

    Point goal;
    goal.x = global_map_.worldToGridX(goal_wx);
    goal.y = global_map_.worldToGridY(goal_wy);

    // ----------------------------------------------------
    // Find the closest waypoint ahead of the robot on the previous path
    // ----------------------------------------------------
    const int KEEP_STEPS = 10;  // Number of waypoints to keep ahead of robot
    
    std::vector<Point> future_path;  // Waypoints ahead of robot
    int closest_idx = -1;
    float min_dist = std::numeric_limits<float>::infinity();
    
    // First, find the closest waypoint to the robot. Prefer waypoints that are "ahead"
    // of the robot based on the robot's yaw (dot product > 0). If none are ahead,
    // fall back to the absolute closest.
    if (!previous_path_.empty()) {
        int closest_idx_any = -1;
        float min_dist_any = std::numeric_limits<float>::infinity();

        int closest_idx_ahead = -1;
        float min_dist_ahead = std::numeric_limits<float>::infinity();

        for (size_t i = 0; i < previous_path_.size(); ++i) {
            float wx = global_map_.gridToWorldX(previous_path_[i].x);
            float wy = global_map_.gridToWorldY(previous_path_[i].y);
            float dx = wx - rx;
            float dy = wy - ry;
            float dist = std::sqrt(dx * dx + dy * dy);

            // Track absolute closest
            if (dist < min_dist_any) {
                min_dist_any = dist;
                closest_idx_any = i;
            }

            // Check if waypoint is ahead based on robot yaw
            float forward_dot = std::cos(yaw) * dx + std::sin(yaw) * dy;
            if (forward_dot > 0.0f) {
                if (dist < min_dist_ahead) {
                    min_dist_ahead = dist;
                    closest_idx_ahead = i;
                }
            }
        }

        // Prefer an ahead waypoint if available
        if (closest_idx_ahead >= 0) {
            closest_idx = closest_idx_ahead;
            min_dist = min_dist_ahead;
            RCLCPP_INFO(this->get_logger(), "Found closest AHEAD waypoint at index %d (dist=%.3fm), keeping %zu waypoints ahead", closest_idx, min_dist, previous_path_.size() - closest_idx);
        } else if (closest_idx_any >= 0) {
            closest_idx = closest_idx_any;
            min_dist = min_dist_any;
            RCLCPP_WARN(this->get_logger(), "No waypoint clearly ahead; using closest at index %d (dist=%.3fm)", closest_idx, min_dist);
        }

        // Keep all waypoints from the chosen closest one onwards
        if (closest_idx >= 0) {
            future_path.assign(previous_path_.begin() + closest_idx, previous_path_.end());
        }
    }

    // ----------------------------------------------------
    // Determine planning start point
    // ----------------------------------------------------
    Point planning_start = robot_pos;
    std::vector<Point> kept_path;
    
    if (future_path.size() > KEEP_STEPS) {
        // Keep first KEEP_STEPS waypoints ahead, replan from waypoint KEEP_STEPS
        kept_path.assign(future_path.begin(), future_path.begin() + KEEP_STEPS);
        planning_start = future_path[KEEP_STEPS];
        
        RCLCPP_INFO(this->get_logger(), 
                    "Keeping %d waypoints, replanning from grid=(%d,%d) world=(%.2f,%.2f)", 
                    KEEP_STEPS, planning_start.x, planning_start.y,
                    global_map_.gridToWorldX(planning_start.x),
                    global_map_.gridToWorldY(planning_start.y));
    } else if (!future_path.empty()) {
        // Keep all future waypoints, replan from the last one
        kept_path = future_path;
        planning_start = future_path.back();
        
        RCLCPP_INFO(this->get_logger(), 
                    "Keeping all %zu future waypoints, replanning from last", kept_path.size());
    } else {
        // No valid previous path - plan from robot
        RCLCPP_INFO(this->get_logger(), 
                    "No previous path. Planning from robot position grid=(%d,%d)", 
                    robot_pos.x, robot_pos.y);
    }

    // ----------------------------------------------------
    // A* Planning
    // ----------------------------------------------------
    auto new_path_segment = planner_.plan(global_map_, planning_start, goal);

    // Avoid duplicating the planning_start point
    if (!new_path_segment.empty() && !kept_path.empty()) {
        new_path_segment.erase(new_path_segment.begin());
    }

    // Combine kept + new path
    std::vector<Point> full_path = kept_path;
    full_path.insert(full_path.end(), new_path_segment.begin(), new_path_segment.end());

    // Store for next iteration
    previous_path_ = full_path;

    // Publish the map
    publishDistanceMap(); 

    // ----------------------------------------------------
    // Publish Path
    // ----------------------------------------------------
    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp = this->now();
    path_msg.header.frame_id = "map";

    for (auto &pt : full_path) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = path_msg.header;

        pose.pose.position.x = global_map_.gridToWorldX(pt.x);
        pose.pose.position.y = global_map_.gridToWorldY(pt.y);
        pose.pose.position.z = 0.0;

        path_msg.poses.push_back(pose);
    }

    path_pub_->publish(path_msg);
    
    RCLCPP_INFO(this->get_logger(), 
                "Published: %zu kept + %zu new = %zu total waypoints", 
                kept_path.size(), new_path_segment.size(), full_path.size());
}


void DynamicPlannerNode::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    // ----------------------------------------------------
    // 1. TF lookup
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
    // yaw not currently used; omit to avoid unused-variable warning

    // ----------------------------------------------------
    // 2. Update maps
    // ----------------------------------------------------
    local_map_.clear();
    local_map_.updateFromScan(
        msg->ranges,
        msg->angle_min,
        msg->angle_increment,
        msg->range_max
    );

    int robot_gx = global_map_.worldToGridX(rx);
    int robot_gy = global_map_.worldToGridY(ry);

    global_map_.fuse(local_map_, robot_gx, robot_gy);
    global_map_.computeDistanceTransform();
}

} // namespace dynamic_planner

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<dynamic_planner::DynamicPlannerNode>());
    rclcpp::shutdown();
    return 0;
}