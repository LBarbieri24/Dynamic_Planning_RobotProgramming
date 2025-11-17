#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <chrono>


// Include my classes
#include "map.hpp"
#include "planner.hpp"

namespace dynamic_planner {

class DynamicPlannerNode : public rclcpp::Node {
public:
    DynamicPlannerNode();

private:
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void publishDistanceMap();
    void planningTimerCallback();

    // Stores a full global map of the environment that grows over time as laser scans integrate
    Map global_map_;
    // Stores a small, robot-centered map that is rebuilt at every scan
    Map local_map_;
    // takes the map and finds a path between two points
    Planner planner_;
    // Store the previous path to retain waypoints
    std::vector<Point> previous_path_;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::TimerBase::SharedPtr planning_timer_;

    
};

} // namespace dynamic_planner
