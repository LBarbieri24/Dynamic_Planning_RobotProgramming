#pragma once

//Includes all core ROS2 C++ functionality
#include <rclcpp/rclcpp.hpp>
//Defines the LaserScan message type we subscribe to
#include <sensor_msgs/msg/laser_scan.hpp>
//Defines the message type we publish as output (a planned path).
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

// This makes the class a ROS2 node
class DynamicPlannerNode : public rclcpp::Node {
public:
    DynamicPlannerNode();

private:
    //function that gets called when a LaserScan arrives
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

    // Subscribe to LaserScan
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    // Become a publisher for the topic path
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    // ADDED
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    // TF
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::TimerBase::SharedPtr planning_timer_;

    
};

} // namespace dynamic_planner
