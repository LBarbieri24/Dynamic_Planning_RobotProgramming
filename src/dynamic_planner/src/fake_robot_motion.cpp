#include <chrono>
#include <cmath>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/path.hpp>

using namespace std::chrono_literals;

//-----------------------------------------------------------
// FAKE ROBOT: Simulates a robot following a path
//-----------------------------------------------------------
class FakeRobot : public rclcpp::Node
{
public:
    FakeRobot() : Node("fake_robot"),   // Initialize ROS2 node
                  x_(0.0),               // Start at origin (0, 0)
                  y_(0.0),
                  speed_(0.02)           // Move 2cm per update (slow for testing)
    {
        RCLCPP_INFO(get_logger(), "FakeRobot started");

        // Listen for path messages from planner
        path_sub_ = create_subscription<nav_msgs::msg::Path>(
            "/planner/path", 10,
            std::bind(&FakeRobot::pathCallback, this, std::placeholders::_1)
        );

        // Update robot position every 100ms
        timer_ = create_wall_timer(
            100ms, std::bind(&FakeRobot::update, this)
        );

        // TF BROADCASTER: Publish robot's position to TF tree
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

private:
    double x_, y_;     // Current position (meters)
    double speed_;     // Movement speed (meters per update)

    std::vector<geometry_msgs::msg::PoseStamped> path_;  // Current path to follow

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;


    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        // Store the path waypoints
        path_ = msg->poses;
        RCLCPP_INFO(get_logger(), "Received path with %zu poses", path_.size());
    }

    //-----------------------------------------------------------
    // BROADCAST TF: Publish robot position to TF tree
    //-----------------------------------------------------------
    // TF (Transform) tree: Allows other nodes to know where robot is
    void broadcastTF()
    {
        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp = now();              // Current timestamp
        tf.header.frame_id = "map";           // Parent frame
        tf.child_frame_id = "base_link";      // Child frame (robot)

        // Set robot position
        tf.transform.translation.x = x_;
        tf.transform.translation.y = y_;
        tf.transform.translation.z = 0.0;     // 2D robot (no height)

        // No rotation (facing along X axis)
        tf.transform.rotation.w = 1.0;
        tf.transform.rotation.x = 0.0;
        tf.transform.rotation.y = 0.0;
        tf.transform.rotation.z = 0.0;

        // Broadcast the transform
        tf_broadcaster_->sendTransform(tf);
    }


    // Move robot toward next waypoint
    void update()
    {
        // Always publish TF (even if not moving)
        broadcastTF();

        // Check if we have a path to follow
        if (path_.empty()) {
            return;  // No path yet, nothing to do
        }

        // Get the first waypoint (target)
        auto target = path_.front().pose.position;

        // Calculate vector from robot to target
        double dx = target.x - x_;
        double dy = target.y - y_;
        double dist = std::sqrt(dx*dx + dy*dy);  // Euclidean distance

        RCLCPP_INFO(get_logger(),
                    "Robot pos=(%.3f,%.3f) target=(%.3f,%.3f) dist=%.3f",
                    x_, y_, target.x, target.y, dist);

        // Waypoint reached: If close enough, move to next waypoint
        if (dist < 0.02) {  // Within 2cm = reached
            path_.erase(path_.begin());  // Remove first element
            RCLCPP_INFO(get_logger(), "Reached waypoint, %zu remaining", path_.size());
            return;
        }

        // Calculate velocity to move toward target
        double vx = speed_ * dx / dist;  // X component of velocity
        double vy = speed_ * dy / dist;  // Y component of velocity

        // Update position
        x_ += vx;
        y_ += vy;
    }
};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);  // Initialize ROS2
    rclcpp::spin(std::make_shared<FakeRobot>());  // Run the node
    rclcpp::shutdown();  // Clean shutdown
    return 0;
}