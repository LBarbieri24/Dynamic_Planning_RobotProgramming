#include <chrono>
#include <cmath>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/path.hpp>

using namespace std::chrono_literals;

class FakeRobot : public rclcpp::Node
{
public:
    FakeRobot() : Node("fake_robot"), x_(0.0), y_(0.0), speed_(0.02)
    {
        RCLCPP_INFO(get_logger(), "FakeRobot started");

        path_sub_ = create_subscription<nav_msgs::msg::Path>(
            "/planner/path", 10,
            std::bind(&FakeRobot::pathCallback, this, std::placeholders::_1)
        );

        timer_ = create_wall_timer(
            100ms, std::bind(&FakeRobot::update, this)
        );

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

private:
    // Robot state
    double x_, y_;
    double speed_;

    std::vector<geometry_msgs::msg::PoseStamped> path_;

    // ROS interfaces
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        path_ = msg->poses;
        RCLCPP_INFO(get_logger(), "Received path with %zu poses", path_.size());
    }

    void broadcastTF()
    {
        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp = now();
        tf.header.frame_id = "map";
        tf.child_frame_id = "base_link";

        tf.transform.translation.x = x_;
        tf.transform.translation.y = y_;
        tf.transform.translation.z = 0.0;

        tf.transform.rotation.w = 1.0;
        tf.transform.rotation.x = 0.0;
        tf.transform.rotation.y = 0.0;
        tf.transform.rotation.z = 0.0;

        tf_broadcaster_->sendTransform(tf);
    }

    void update()
    {
        // Publish TF always
        broadcastTF();

        if (path_.empty()) {
            // No path yet
            return;
        }

        auto target = path_.front().pose.position;

        double dx = target.x - x_;
        double dy = target.y - y_;
        double dist = std::sqrt(dx*dx + dy*dy);

        RCLCPP_INFO(get_logger(),
                    "Robot pos=(%.3f,%.3f) target=(%.3f,%.3f) dist=%.3f",
                    x_, y_, target.x, target.y, dist);

        // Close enough → pop waypoint
        if (dist < 0.02) {
            path_.erase(path_.begin());
            RCLCPP_INFO(get_logger(), "Reached waypoint, %zu remaining", path_.size());
            return;
        }

        // Move toward waypoint
        double vx = speed_ * dx / dist;
        double vy = speed_ * dy / dist;

        x_ += vx;
        y_ += vy;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FakeRobot>());
    rclcpp::shutdown();
    return 0;
}
