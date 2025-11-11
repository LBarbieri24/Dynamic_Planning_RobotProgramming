#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <cmath>
#include <vector>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class FakeScan : public rclcpp::Node {
public:
    FakeScan() : Node("fake_scan")
    {
        pub_ = create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        timer_ = create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&FakeScan::publishScan, this)
        );

    }

private:
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    struct Obstacle {
        float x, y, radius;
    };

    std::vector<Obstacle> obstacles_ = {
        {2.0, -0.5, 0.15},
        {2.0,  0.0, 0.15},
        {2.0,  0.5, 0.15},
        {3.0,  1.5, 0.15},
        {3.5,  1.5, 0.15},  
        {4.0,  1.5, 0.15},
        {4.5,  1.5, 0.15},  
        {5.0,  1.5, 0.15},
        {4.2,  0.3, 0.20}
    };

    void publishScan()
    {
        geometry_msgs::msg::TransformStamped tf;
        try {
            tf = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, 
                                "TF lookup failed: %s", ex.what());
        }

        float rx = tf.transform.translation.x;
        float ry = tf.transform.translation.y;

        sensor_msgs::msg::LaserScan scan;
        scan.header.stamp = now();
        scan.header.frame_id = "base_link";

        scan.angle_min = -1.57;
        scan.angle_max =  1.57;
        scan.angle_increment = 0.01;
        scan.range_min = 0.05;
        scan.range_max = 2.0;

        int n = (scan.angle_max - scan.angle_min) / scan.angle_increment;
        scan.ranges.resize(n);

        // Get robot yaw
        float robot_yaw = tf2::getYaw(tf.transform.rotation);

        for (int i = 0; i < n; i++) {
            float angle_base = scan.angle_min + i * scan.angle_increment;
            float angle_map = robot_yaw + angle_base;  // transform to map frame
            float best = scan.range_max;

            // Ray direction in map frame
            float dx = std::cos(angle_map);
            float dy = std::sin(angle_map);

            for (auto &obs : obstacles_) {
                float ox = obs.x - rx;
                float oy = obs.y - ry;

                float proj = ox * dx + oy * dy;  // projection onto ray
                if (proj > 0) {
                    float perp = std::fabs(-dy * ox + dx * oy);
                    if (perp < obs.radius) {
                        if (proj < best)
                            best = proj;
                    }
                }
            }

            scan.ranges[i] = best;
        }


        pub_->publish(scan);
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FakeScan>());
    rclcpp::shutdown();
    return 0;
}
