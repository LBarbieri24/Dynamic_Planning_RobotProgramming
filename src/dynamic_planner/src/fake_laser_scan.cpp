#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <cmath>
#include <vector>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

//-----------------------------------------------------------
// FAKE SCAN NODE: Simulates a laser scanner for testing
//-----------------------------------------------------------
class FakeScan : public rclcpp::Node {
public:
    FakeScan() : Node("fake_scan")  
    {
        // Publisher: Sends laser scan messages to /scan topic
        pub_ = create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);

        // TF system: Track robot position and orientation
        // tf buffer stores transforms between coordinate frames and get_clock() provides time reference
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Timer: Call publishScan() every 100 milliseconds
        timer_ = create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&FakeScan::publishScan, this)
        );
    }

private:
    // ROS interfaces
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Obstacle definition
    struct Obstacle {
        float x, y;    // Position in world coordinates (meters)
        float radius;  // Size of obstacle (meters)
    };

    // List of obstacles in the simulated world
    std::vector<Obstacle> obstacles_ = {
        {2.0, -0.5, 0.15},  // Three obstacles forming a vertical line at x=2.0
        {2.0,  0.0, 0.15},
        {2.0,  0.5, 0.15},
        {3.0,  1.5, 0.15},  // Five obstacles forming a horizontal line at y=1.5
        {3.5,  1.5, 0.15},  
        {4.0,  1.5, 0.15},
        {4.5,  1.5, 0.15},  
        {5.0,  1.5, 0.15},
        {4.2,  0.3, 0.20}   // Single obstacle near goal
    };

 
    // Generate and publish a fake laser scan based on obstacles
    void publishScan()
    {
        // Get robot's current position from TF
        geometry_msgs::msg::TransformStamped tf;
        try {
            tf = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
        } catch (const tf2::TransformException& ex) {
            // RCLCPP_WARN_THROTTLE: Only print warning once per 1000ms (avoid spam)
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, 
                                "TF lookup failed: %s", ex.what());
            return;  // Can't generate scan without robot position
        }

        // Extract robot position
        float rx = tf.transform.translation.x;
        float ry = tf.transform.translation.y;

        // Create LaserScan message
        sensor_msgs::msg::LaserScan scan;
        scan.header.stamp = now();
        scan.header.frame_id = "base_link";  // Scan is in robot's frame

        // Laser parameters (simulating a 180° laser)
        scan.angle_min = -1.57;        // -90° in radians
        scan.angle_max =  1.57;        // +90° in radians
        scan.angle_increment = 0.01;   // 0.57° between rays (high resolution)
        scan.range_min = 0.05;         // Minimum detection distance (5cm)
        scan.range_max = 2.0;          // Maximum detection distance (2m)

        // Calculate number of rays
        int n = (scan.angle_max - scan.angle_min) / scan.angle_increment;
        scan.ranges.resize(n);  // Allocate space for all measurements

        // Get robot's heading angle
        float robot_yaw = tf2::getYaw(tf.transform.rotation);

        // Simulate each laser ray
        for (int i = 0; i < n; i++) {
            // Calculate ray angle in robot's frame
            float angle_base = scan.angle_min + i * scan.angle_increment;
            
            // Transform to map frame (add robot's heading)
            float angle_map = robot_yaw + angle_base;
            
            // Default: No obstacle detected (max range)
            float best = scan.range_max;

            // Ray direction vector in map frame
            float dx = std::cos(angle_map);
            float dy = std::sin(angle_map);

            // Check each obstacle for intersection with this ray
            for (auto &obs : obstacles_) {  
                                             
                
                // Vector from robot to obstacle center
                float ox = obs.x - rx;
                float oy = obs.y - ry;

                // PROJECT obstacle onto ray direction
                // Dot product tells us how far along the ray the obstacle center is
                float proj = ox * dx + oy * dy;
                
                if (proj > 0) {  // Obstacle is in front of robot
                    
                    // PERPENDICULAR DISTANCE: How far is obstacle from ray line?
                    float perp = std::fabs(-dy * ox + dx * oy);
                    
                    // If perpendicular distance < radius, ray hits obstacle
                    if (perp < obs.radius) {
                        // Update closest hit distance
                        if (proj < best)
                            best = proj;
                    }
                }
            }

            // Store measurement for this ray
            scan.ranges[i] = best;
        }

        // Publish the simulated scan
        pub_->publish(scan);
    }
};



int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);  // Initialize ROS2
    rclcpp::spin(std::make_shared<FakeScan>());  // Run the node
    rclcpp::shutdown();  // Clean shutdown
    return 0;
}