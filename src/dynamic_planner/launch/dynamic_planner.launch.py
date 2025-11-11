from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="dynamic_planner",
            executable="dynamic_planner_node",
            name="dynamic_planner",
            output="screen"
        ),

        Node(
            package="dynamic_planner",
            executable="fake_laser_scan",
            name="fake_laser_scan",
            output="screen"
        ),
        Node(
            package="dynamic_planner",
            executable="fake_robot",
            name="fake_robot",
            output="screen"
        ),


    ])
