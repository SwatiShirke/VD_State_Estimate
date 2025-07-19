import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Node 1: Start Carla Publisher
        Node(
            package='vd_sim',
            executable='carla_pub_node',
            name='carla_publisher',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),

        #Node 2: Start MPC Controller
        Node(
            package='vd_mpc_controller',
            executable='vd_nmpc_node',
            name='mpc_controller',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),

        # Node 3: Start ROS Bag Node
        Node(
            package='vd_sim',
            executable='ros_bag_node',
            name='ros_bag_recorder',
            output='screen',
            parameters=[{'use_sim_time': True}]
        )
    ])
