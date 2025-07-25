import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Node 1: Start Carla Publisher
        Node(
            package='vd_state_estimate',
            executable='gps_node',
            name='gps_node',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),

        #Node 2: Start MPC Controller
        Node(
            package='vd_state_estimate',
            executable='IMU_node',
            name='IMU_node',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),

        # Node 3: Start ROS Bag Node
        # Node(
        #     package='vd_state_estimate',
        #     executable='EKF_node',
        #     name='EKF_node',
        #     output='screen',
        #     parameters=[{'use_sim_time': True}]
        # )
    ])

