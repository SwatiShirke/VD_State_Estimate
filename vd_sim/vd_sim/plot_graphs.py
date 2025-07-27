import os
import rosbag2_py
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker  # Import ticker for formatting
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry, Path
from carla_msgs.msg import CarlaEgoVehicleControl
from std_msgs.msg import Float32  # Import Float32 for norm_error topic
import numpy as np
from geometry_msgs.msg import Pose2D

def read_vehicle_bag_data(bag_path):
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')
    reader.open(storage_options, converter_options)

    topics = [
        '/carla/ego_vehicle/odometry',
        '/carla/ego_vehicle/vehicle_control_cmd',
        '/carla/ego_vehicle/waypoints',
        '/norm_error',  # Include norm_error topic,
        '/vehicle_est_pose'
    ]

    topic_data = {topic: [] for topic in topics}

    while reader.has_next():
        topic_name, serialized_msg, t = reader.read_next()
        #print(topic_name, " ", t)
        
        # Convert nanoseconds to seconds
        time_sec = t * 1e-9

        if topic_name in topics:
            if topic_name == '/carla/ego_vehicle/odometry':
                msg = deserialize_message(serialized_msg, Odometry)
                topic_data[topic_name].append((time_sec, msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z,
                                               msg.pose.pose.orientation.x,  # Yaw angle
                                               msg.twist.twist.linear.x))  # Longitudinal velocity
            elif topic_name == '/carla/ego_vehicle/vehicle_control_cmd':
                msg = deserialize_message(serialized_msg, CarlaEgoVehicleControl)
                topic_data[topic_name].append((time_sec, msg.throttle, msg.brake, msg.steer))
            elif topic_name == '/carla/ego_vehicle/waypoints':
                msg = deserialize_message(serialized_msg, Path)
                if len(msg.poses) > 0:
                    first_pose = msg.poses[0].pose
                    topic_data[topic_name].append((time_sec, first_pose.position.x, first_pose.position.y, first_pose.position.z,
                                                   first_pose.orientation.x,  # Yaw
                                                   first_pose.orientation.w))  # Reference velocity
            elif topic_name == '/norm_error':  # Read norm error values
                msg = deserialize_message(serialized_msg, Float32)
                topic_data[topic_name].append((time_sec, msg.data))  # Store norm_error values
            elif topic_name == '/vehicle_est_pose': # read estimated vehicle pose
                print("got estimated pose data")
                msg = deserialize_message(serialized_msg, Pose2D)
                topic_data[topic_name].append((msg.x, msg.y, msg.theta)) 
                

    return topic_data

def calculate_rmse(ref_x, ref_y, actual_x, actual_y):
    """ Compute RMSE (Root Mean Square Error) for trajectory tracking """
    ref_x, ref_y = np.array(ref_x), np.array(ref_y)
    actual_x, actual_y = np.array(actual_x), np.array(actual_y)

    min_length = min(len(ref_x), len(actual_x))  # Ensure matching size
    ref_x, ref_y = ref_x[:min_length], ref_y[:min_length]
    actual_x, actual_y = actual_x[:min_length], actual_y[:min_length]

    errors = np.sqrt((actual_x - ref_x) ** 2 + (actual_y - ref_y) ** 2)
    rmse = np.sqrt(np.mean(errors ** 2))

    print(f"🚀 Trajectory Tracking RMSE: {rmse:.4f} meters")
    return rmse

def compute_norm_error_rmse(topic_data):
    """ Compute RMSE for norm error """
    if '/norm_error' in topic_data and len(topic_data['/norm_error']) > 0:
        _, norm_errors = zip(*topic_data['/norm_error'])  # Extract norm error values
        norm_errors = np.array(norm_errors)

        rmse = np.sqrt(np.mean(norm_errors ** 2))  # Compute RMSE
        print(f"🔥 Norm Error RMSE: {rmse:.4f} meters")
        return rmse
    else:
        print("⚠️ No norm error data found in the ROS bag.")
        return None

def plot_vehicle_data(topic_data):
    # if '/carla/ego_vehicle/odometry' in topic_data and '/carla/ego_vehicle/waypoints' in topic_data:
    #     odom_times, odom_x, odom_y, odom_z, odom_yaw, odom_long_vel = zip(*topic_data['/carla/ego_vehicle/odometry'])
    #     traj_times, traj_x, traj_y, traj_z, traj_yaw, traj_ref_vel = zip(*topic_data['/carla/ego_vehicle/waypoints'])
               

    #     plt.figure()
    #     plt.plot(odom_times, odom_x, label='Vehicle X')
    #     plt.plot(traj_times, traj_x, label='Waypoints X', linestyle='--')
    #     plt.legend()
    #     plt.xlabel('Time (seconds)')
    #     plt.ylabel('X Position')
    #     plt.title('Vehicle X Position vs Waypoints')
    #     plt.grid(True)
    #     plt.gca().xaxis.set_major_formatter(ticker.ScalarFormatter(useOffset=False))  # Fix time axis
        
    #     # Plot Y Position
    #     plt.figure()
    #     plt.plot(odom_times, odom_y, label='Vehicle Y')
    #     plt.plot(traj_times, traj_y, label='Waypoints Y', linestyle='--')
    #     plt.legend()
    #     plt.xlabel('Time (seconds)')
    #     plt.ylabel('Y Position')
    #     plt.title('Vehicle Y Position vs Waypoints')
    #     plt.grid(True)
    #     plt.gca().xaxis.set_major_formatter(ticker.ScalarFormatter(useOffset=False))

    #     # Plot Yaw Angle
    #     plt.figure()
    #     plt.plot(odom_times, odom_yaw, label='Yaw Angle')
    #     plt.plot(traj_times, traj_yaw, label='Reference Yaw', linestyle='--')
    #     plt.legend()
    #     plt.xlabel('Time (seconds)')
    #     plt.ylabel('Yaw Angle')
    #     plt.title('Yaw Angle vs Reference Yaw')
    #     plt.grid(True)
    #     plt.gca().xaxis.set_major_formatter(ticker.ScalarFormatter(useOffset=False))

    #     # Plot Longitudinal Velocity
    #     plt.figure()
    #     plt.plot(odom_times, odom_long_vel, label='Longitudinal Velocity')
    #     plt.plot(traj_times, traj_ref_vel, label='Reference Velocity', linestyle='--')
    #     plt.legend()
    #     plt.xlabel('Time (seconds)')
    #     plt.ylabel('Velocity')
    #     plt.title('Longitudinal Velocity vs Reference Velocity')
    #     plt.grid(True)
    #     plt.gca().xaxis.set_major_formatter(ticker.ScalarFormatter(useOffset=False))

    # if '/carla/ego_vehicle/vehicle_control_cmd' in topic_data:
    #     times, throttle, brake, steer = zip(*topic_data['/carla/ego_vehicle/vehicle_control_cmd'])
        
    #     plt.figure()
    #     plt.plot(times, throttle, label='Throttle')
    #     plt.xlabel('Time (seconds)')
    #     plt.ylabel('Throttle')
    #     plt.title('Throttle Command Over Time')
    #     plt.grid(True)
    #     plt.gca().xaxis.set_major_formatter(ticker.ScalarFormatter(useOffset=False))

    #     plt.figure()
    #     plt.plot(times, brake, label='Brake')
    #     plt.xlabel('Time (seconds)')
    #     plt.ylabel('Brake')
    #     plt.title('Brake Command Over Time')
    #     plt.grid(True)
    #     plt.gca().xaxis.set_major_formatter(ticker.ScalarFormatter(useOffset=False))

    #     plt.figure()
    #     plt.plot(times, steer, label='Steer')
    #     plt.xlabel('Time (seconds)')
    #     plt.ylabel('Steering Angle')
    #     plt.title('Steering Command Over Time')
    #     plt.grid(True)
    #     plt.gca().xaxis.set_major_formatter(ticker.ScalarFormatter(useOffset=False))

    if '/vehicle_est_pose' in topic_data and '/carla/ego_vehicle/odometry' in topic_data:
        est_x, est_y, est_theta = zip(*topic_data['/vehicle_est_pose'])
        odom_times, odom_x, odom_y, odom_z, odom_yaw, odom_long_vel = zip(*topic_data['/carla/ego_vehicle/odometry'])
        print(len(est_y))
        print(len(odom_x))
        calculate_rmse(est_x, est_y, odom_x, odom_y) 
        # Plot X Position
        plt.figure()
        plt.plot(est_x, est_y, label='Estimated Vehicle Pose')
        plt.plot(odom_x, odom_y, label='Ground Treuth Vehicle Pose X', linestyle='--')
        plt.legend()
        plt.xlabel('X Position')
        plt.ylabel('Y Position')
        #plt.ylim(0, 48)  # <-- Set Y-axis limits here
        plt.title('Estimated Vs Ground truth Position')
        plt.grid(True)
        plt.gca().xaxis.set_major_formatter(ticker.ScalarFormatter(useOffset=False))  # Fix time axis

    plt.legend()
    plt.show()

# Read and process data
bag_path = "/home/swati/Adv_Robot_Nav/VD_State_Estimation/multi_topic_bag"
data = read_vehicle_bag_data(bag_path)

# Compute RMSE for norm error
compute_norm_error_rmse(data)

# Plot data
plot_vehicle_data(data)

