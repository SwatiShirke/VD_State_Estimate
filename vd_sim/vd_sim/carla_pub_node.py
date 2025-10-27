import carla
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, Quaternion
import math
from rclpy.clock import Clock

import numpy as np
from std_msgs.msg import Float32
from rclpy.qos import QoSProfile, QoSHistoryPolicy, ReliabilityPolicy, DurabilityPolicy


ref_vel = 15.0
class CarlaRosPublisher(Node):
    def __init__(self, VEHICLE_ROLE_NAME):
        super().__init__('carla_ros_publisher')
        self.clock = Clock() #wall clock
        self.sim_clock = self.get_clock() #sim clock
        # Connect to CARLA
        self.client = carla.Client('localhost', 2000)
         # Get the current ROS2 time
        
        self.client.set_timeout(10.0)
        self.world = self.client.get_world()
        self.T_pred = 0.1
        self.N = 10 
        # Retrieve the vehicle using role name
        self.role_name = VEHICLE_ROLE_NAME
        self.vehicle = self.get_vehicle_by_role_name()

        if self.vehicle == None:
            raise RuntimeError(f"Vehicle with ID {self.role_name} not found!")        
        #ROS 2 Publishers
        qos_profile = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE)

        self.odom_pub = self.create_publisher(Odometry, '/carla/ego_vehicle/odometry', qos_profile)
        self.waypoints_pub = self.create_publisher(Path, '/carla/ego_vehicle/waypoints', qos_profile)
        self.err_pub = self.create_publisher(Float32, '/norm_error', qos_profile)
        self.ref_waypoint = None
        self.current_loc = None
        self.goal_loc = carla.Location(x=225, y=-50, z=1)

        # Timer for publishing at 20 Hz
        self.create_timer(self.T_pred, self.publish_data)  # 100 Hz

    def publish_data(self):
        """Publish odometry and trajectory data."""
        # =======================
        # Publish Odometry
        # =======================
        self.publish_odometry()

        #cal nd publish norm error 
        self.cal_error()
        # =======================
        # Publish Trajectory
        # =======================
        self.publish_waypoints()

    @staticmethod
    def euler_to_quaternion(roll, pitch, yaw):
        """Convert Euler angles to quaternion."""
        qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
        qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
        qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        return qx, qy, qz, qw

    def publish_odometry(self):
        
        vehicle_transform = self.vehicle.get_transform()
        vehicle_velocity = self.vehicle.get_velocity()
        self.current_loc = vehicle_transform

        odom_msg = Odometry()
        current_time = self.sim_clock.now()
        #print(current_time) 
        odom_msg.header.stamp = current_time.to_msg()
        #odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'map'

        # Position
        odom_msg.pose.pose.position.x = vehicle_transform.location.x
        odom_msg.pose.pose.position.y = vehicle_transform.location.y
        odom_msg.pose.pose.position.z = vehicle_transform.location.z

        # Orientation: Use Yaw, Pitch, Roll (in radians)
        yaw = (math.radians(vehicle_transform.rotation.yaw) + 2*np.pi) % (4*np.pi) - 2*np.pi
        pitch = math.radians(vehicle_transform.rotation.pitch)
        roll = math.radians(vehicle_transform.rotation.roll)

        # Optional: Add YPR as part of the covariance field
        # This is a hack if you want to include YPR without a custom message
        odom_msg.pose.pose.orientation.x = yaw
        odom_msg.pose.pose.orientation.y = pitch
        odom_msg.pose.pose.orientation.z = roll
                
        # Velocity
        # Velocity in longitudinal and lateral directions
        forward_vector = vehicle_transform.get_forward_vector()
        longitudinal_velocity = (vehicle_velocity.x * forward_vector.x +
                                 vehicle_velocity.y * forward_vector.y +
                                 vehicle_velocity.z * forward_vector.z)
        
        # Assigning longitudinal and lateral velocities to odometry message (optional fields)
        odom_msg.twist.twist.linear.x = longitudinal_velocity
        self.odom_pub.publish(odom_msg)
        
    
    def cal_error(self):
        if self.ref_waypoint !=None and self.current_loc != None:
            norm_error =  np.sqrt((self.ref_waypoint.location.x - self.current_loc.location.x)**2 + (self.ref_waypoint.location.y - self.current_loc.location.y)**2)
            float_msg = Float32()
            float_msg.data = norm_error
            #print("norm_error",norm_error)
            self.err_pub.publish(float_msg)


    def publish_waypoints(self, N=10):       
        # Retrieve waypoints
        waypoints = self.get_time_spanned_waypoints()
        #Create PoseArray for waypoints
        path_msg = Path()
        current_time = self.sim_clock.now()        
        path_msg.header.stamp = current_time.to_msg()
        path_msg.header.frame_id = 'map'

        way_point_list = []
        for i, wp in enumerate(waypoints):
            if i ==0:
                self.ref_waypoint = wp.transform

            pose_stamped = PoseStamped()
            pose_stamped.header = path_msg.header
            current_time = self.sim_clock.now()        
            pose_stamped.header.stamp = current_time.to_msg()
            pose_stamped.pose.position.x = wp.transform.location.x
            pose_stamped.pose.position.y = wp.transform.location.y
            pose_stamped.pose.position.z = wp.transform.location.z

            yaw = (math.radians(wp.transform.rotation.yaw) + 2*np.pi) % (4*np.pi) - 2*np.pi
            # pitch = math.radians(wp.transform.rotation.pitch)
            # roll = math.radians(wp.transform.rotation.roll)
            #x, y,z, w = self.euler_to_quaternion(roll, pitch, yaw)

            
            pose_stamped.pose.orientation.x = yaw#% 2 *math.pi
            pose_stamped.pose.orientation.y = 0.0
            pose_stamped.pose.orientation.z = 0.0
            pose_stamped.pose.orientation.w = ref_vel  # self.vehicle.get_speed_limit()

            pose_stamped.pose
            #print("wavepoint :", wp.transform.location.x, " " ,wp.transform.location.y," ", wp.transform.rotation.yaw)         
            path_msg.poses.append(pose_stamped)  
        self.waypoints_pub.publish(path_msg)

    def euler_to_quaternion(roll, pitch, yaw):
        """
        Convert Euler angles (in radians) to Quaternion (x, y, z, w).
        """
        qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
        qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
        qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        return qx, qy, qz, qw

    def get_time_spanned_waypoints(self, goal_point, threshold=2.0):
        """
        Generate waypoints spaced at consistent time intervals. 
        If any waypoint is close to the goal point, all subsequent waypoints are set to the goal.
    
        Args:
            goal_point (carla.Location): The target goal location.
            threshold (float): Distance threshold (meters) to consider a waypoint close to the goal.
    
        Returns:
            List of time-spanned waypoints.
        """
        map = self.vehicle.get_world().get_map()
        current_waypoint = map.get_waypoint(self.vehicle.get_transform().location)
        waypoints = []
        
        goal_reached = False  # Flag to check if goal is close
    
        for _ in range(int(self.N)):
            if goal_reached:
                waypoints.append(map.get_waypoint(goal_point))  # Set remaining waypoints to goal
                continue
            
            velocity = self.vehicle.get_velocity()
            forward_vector = self.vehicle.get_transform().get_forward_vector()
            longitudinal_speed = (velocity.x * forward_vector.x +
                                  velocity.y * forward_vector.y +
                                  velocity.z * forward_vector.z)
    
            # Calculate the distance to the next waypoint
            distance = max(longitudinal_speed * self.T_pred, 0.1)  # Ensure non-zero distance
            
            next_waypoints = current_waypoint.next(distance)
    
            if next_waypoints:
                current_waypoint = next_waypoints[0]  # Use the first waypoint in the list
                
                # Check if the waypoint is close to the goal
                if current_waypoint.transform.location.distance(goal_point) <= threshold:
                    goal_reached = True
                    waypoints.append(map.get_waypoint(goal_point))  # First waypoint set to goal
                else:
                    waypoints.append(current_waypoint)
            else:
                break  # No more waypoints available (end of the road)
            
        return waypoints

    def get_time_spanned_waypoints(self):
        """
        Generate waypoints spaced at consistent time intervals.
    
        Args:
            vehicle: CARLA vehicle object
            map: CARLA map object
            total_time: Total time horizon (seconds)
            delta_t: Time interval between waypoints (seconds)
    
        Returns:
            List of time-spanned waypoints
        """
        map = self.vehicle.get_world().get_map()
        current_waypoint = map.get_waypoint(self.vehicle.get_transform().location)
        waypoints = []
        
        for _ in range(int(self.N)):
            velocity = self.vehicle.get_velocity()
            forward_vector = self.vehicle.get_transform().get_forward_vector()
            longitudinal_speed = (velocity.x * forward_vector.x +
                                  velocity.y * forward_vector.y +
                                  velocity.z * forward_vector.z)
            # Calculate the distance to the next waypoint
            distance = max(longitudinal_speed * self.T_pred, 0.1 ) # Ensure non-zero distance
            #print("dist ", distance)
            next_waypoints = current_waypoint.next(distance)
        
            if next_waypoints:
                current_waypoint = next_waypoints[0]  # Use the first waypoint in the list
                waypoints.append(current_waypoint)
            else:
                break  # No more waypoints available (end of the road)
            
        return waypoints

    def get_vehicle_by_role_name(self):
        vehicles = self.world.get_actors().filter('vehicle.*')
        for vehicle in vehicles:
            if vehicle.attributes.get('role_name') == self.role_name:
                print(f"Found vehicle with role_name: {self.role_name}, ID: {vehicle.id}")
                return vehicle

def main():
    # Initialize ROS
    rclpy.init()

    # Vehicle role name (shared between nodes or retrieved dynamically)
    VEHICLE_ROLE_NAME = "hero" # Replace with the actual vehicle ID

    # Create the publisher node
    node = CarlaRosPublisher(VEHICLE_ROLE_NAME)

    try:
        rclpy.spin(node)
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
