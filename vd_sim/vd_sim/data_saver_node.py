import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from carla_msgs.msg import CarlaEgoVehicleControl
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, Quaternion
import rosbag2_py
from rclpy.qos import QoSProfile, QoSHistoryPolicy, ReliabilityPolicy, DurabilityPolicy
import shutil
import os
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose2D
from vd_msgs.msg import VDpose

class BagWriterNode(Node):
    def __init__(self):
        super().__init__('bag_writer_node')

        

        # Initialize ROS bag writer
        self.bag_writer = rosbag2_py.SequentialWriter()

        # Bag file directory and cleanup if needed
        bag_dir = "multi_topic_bag"
        if os.path.exists(bag_dir):
            #self.get_logger().info(f"Removing existing directory: {bag_dir}")
            shutil.rmtree(bag_dir)  # Remove existing directory

        # Set storage options (bag file path and storage format)
        storage_options = rosbag2_py.StorageOptions(
            uri='multi_topic_bag',  # Folder name for the bag file
            storage_id='sqlite3'
        )
        converter_options = rosbag2_py.ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr'
        )
        self.bag_writer.open(storage_options, converter_options)

        # Create topics for saving messages
        self.create_bag_topic('/carla/ego_vehicle/odometry', 'nav_msgs/msg/Odometry')
        self.create_bag_topic('/carla/ego_vehicle/vehicle_control_cmd', 'carla_msgs/msg/CarlaEgoVehicleControl')
        self.create_bag_topic('/carla/ego_vehicle/waypoints', 'nav_msgs/msg/Path')
        self.create_bag_topic( '/norm_error', 'std_msgs/msg/Float32')
        self.create_bag_topic( '/vehicle_est_pose', 'vd_msgs/msg/VDpose')

        
        # Subscriptions
        qos_profile = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE)

        self.create_subscription(Odometry, "/carla/ego_vehicle/odometry", self.odom_callback, qos_profile)
        self.create_subscription(CarlaEgoVehicleControl, "/carla/ego_vehicle/vehicle_control_cmd", self.control_input_callback, qos_profile)
        self.create_subscription(Path, "/carla/ego_vehicle/waypoints", self.desired_traj_callback, qos_profile)
        self.create_subscription(Float32, "/norm_error", self.err_callback, qos_profile)
        self.create_subscription(VDpose, "/vehicle_est_pose", self.vehicle_est_pose_callback, qos_profile)

    def create_bag_topic(self, topic_name, type_name):
        """Create a topic in the ROS bag."""
        self.bag_writer.create_topic(
            rosbag2_py.TopicMetadata(
                name=topic_name,
                type=type_name,
                serialization_format='cdr'
            )
        )
        #self.get_logger().info(f"Created topic: {topic_name}")

    def err_callback(self, msg):
        """Handle err messages."""
        #self.get_logger().info(f"Received err message")
        self.write_to_bag("/norm_error", msg) 

    def odom_callback(self, msg):
        """Handle odometry messages."""
        #self.get_logger().info(f"Received Odometry message")
        self.write_to_bag("/carla/ego_vehicle/odometry", msg)

    def control_input_callback(self, msg):
        msg.steer = msg.steer * 0.7 # steer is between -1 to +1 between -0.7 to +0.7 radians
        """Handle control input messages."""
        #self.get_logger().info(f"Received FMNCommand message")
        self.write_to_bag("/carla/ego_vehicle/vehicle_control_cmd", msg)

    def desired_traj_callback(self, msg):
        """Handle reference trajectory messages."""
        if len(msg.poses) > 0:  # Ensure there is at least one point in the trajectory
            # Get the 0th point (first point)
            first_point = msg.poses[0]
            #self.get_logger().info(f"Received TrajCommand message with first point: {first_point}")
            self.write_to_bag("/carla/ego_vehicle/waypoints", msg)  # Save the entire TrajCommand message (or just the first point if needed)

    def write_to_bag(self, topic_name, message):
        """Write a message to the ROS bag."""
        #self.get_logger().info(f"Writing message to topic: {topic_name}")
        self.bag_writer.write(
            topic_name,
            self.serialize_message(message),
            self.get_clock().now().nanoseconds
        )

    def serialize_message(self, message):
        """Serialize a ROS message."""
        from rclpy.serialization import serialize_message
        return serialize_message(message)

    def destroy(self):
        """Ensure the bag writer is properly closed."""
        #self.get_logger().info("Closing bag writer")
        self.bag_writer.reset()
        super().destroy_node()

    def vehicle_est_pose_callback(self, msg):
        """Save data"""
        if msg:
            #self.get_logger().info(f"Received Estimated Vehicle Pose msg")
            self.write_to_bag("/vehicle_est_pose", msg)  


def main(args=None):
    rclpy.init(args=args)
    node = BagWriterNode()
        
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
