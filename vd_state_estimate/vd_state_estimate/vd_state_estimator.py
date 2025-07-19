from rclpy.node import Node 
import numpy as np
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import Imu
from EKF import EKF


#This is a ros node for state estimation
#creates a filter object
## recived IMU data, the perform update and publish the state
## recives GPS data, then perform correct

class state_estim_node(Node):
    def __init__(self):
        super().__init__('state_estimation')
        self.publisher = self.create_publisher(Pose2D,"vehicle_pose", 1)
        self.create_subscription(Imu, 'IMU_data', self.IMU_callback ,10)
        self.create_subscription(Pose2D, 'GPS_location', self.GPS_callback ,10)

        #setup EKF
        #state = [x, y, theta, Vx, Vy, b_accx, b_accy, b_gyro]
        #get intial state value (x,y) from GPS
        x0, y0 = 0,0 #GPS_service()
        self.X = np.array([x0,y0, 0,0,0,0,0,0]).reshape(8,1)
        self.P = np.zeros((8,8))        
        w_g = 0.001
        w_a = 0.001
        w_ba = 0.001
        w_bg = 0.001
        self.EKF = EKF(P, w_g, w_a, w_ba, w_bg)

        
    def IMU_callback(self, msg):
        #estimate using filter and publish
        acc_x = msg.linear_acceleration.x
        acc_y = msg.linear_acceleration.y
        omega = msg.angular_velocity.z
        U = np.array([acc_x, acc_y, omega])
        self.X, self.P = self.EKF.predict(self.X, U)
        
        #publish 
        pose_msg = Pose2D()
        pose_msg.x = self.X[0,0]
        pose_msg.y = self.X[1,0]
        pose_msg.theta = self.X[2,0]
        self.publisher.publish(pose_msg)

    def GPS_callback(self, msg):
        #update using filter
        z = np.array([msg.x, msg.y])
        R = np.eye(2) * 0.001
        self.X, self.P =  self.EKF.update(self.X, self.P, z, R)
         

         