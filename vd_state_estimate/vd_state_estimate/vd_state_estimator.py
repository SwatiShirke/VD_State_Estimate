import rclpy
from rclpy.node import Node 
import numpy as np
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import Imu
from vd_state_estimate.EKF import EKF 



#This is a ros node for state estimation
#creates a filter object
## recived IMU data, the perform update and publish the state
## recives GPS data, then perform correct

class state_estim_node(Node):
    def __init__(self):
        super().__init__('state_estimation')
        self.publisher = self.create_publisher(Pose2D,"/vehicle_est_pose", 1)
        self.create_subscription(Imu, 'IMU_data', self.IMU_callback ,10)
        self.create_subscription(Pose2D, 'GPS_location', self.GPS_callback ,10)

        #setup EKF
        #state = [x, y, theta, Vx, Vy, b_accx, b_accy, b_gyro]
        #get intial state value (x,y) from GPS
        
        self.GPS_init_flag = False              #wether initial reading got from reading or not 
        self.X = np.array([0,0, 0,0,0,0,0,0])
        self.P = np.zeros((8,8))        
        w_g = 0.01
        w_a = 0.01
        w_ba = [0.1, 0.1]
        w_bg = 0.001
        
        self.dt = 0.001 # 1000 hz - freuqncy of IMU node
        self.EKF = EKF(self.P, w_a, w_g, w_ba, w_bg, self.dt)

        
    def IMU_callback(self, msg):
        if not self.GPS_init_flag:
            pass
        
        else:#estimate using filter and publish
            acc_x = msg.linear_acceleration.x
            acc_y = msg.linear_acceleration.y
            omega = msg.angular_velocity.z
            U = np.array([acc_x, acc_y, omega])
            print("U", U)
            self.X, self.P = self.EKF.predict(self.X, U, self.P)

            #publish 
            pose_msg = Pose2D()
            pose_msg.x = self.X[0]
            pose_msg.y = self.X[1]
            pose_msg.theta = self.X[2]
            self.publisher.publish(pose_msg)

    def GPS_callback(self, msg):
        if not self.GPS_init_flag:
            self.X = np.array([msg.x, msg.y,0,0,0,0,0,0])
            self.GPS_init_flag = True
        else:
            #update using filter
            z = np.array([msg.x, msg.y])
            R = np.eye(2) * 0.001
            self.X, self.P =  self.EKF.update(self.X, self.P, z, R)


def main():
    # Initialize ROS
    rclpy.init()
     
    node = state_estim_node()

    try:
        rclpy.spin(node)
    finally:
        rclpy.shutdown()        

         
if __name__ == "__main__":
    main()

