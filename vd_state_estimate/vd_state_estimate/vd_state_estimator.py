import rclpy
from rclpy.node import Node 
import numpy as np
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import Imu
from vd_state_estimate.EKF import EKF 
from vd_msgs.msg import VDpose
from rclpy.qos import QoSProfile, QoSHistoryPolicy, ReliabilityPolicy, DurabilityPolicy
from vd_msgs.msg import GPSpose
from rclpy.clock import Clock


#This is a ros node for state estimation
#creates a filter object
## recived IMU data, the perform update and publish the state
## recives GPS data, then perform correct

class state_estim_node(Node):
    def __init__(self):
        super().__init__('state_estimation')
        
        qos_profile = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE)

        self.publisher = self.create_publisher(VDpose,"/vehicle_est_pose", qos_profile)
        self.create_subscription(Imu, 'IMU_data', self.IMU_callback ,qos_profile)
        self.create_subscription(GPSpose, 'GPS_location', self.GPS_callback ,qos_profile)

        #setup EKF
        #state = [x, y, theta, Vx, Vy, b_accx, b_accy, b_gyro]
        #get intial state value (x,y) from GPS
        
        ##GPS 
        self.GPS_init_flag = False              #wether initial reading got from reading or not 
        self.X = np.array([0,0, 0,0,0,0,0,0])
        self.P = np.zeros((8,8)) 

        ##IMU noise      
        w_a = [0.0001, 0.0001]            ## std deviation accelerometer for both x and y m/s^2/hz # [160*9.8 * 1e-6 , 160*9.8 *1e-6]
        w_g = 0.0001               ## std deviation  gyro = rad/sec/hz     #0.008 * 3.14/180   
        
        self.b_a = 0.01 #20*9.8* 1e-3         #m/s^2 bias init value
        self.b_g = 0.01  #0.5 * 3.14/180       #rad/s gyro bias init value # wokring 
        w_ba = [0.05, 0.05]                   ## std devia    tion for bias acceleromteer same for x and y = 
        w_bg = 0.001                           ## std deviation for bias gyro

        self.dt = 0.03 # 1000 hz - freuqncy of IMU node
        self.EKF = EKF(self.P, w_a, w_g, w_ba, w_bg, self.dt) 
        self.init_count = 0 

        
    def IMU_callback(self, msg):
        if not self.GPS_init_flag:
            pass
        
        else:#estimate using filter and publish
            # print("   ")
            # print("velocity before ", self.X[3]) 
            acc_x = msg.linear_acceleration.x
            acc_y = msg.linear_acceleration.y
            omega = msg.angular_velocity.z
            U = np.array([acc_x, acc_y, omega])
            #print("U", U)
            self.X, self.P, dx = self.EKF.predict(self.X, U, self.P)
            #print("state ", self.X[0:2])
            #print("velocity after predict", self.X[3])   

            #publish 
            pose_msg = VDpose()                
            pose_msg.x = self.X[0]
            pose_msg.y = self.X[1]
            pose_msg.psi = self.X[2]
            pose_msg.vf = self.X[3]
            pose_msg.vx_dt = dx[3]
            pose_msg.vy_dt = dx[4]
            self.publisher.publish(pose_msg)


    def GPS_callback(self, msg):
        if not self.GPS_init_flag:
            self.init_count += 1
            if self.init_count <  10:
                pass
            else:
                self.X = np.array([msg.x, msg.y,0.0,0,0,self.b_a,self.b_a,self.b_g])  
                self.x_last = msg.x
                self.y_last = msg.y         
                self.P = np.diag(np.array([msg.covar_x, msg.covar_y, 0, 0,0,0,0,0]))            
                #print("init state ", self.X[0:2])
                self.GPS_init_flag = True
        else:
            
            #vel calculations
            vel_x = (self.X[0] - self.x_last)/ self.dt
            vel_y = (self.X[1] - self.y_last) /self.dt

            #measurement vector 
            z = np.array([msg.x, msg.y, vel_x, vel_y])

            #covarince calculations
            x_covar = msg.covar_x
            y_covar = msg.covar_y

            covar_vx = 2 * x_covar**2 / self.dt**2
            covar_vy = 2 * y_covar**2 / self.dt**2 

            R = np.diag(np.array([x_covar, y_covar, covar_vx, covar_vy]))            
            R[0,1] = 0.02
            R[1,0] = 0.02 
            R[0,2] = 0.02
            R[2,0] = 0.02  

            #print("R", R) 
            self.X, self.P =  self.EKF.update(self.X, self.P, z, R)

            self.x_last = self.X[0]
            self.y_last = self.X[1] 

            #print("velocity after update", self.X[3]) 


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

