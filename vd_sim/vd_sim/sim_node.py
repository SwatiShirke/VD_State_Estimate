import rclpy
from rclpy.node import Node
from vd_msgs.msg import VDstate, VDtraj, VDpose, VDControlCMD
import numpy as np
from vd_sim.model_utils import *
import scipy.integrate
from ament_index_python.packages import get_package_share_directory
import os
from nav_msgs.msg import Odometry, Path
from rclpy.clock import Clock
from geometry_msgs.msg import PoseStamped, Wrench, TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


package_share_directory = get_package_share_directory('vd_sim')
config_directory = os.path.join(package_share_directory, 'config')
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy 

class SimPublisher(Node):

    def __init__(self, params):
        super().__init__('sim_publisher')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,  # Best effort, similar to TCP no delay
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1)  # Equivalent to queue_size=1)

        self.state_publisher = self.create_publisher(VDstate, 'vd_state', qos_profile)
        self.cmd_subscriber = self.create_subscription(VDControlCMD,'vd_control_cmd',self.mpc_cmd_cb, qos_profile)
        self.pid_subscriber = self.create_subscription(VDControlCMD,'pid_control_cmd',self.pid_cmd_cb, qos_profile)
        self.vd_path_publisher = self.create_publisher(Path, 'vd_path' ,100)
        self.vd_path = Path()
        self.worldframe = "simulator"
        self.clock = Clock() 
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)

        #create VD model
        self.z_state = np.zeros((11), dtype= float)
        self.torque_in = 0
        self.steering_angle = 0 
        self.params = params
        # self.Kp = 300
        # self.Ki = 0.0
        # self.Kd = 0
        self.t_solve = (0,0.01)
        # self.cumm_error = 0
        # self.last_error = 0
    
    def timer_callback(self):     
        state_msg = VDstate()
        u = [self.torque_in, self.steering_angle]
        print("u for sim")
        print(u)
        # print(self.z_state)
        sol = scipy.integrate.solve_ivp(self.VD_model_closed_loop,self.t_solve, self.z_state, method= 'LSODA', args=[u])
        #print(sol.y[-5])
        #print(sol.y[:,-1].shape)
        self.z_state  = sol.y[:,-1]
        state_msg.vx = self.z_state[0]
        state_msg.vy = self.z_state[1]
        state_msg.r = self.z_state[2]
        state_msg.x = self.z_state[3]
        state_msg.y = self.z_state[4]
        state_msg.psi = self.z_state[5]
        state_msg.w_fl = self.z_state[6]
        state_msg.w_fr = self.z_state[7]
        state_msg.w_rl = self.z_state[8]
        state_msg.w_rr= self.z_state[9]
        state_msg.s = self.z_state[10]
        self.state_publisher.publish(state_msg)
        print("current_state")
        print([state_msg.x, state_msg.y, state_msg.psi])

        #add to payload path
        current_time = self.clock.now().to_msg()
        self.vd_path.header.stamp = current_time
        self.vd_path.header.frame_id = self.worldframe 
        vd_pose_stamped = PoseStamped()
        vd_pose_stamped.header.stamp = current_time
        vd_pose_stamped.header.frame_id = self.worldframe
        
        #Populate the position
        vd_pose_stamped.pose.position.x = state_msg.x
        vd_pose_stamped.pose.position.y = state_msg.y
        vd_pose_stamped.pose.position.z = 0.0

        #Populate the orientation (quaternion)
        vd_pose_stamped.pose.orientation.x = 0.0
        vd_pose_stamped.pose.orientation.y = 0.0
        vd_pose_stamped.pose.orientation.z = 0.0
        vd_pose_stamped.pose.orientation.w = 1.0

        self.vd_path.poses.append(vd_pose_stamped)        
        self.vd_path_publisher.publish(self.vd_path)
        self.publish_static_transform()


    def publish_static_transform(self):
        # Define the static transform from 'map' to 'simulator'
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'map'          # Global frame
        transform.child_frame_id = 'simulator'     # Frame of your odometry
        transform.transform.translation.x = 0.0
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = 0.0
        transform.transform.rotation.w = 1.0       # Identity quaternion, no rotation

        # Broadcast the static transform
        self.static_tf_broadcaster.sendTransform(transform)

 
    def VD_model_closed_loop(self,t, z, u):        
        torque_in = u[0]
        delta = u[1]        
        #states Vx, Vy, r, x, y, psi, w1, w1, w3, w4
        Vx = z[0]
        Vy = z[1]
        r  = z[2]
        x  = z[3]
        y  = z[4]
        psi= z[5]

        #torque_in = self.apply_PID(ref_vel, Vx)        
        ##calculate intial accel values for load transfer calculations 
        ax = Vy * r
        ay = -Vx * r

        #input torue to each wheel 
        tau_in = cal_wheel_torque_in(self.params, torque_in)        
        tau_in_fl = tau_in[0]
        tau_in_fr = tau_in[1]
        tau_in_rl = tau_in[2]
        tau_in_rr = tau_in[3]

        #calculate forces
        t_forces = tire_forces(self.params, z, delta, ax, ay, torque_in)
        Fx_sum, Fy_f, Fy_r, Fy_del = vehicle_forces(self.params, z, t_forces, delta)

        Vx_dt = Vy * r + (Fx_sum / self.params.m)
        Vy_dt = -Vx * r + ((Fy_f+ Fy_r) / self.params.m)

        t_forces = tire_forces(self.params, z, delta, Vx_dt, Vy_dt,torque_in)
        Fx_sum, Fy_f, Fy_r, Fy_del = vehicle_forces(self.params, z, t_forces, delta)

        #state dt equations
        Vx_dt = Vy * r + (Fx_sum / self.params.m)
        Vy_dt = -Vx * r + ((Fy_f+ Fy_r) / self.params.m)
        r_dt =  (self.params.a * Fy_f - self.params.b *Fy_r) / self.params.Izz_v

        x_dt = Vx * np.cos(psi) - Vy * np.sin(psi)
        y_dt = Vx * np.sin(psi) + Vy * np.cos(psi)
        psi_dt  = r

        #%set tire forces 
        Fx_fl = t_forces[0]
        Fx_fr = t_forces[1]
        Fx_rl = t_forces[2]
        Fx_rr = t_forces[3] 

        #%wheel velocities 
        w_fl_dt = (tau_in_fl - Fx_fl * self.params.R_f) / self.params.Iyy_w_f
        w_fr_dt = (tau_in_fr - Fx_fr * self.params.R_f) / self.params.Iyy_w_f
        w_rl_dt = (tau_in_rl - Fx_rl * self.params.R_r) / self.params.Iyy_w_r
        w_rr_dt = (tau_in_rr - Fx_rr * self.params.R_r) / self.params.Iyy_w_r
        s_dt = Vx
        dz = [Vx_dt, Vy_dt, r_dt, x_dt, y_dt, psi_dt, w_fl_dt, w_fr_dt, w_rl_dt, w_rr_dt, s_dt]
        #print(dz)
        return dz 


    def apply_PID(self, ref_vel, Vx):
        error = (ref_vel - Vx )                
        torque = error * self.Kp + self.cumm_error * self.Ki + (self.last_error - error)* self.Kd
        self.cumm_error += error 
        self.last_error = error
        return torque


    def mpc_cmd_cb(self, msg):
        #self.Vf = msg.velocity
        self.steering_angle = msg.steering_angle

    def pid_cmd_cb(self, msg):
        self.torque_in = msg.velocity


def main(args=None):

    rclpy.init(args=args)
    filename = "params.yaml"
    param_file = os.path.join(config_directory, filename)
    #rclpy.node.get_logger().info(f'Node initialized with file name: {param_file}')
    print("##########################")
    print(param_file)
    params_object = load_params_from_yaml(param_file)
    sim_publisher = SimPublisher(params_object)
    rclpy.spin(sim_publisher)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    sim_publisher.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':    
    main()