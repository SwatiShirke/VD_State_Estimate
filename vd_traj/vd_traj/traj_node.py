import rclpy
from rclpy.node import Node
from vd_msgs.msg import VDstate, VDtraj, VDpose
from vd_msgs.srv import VDGenTraj
from std_msgs.msg import String
from vd_traj.traj_mod import traj
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import numpy as np
N = 10
Tf = 0.01

def is_terminate(last_pose, z_state, threshold):
    #np.linalg.norm([z_state.x, z_state.y], last_pose[1:3]) < threshold
    print( np.sqrt((z_state.x - last_pose[1])**2 + (z_state.y - last_pose[2])**2 ) )
    if z_state.s >= last_pose[0] and np.sqrt((z_state.x - last_pose[1])**2 + (z_state.y - last_pose[2])**2 ) < threshold:
        #print("trajectory finished!")
        is_terminate_flag = True
    else:
        is_terminate_flag = False

    return is_terminate_flag

class TrajPublisher(Node):

    def __init__(self):
        super().__init__('traj_publisher')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,  # Best effort, similar to TCP no delay
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1)  # Equivalent to queue_size=1)

        self.traj_pub = self.create_publisher(VDtraj, 'vd_traj', qos_profile)
        self.subscription = self.create_subscription(VDstate,'vd_state',self.state_cb, qos_profile)
        self.subscription  # prevent unused variable warning

        
        self.srv = self.create_service(VDGenTraj, 'vd_gen_traj', self.vd_gen_traj_cb)
        self.traj = traj()
        self.vel = 0
        self.traj_generated = False
        self.threshold = 1
        

    def state_cb(self, z_state):
        #print("I am here")
        if self.traj_generated == True: 
            #print("inside the state cb")          
            if is_terminate(self.traj.last_pose, z_state, self.threshold):

                pose_list = []            
                traj_message = VDtraj()                               
                pose_message = VDpose()   
                pose_message.s = self.traj.last_pose[0]
                pose_message.x = self.traj.last_pose[1]
                pose_message.y = self.traj.last_pose[2]
                pose_message.psi = self.traj.last_pose[3]
                pose_message.vf = self.traj.last_pose[4]
                #pose_message.pose =[s, self.traj.x_interpld(s), self.traj.y_interpld(s), self.traj.yaw_interpld(s), self.traj.vel_interpld(s)]
                pose_list.append(pose_message)
                traj_message.poses = pose_list
                self.traj_pub.publish(traj_message)
                self.traj_generated = False

            else:    
                pose_list = []            
                traj_message = VDtraj()
                s0 = z_state.s 
                for j in range(N):  
                    pose_message = VDpose()      
                    s = s0 + (self.vel * Tf) * (j+1)/ N
                    pose_message.s = s
                    #print(self.traj.x_interpld(s))
                    pose_message.x = float(self.traj.x_interpld(s))
                    pose_message.y = float(self.traj.y_interpld(s))
                    pose_message.psi = float(self.traj.yaw_interpld(s))
                    pose_message.vf = float(self.traj.vel_interpld(s))
                    #pose_message.pose =[s, self.traj.x_interpld(s), self.traj.y_interpld(s), self.traj.yaw_interpld(s), self.traj.vel_interpld(s)]
                    pose_list.append(pose_message)
                traj_message.poses = pose_list
                self.traj_pub.publish(traj_message)
        

    def vd_gen_traj_cb(self, request, response):
        traj_type = request.trajtype
        self.traj.set_traj(traj_type)
        response.success = True
        response.message = "generated traj successfully!"        
        self.traj_generated = True
        return response

def main(args=None):
    rclpy.init(args=args)

    traj_publisher = TrajPublisher()

    rclpy.spin(traj_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    traj_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()