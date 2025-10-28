import carla
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
import math
from rclpy.clock import Clock
import numpy as np
from std_msgs.msg import Float32
from rclpy.qos import QoSProfile, QoSHistoryPolicy, ReliabilityPolicy, DurabilityPolicy
from vd_msgs.msg import GPSpose

#This file holds code for GPS data -
##make a ros node to pusblish data at 100 hz    

class GPS(Node):
    def __init__(self, VEHICLE_ROLE_NAME, init_location):
        super().__init__('GPS_processing')
        qos_profile = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE)

        ##carla setup
        # self.clock = Clock() #wall clock
        # self.sim_clock = self.get_clock() #sim clock        
        self.client = carla.Client('localhost', 2000)              
        self.client.set_timeout(10.0)
        self.world = self.client.get_world()

        ##add gnss 
        self.role_name = VEHICLE_ROLE_NAME
        self.vehicle = self.get_vehicle_by_role_name()
        if self.vehicle == None:
            raise RuntimeError(f"Vehicle with ID {self.role_name} not found!")
        vehicle_transform = self.vehicle.get_transform() 
        gps_transform = carla.Transform(carla.Location(x=0, y=0, z=0))  
        print("gps_transform", gps_transform)
        self.gnss_bp = self.world.get_blueprint_library().find('sensor.other.gnss')
        self.dt = 0.01 

        ## GPS accuracy is 1m - based on that 1/111km = latitude std dev
        ## init location is location of reference frame 
        self.longitude_ref = init_location[0]
        self.latitude_ref = init_location[1]
        self.R_earth = 6371000 #m                

        # bias settings 
        self.bias_longitude = 0.0 #3.6261e-06    # 30% of 1m = 0.3m - from this = 0.3 / 111km * cos(lat) = 
        self.bias_latitude  = 0.0 #3.6261e-06    # 30% of 1m = 0.3m - from this = 0.3 / 111km = 
        self.long_bias_random = 0.0         # keeping it 0 as carla does handle model bias random walk  
        self.lat_bias_random = 0.0    

        ##white noise
        self.w_long = 9e-6  #white noise std deviation in degree.    1 degree = 111km , so how much degree for 1m distance  #9e-6
        self.w_lat = 9e-6 
         
        
        self.gnss_bp.set_attribute('sensor_tick', '0.1')
        self.gnss_bp.set_attribute('noise_lat_bias', str(self.bias_latitude))         # mean bias for latitude
        self.gnss_bp.set_attribute('noise_lat_stddev', str(self.w_lat))     # standard deviation for latitude
        self.gnss_bp.set_attribute('noise_lon_bias', str(self.bias_longitude))
        self.gnss_bp.set_attribute('noise_lon_stddev', str(self.w_long)) 
        self.gnss_sensor = self.world.spawn_actor(self.gnss_bp, gps_transform, attach_to=self.vehicle)

        #publisher
        self.publisher = self.create_publisher(GPSpose, 'GPS_location', qos_profile) 
        self.x = 0.0
        self.y = 0.0 

        ##spawn dummy vehicle and get its location         
        self.gnss_sensor.listen(self.gnss_callback)

       
        

    def get_vehicle_by_role_name(self):
            vehicles = self.world.get_actors().filter('vehicle.*')
            for vehicle in vehicles:
                if vehicle.attributes.get('role_name') == self.role_name:
                    print(f"Found vehicle with role_name: {self.role_name}, ID: {vehicle.id}")
                    return vehicle     


    
    def update_GPS_noise(self, latitude ):
       
        ##bias model
        self.bias_long_dt = np.random.normal(0, self.long_bias_random * np.cos(latitude))
        self.bias_lat_dt = np.random.normal(0, self.lat_bias_random)
        self.bias_longitude = self.bias_longitude + self.dt * self.bias_long_dt
        self.bias_latitude = self.bias_latitude + self.dt * self.bias_lat_dt


    def gnss_callback( self, data):
        # print("here after gps call")
        # print("long", data.longitude)
        # print("here...............")
        # print("latt", data.latitude) 

        self.update_GPS_noise(data.latitude )
        self.longitude = data.longitude - self.bias_longitude 
        self.latitude = data.latitude - self.bias_latitude 
        self.transform = data.transform

        del_longitude = math.radians( self.longitude - self.longitude_ref)
        del_latitude = math.radians(self.latitude - self.latitude_ref)
        self.x = self.R_earth * del_longitude * math.cos(math.radians(self.latitude_ref))
        self.y = -self.R_earth * del_latitude
        
        pose = GPSpose()
        pose.x = self.x
        pose.y = self.y

        lat_rad = math.radians(self.latitude)  # convert latitude once

        # Standard deviations (1σ) in degrees
        sigma_lon_deg = self.w_long  
        sigma_lat_deg = self.w_lat

        # Convert degree errors to meters
        sigma_east_m = self.R_earth * math.cos(lat_rad) * math.radians(sigma_lon_deg)
        sigma_north_m = - self.R_earth * math.radians(sigma_lat_deg)

        # Covariances (squared standard deviations)
        x_covar = sigma_east_m**2
        y_covar = sigma_north_m**2 

        # self.w_long = self.w_long_base * np.cos(math.radians(self.latitude))
        # x_covar = (self.R_earth * math.radians(self.w_long)* math.cos(math.radians(self.latitude)) )**2
        # y_covar = (- self.R_earth * math.radians(self.w_lat))**2
        pose.covar_x = x_covar
        pose.covar_y = y_covar
        self.publisher.publish(pose)

       
        

def main():
    # Initialize ROS
    rclpy.init()

    VEHICLE_ROLE_NAME = "hero"
    init_location = [0.0, 0.0]    # reference frame's location's longitude and latitude 

    # Create the publisher node
    node = GPS(VEHICLE_ROLE_NAME, init_location)

    try:
        rclpy.spin(node)
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()