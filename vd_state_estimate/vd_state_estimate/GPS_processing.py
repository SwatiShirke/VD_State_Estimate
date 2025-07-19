import carla
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
import math
from rclpy.clock import Clock
import numpy as np
from std_msgs.msg import Float32

#This file holds code for GPS data -
##make a ros node to pusblish data at 100 hz    

class GPS(Node):
    def __init__(self, VEHICLE_ROLE_NAME):
        super().__init__('GPS_processing')

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
        self.gnss_bp = self.world.get_blueprint_library().find('sensor.other.gnss')
        self.gnss_sensor = self.world.spawn_actor(self.gnss_bp, gps_transform, attach_to=self.vehicle)

        #publisher
        self.publisher = self.create_publisher(Pose2D, 'GPS_location', 10)
        self.x = 0.0
        self.y = 0.0 

        ##spawn dummy vehicle and get its location 
        self.longitude_init = 0
        self.latitude_init = 0
        self.R_earth = 6371000 #m
        self.previous_pose = [self.longitude_init, self.latitude_init]        
        self.gnss_sensor.listen(self.gnss_callback)
        

    def get_vehicle_by_role_name(self):
            vehicles = self.world.get_actors().filter('vehicle.*')
            for vehicle in vehicles:
                if vehicle.attributes.get('role_name') == self.role_name:
                    print(f"Found vehicle with role_name: {self.role_name}, ID: {vehicle.id}")
                    return vehicle     

    def gnss_callback( self, data):
        print("here after gps call")
        print("long", data.longitude)
        print("here...............")
        print("latt", data.latitude) 
        self.longitude = data.longitude
        self.latitude = data.latitude
        self.transform = data.transform

        del_longitude = math.radians( self.longitude - self.longitude_init)
        del_latitude = math.radians(self.latitude - self.latitude_init)


        self.x = self.R_earth * del_longitude * math.cos(math.radians(self.latitude_init))
        self.y = -self.R_earth * del_latitude
        

        # pose = Pose2D()
        # pose.x = self.x
        # pose.y = self.y
        # pose.theta = 0.0
        # self.publisher.publish(pose)

        # print(pose.x)
        # print(pose.y)
        # print("loc", self.transform)

        #self.gnss_sensor.stop()
        #return data.longitude, data.lattitude

        

def main():
    # Initialize ROS
    rclpy.init()

    VEHICLE_ROLE_NAME = "hero"

    # Create the publisher node
    node = GPS(VEHICLE_ROLE_NAME)

    try:
        rclpy.spin(node)
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()