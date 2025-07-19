
import carla
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
import math
from rclpy.clock import Clock
import numpy as np
from std_msgs.msg import Float32


def main(VEHICLE_ROLE_NAME):            
    client = carla.Client('localhost', 2000)              
    client.set_timeout(10.0)
    world = client.get_world()
    ##add gnss 
   
    # Get a vehicle blueprint
    blueprint_library = world.get_blueprint_library()
    vehicle_bp = blueprint_library.filter('vehicle.mini.cooper')[0]  # or any other model

    # Define the spawn transform at origin (0, 0, 0)
    spawn_transform = carla.Transform(
        location=carla.Location(x=0.0, y=0.0, z=0.1),  # z slightly above ground
        rotation=carla.Rotation(yaw=0.0)
    )

    # Spawn the vehicle
    vehicle = world.try_spawn_actor(vehicle_bp, spawn_transform)
    gnss_bp = world.get_blueprint_library().find('sensor.other.gnss')
    gnss_sensor = world.spawn_actor(gnss_bp, spawn_transform)
    
    print("I am here")
    gnss_sensor.listen(gnss_callback)

    while True:
        pass 
    






def gnss_callback(data):
        print(f"Origin GPS -> Latitude: {data.latitude}, Longitude: {data.longitude}, Altitude: {data.altitude}")
        # Store it and stop listening after one reading
        #gnss_sensor_test.stop()



if __name__ == "__main__":
    VEHICLE_ROLE_NAME = "hero"
    main(VEHICLE_ROLE_NAME)
