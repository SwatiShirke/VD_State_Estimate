import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
import math
from rclpy.clock import Clock
import numpy as np
from std_msgs.msg import Float32
import carla
from sensor_msgs.msg import Imu


class IMU(Node):
    def __init__(self, VEHICLE_ROLE_NAME):
        super().__init__('IMU_processing')

        ##set all bias covar, random walk, white noise covar
        ##connect with Carla server
        self.client = carla.Client('localhost', 2000)              
        self.client.set_timeout(10.0)
        self.world = self.client.get_world() 

        ##find vehicle
        self.role_name = VEHICLE_ROLE_NAME
        self.vehicle = self.get_vehicle_by_role_name()
        if self.vehicle == None:
            raise RuntimeError(f"Vehicle with ID {self.role_name} not found!")       
         
        
        ## add imu
        imu_bp = self.world.get_blueprint_library().find('sensor.other.imu')
        imu_bp.set_attribute('sensor_tick', '0.001')  # 100 Hz
        imu_bp.set_attribute('noise_accel_stddev_x', '0.1')
        imu_bp.set_attribute('noise_accel_stddev_y', '0.1')
        imu_bp.set_attribute('noise_gyro_stddev_z', '0.1')
        imu_transform = carla.Transform(carla.Location(x=0.0, y=0.0, z=0.0))  # Center of vehicle
        self.imu_sensor = self.world.spawn_actor(imu_bp, imu_transform, attach_to=self.vehicle)
        
        
        #publisher
        self.publisher = self.create_publisher(Imu, 'IMU_data', 10)
        self.x = 0.0
        self.y = 0.0 
        self.filtering_type =  "median"
        self.imu_sensor.listen(self.imu_callback) 
        self.buffer_size = 10
        self.IMU_data = np.zeros((self.buffer_size, 3))  # IMU buffer 
        self.buffer_index = 0
        

    def get_vehicle_by_role_name(self):
            vehicles = self.world.get_actors().filter('vehicle.*')
            for vehicle in vehicles:
                if vehicle.attributes.get('role_name') == self.role_name:
                    print(f"Found vehicle with role_name: {self.role_name}, ID: {vehicle.id}")
                    return vehicle     
      

    def imu_callback(self, data):
        # print("IMU:")
        # print(f"  Accelerometer: {data.accelerometer}")
        # print(f"  Gyroscope:     {data.gyroscope}")
        # print(f"  Compass:       {data.compass}") 
        msg = Imu()
        self.IMU_data[self.buffer_index] = [data.accelerometer.x, data.accelerometer.y, data.gyroscope.z]
        self.buffer_index += 1 
        self.buffer_index = self.buffer_index % self.buffer_size

        
        if self.filtering_type == "mean":                    #moving average 
            processed_data = np.mean(self.IMU_data, axis = 0)  
        elif self.filtering_type == "median":                #median filtering
            processed_data = np.median(self.IMU_data, axis = 0)   #no filtering 
        else :
            processed_data = [data.accelerometer.x, data.accelerometer.y, data.gyroscope.z]
        
        msg.linear_acceleration.x = processed_data[0]
        msg.linear_acceleration.y = processed_data[1]
        msg.angular_velocity.z = processed_data[2]         
        self.publisher.publish(msg) 



def main():
    # Initialize ROS
    rclpy.init()

    VEHICLE_ROLE_NAME = "hero"
    # Create the publisher node
    node = IMU(VEHICLE_ROLE_NAME)

    try:
        rclpy.spin(node)
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()





