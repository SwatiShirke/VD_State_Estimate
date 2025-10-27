import carla
import pygame
import numpy as np
import math
import weakref
import sys

##functions
def draw_image(surface, image):
    array = np.frombuffer(image.raw_data, dtype=np.uint8)
    array = array.reshape((image.height, image.width, 4))
    array = array[:, :, :3]  # BGRA -> BGR
    array = array[:, :, ::-1]  # BGR -> RGB
    surface.blit(pygame.surfarray.make_surface(array.swapaxes(0, 1)), (0, 0))

def main():
    pygame.init()
    pygame.font.init()
    width, height = 1280, 720
    display = pygame.display.set_mode((width, height))
    pygame.display.set_caption("CARLA Vehicle Camera")

    client = carla.Client('127.0.0.1', 2000)
    client.set_timeout(10.0)
    world = client.get_world()

    blueprint_library = world.get_blueprint_library()
    vehicle_bp = world.get_blueprint_library().find('vehicle.mini.cooper')
    spawn_points = world.get_map().get_spawn_points()
    print("spawn_points[0]", spawn_points[0])
    vehicle_bp.set_attribute('role_name', 'hero')
    vehicle_bp.set_attribute("ros_name", 'ego_vehicle')
    vehicle_transform =  carla.Transform(carla.Location(x=-64.00, y= 24.00, z=0),  # 3 meters behind the vehicle, elevated a bit
    carla.Rotation(pitch=0.0, yaw=0.0, roll=0.0))  # face backward (yaw 180))
    vehicle = world.spawn_actor(vehicle_bp, spawn_points[0])
    


    #GNSS adding 
    # gps_transform = carla.Transform(carla.Location(x=0, y=0, z=0))  
    # gnss_bp = world.get_blueprint_library().find('sensor.other.gnss')    
    # gnss_bp.set_attribute('sensor_tick', '0.01')    
    # gnss_sensor = world.spawn_actor(gnss_bp, gps_transform, attach_to=vehicle)

    #Attach RGB camera sensor to vehicle
    camera_bp = blueprint_library.find('sensor.camera.rgb')
    camera_bp.set_attribute('image_size_x', str(width))
    camera_bp.set_attribute('image_size_y', str(height))
    camera_bp.set_attribute('fov', '110')
    camera_transform = carla.Transform(carla.Location(x=-5.0, z=2.4),  # 3 meters behind the vehicle, elevated a bit
    carla.Rotation(pitch=0.0, yaw=0.0, roll=0.0))  # face backward (yaw 180))
    camera = world.spawn_actor(camera_bp, camera_transform, attach_to=vehicle)

    # Use a pygame surface to hold the image
    camera_surface = pygame.Surface((width, height))

    # Use a weakref to avoid circular references
    def camera_callback(image, surface):
        draw_image(surface, image)

    #camera.listen(lambda image: camera_callback(image, camera_surface))

    clock = pygame.time.Clock()

    try:
        while True:
            clock.tick(100)

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    return
        

            #vehicle.apply_control(carla.VehicleControl(throttle=0.7, brake=0.0))

            

            #Draw the camera image to pygame window
            # display.blit(camera_surface, (0, 0))
            pygame.display.flip()

    finally:
        # camera.stop()
        # camera.destroy()
        #gnss_sensor.destroy()
        #vehicle.destroy()
        pygame.quit()

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("Exiting by user")
        pygame.quit()
        sys.exit()
