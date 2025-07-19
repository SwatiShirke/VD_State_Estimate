from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'vd_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='swati',
    maintainer_email='swatishirke.shirke88@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "sim_node = vd_sim.sim_node:main",
            "carla_pub_node = vd_sim.carla_pub_node:main",
            "ros_bag_node = vd_sim.data_saver_node:main"
        ],
    },
)
