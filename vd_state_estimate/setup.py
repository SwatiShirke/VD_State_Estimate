from setuptools import find_packages, setup

package_name = 'vd_state_estimate'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
           "gps_node = vd_state_estimate.GPS_processing:main",
           "IMU_node = vd_state_estimate.IMU_processing:main"
        ],
    },
)
