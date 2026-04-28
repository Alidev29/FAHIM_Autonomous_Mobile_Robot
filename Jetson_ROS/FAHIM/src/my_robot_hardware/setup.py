from setuptools import find_packages, setup

package_name = 'my_robot_hardware'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robot26',
    maintainer_email='aa.k4132@gmail.com',
    description='FAHIM low-level hardware bridge: Arduino motor/encoder + MPU6050 IMU',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'arduino_bridge = my_robot_hardware.arduino_bridge:main',
            'mpu6050_node   = my_robot_hardware.imu_node:main',
        ],
    },
)
