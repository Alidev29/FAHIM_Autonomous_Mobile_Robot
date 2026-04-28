from setuptools import find_packages, setup

package_name = 'my_robot_vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/yolo.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robot26',
    maintainer_email='aa.k4132@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'yolo_node = my_robot_vision.yolo_node:main',
        ],
    },
)
