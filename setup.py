from setuptools import setup

import glob
import os

package_name = 'ros2_aruco'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name,'ros2_aruco.markers'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'urdf_model/Meshes'), glob.glob('urdf_model/Meshes/*.dae')),
        (os.path.join('share', package_name, 'urdf_model'),        glob.glob('urdf_model/model.urdf')),
        (os.path.join('share', package_name, 'launch'),            glob.glob('launch/myrobot_launch.py')),
        (os.path.join('share', package_name, 'rviz'),              glob.glob('rviz/config.rviz')),
        (os.path.join('share', package_name, 'ros2_aruco'),        glob.glob('ros2_aruco/panel1.txt')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yasmin',
    maintainer_email='yasmin@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aruco_node2 = ros2_aruco.aruco_node2:main',
            'robot = ros2_aruco.robot:main',
            'panel = ros2_aruco.panel:main',
            'orientation = ros2_aruco.orientation:main'
        ],
    },
)
