#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Obtain the path to the URDF file
    urdf_path = os.path.join(get_package_share_directory('your_agv_description_pkg'), 'urdf', 'agv.urdf')
    
    # Launch Gazebo with ROS2 plugins enabled and load the AGV model via ROS2 factory
    return LaunchDescription([
        ExecuteProcess(
            cmd=[
                'gazebo', '--verbose', '-s', 'libgazebo_ros_init.so',
                '-s', 'libgazebo_ros_factory.so'
            ],
            output='screen'
        )
        # You can add additional nodes here, e.g. RViz2, if desired.
    ])

if __name__ == '__main__':
    ld = generate_launch_description()
