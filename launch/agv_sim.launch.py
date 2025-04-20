from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = FindPackageShare('my_agv_pkg').find('my_agv_pkg')
    urdf_path = pkg_share + '/urdf/agv.urdf.xacro'

    return LaunchDescription([
        # 1) Start Gazebo with ROS factory plugin
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
            output='screen',
        ),

        # 2) Publish robot_description and spawn the AGV
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='rsp',
            parameters=[{
                'robot_description': Command(['xacro ', urdf_path])
            }],
            output='screen'
        ),
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-topic', 'robot_description',
                '-entity', 'agv',
                '-x', '0', '-y', '0', '-z', '0.1'
            ],
            output='screen'
        ),

        # 3) Trajectory selector GUI
        Node(
            package='my_agv_pkg',
            executable='trajectory_selector',
            name='trajectory_selector',
            output='screen'
        ),

        # 4) PID controller
        Node(
            package='my_agv_pkg',
            executable='pid_controller',
            name='pid_controller',
            output='screen'
        ),
    ])
