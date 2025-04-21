from setuptools import setup

package_name = 'my_agv_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
      ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
      ('share/' + package_name, ['package.xml']),
      ('share/' + package_name + '/launch', ['launch/agv_sim.launch.py']),
      ('share/' + package_name + '/urdf', ['urdf/agv.urdf.xacro']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='AGV sim with 2D GUI + PID in Gazebo',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'trajectory_selector = my_agv_pkg.trajectory_selector:main',
            'pid_controller = my_agv_pkg.pid_controller:main',
        ],
    },
)
