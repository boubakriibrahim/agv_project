cmake_minimum_required(VERSION 3.5)
project(my_agv_pkg)

find_package(ament_cmake REQUIRED)
# Install the Python package:
ament_python_install_package(${PROJECT_NAME})

ament_package()
