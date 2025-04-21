cmake_minimum_required(VERSION 3.5)
project(my_agv_pkg)

# Standard CMake/ament setup
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install the Python module
ament_python_install_package(${PROJECT_NAME})

# Install the "resource" file so ament_index knows about your package
install(
  FILES
    resource/${PROJECT_NAME}
  DESTINATION
    share/ament_index/resource_index/packages
)

# Install runtime files (launch, URDF)
install(
  DIRECTORY
    launch
    urdf
  DESTINATION
    share/${PROJECT_NAME}
)

ament_package()
