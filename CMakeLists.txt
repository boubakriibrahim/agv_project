cmake_minimum_required(VERSION 3.5)
project(my_agv_pkg)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# This tells ament to install your Python module named my_agv_pkg
ament_python_install_package(${PROJECT_NAME})

# If you have data files (launch, URDF), install them too:
install(DIRECTORY
  launch
  urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
