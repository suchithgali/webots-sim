cmake_minimum_required(VERSION 3.8)
project(my_package)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(autopilot_node src/autopilot_node.cpp)
ament_target_dependencies(autopilot_node rclcpp geometry_msgs std_msgs)
install(TARGETS autopilot_node RUNTIME DESTINATION lib/${PROJECT_NAME})

# Install the Python mavic_driver module so WebotsController can find it
ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY launch resource worlds DESTINATION share/${PROJECT_NAME})

ament_package()