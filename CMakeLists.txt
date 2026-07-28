cmake_minimum_required(VERSION 3.8)
project(mavlink_nav_bridge)

set(CMAKE_CXX_STANDARD 20)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Threads REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RCChannels.msg"
  "msg/VehicleCommand.msg"
  "msg/DroneStatus.msg"
  DEPENDENCIES std_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

include_directories(include)

# gcs_shell
add_executable(gcs_shell src/gcs_shell.cpp)
ament_target_dependencies(gcs_shell rclcpp)
target_link_libraries(gcs_shell "${cpp_typesupport_target}" Threads::Threads)

# drone_sim
add_executable(drone_sim src/drone_sim.cpp)
ament_target_dependencies(drone_sim rclcpp tf2 tf2_ros geometry_msgs nav_msgs visualization_msgs)
target_link_libraries(drone_sim "${cpp_typesupport_target}")

install(TARGETS gcs_shell drone_sim
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config/
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_drone_physics test/test_drone_physics.cpp)
  target_include_directories(test_drone_physics PRIVATE include)
endif()

ament_package()
