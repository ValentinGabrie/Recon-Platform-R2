cmake_minimum_required(VERSION 3.8)
project(roomba_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# motor_controller
add_executable(motor_controller src/motor_controller.cpp)
ament_target_dependencies(motor_controller
  rclcpp
  geometry_msgs
  std_msgs
)
target_link_libraries(motor_controller pigpio)
target_include_directories(motor_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# sim_sensor_node — simulated ultrasonic + LIDAR (no hardware deps)
add_executable(sim_sensor_node src/sim_sensor_node.cpp)
ament_target_dependencies(sim_sensor_node
  rclcpp
  sensor_msgs
  nav_msgs
  std_msgs
)
target_include_directories(sim_sensor_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# sim_motor_node — simulated diff-drive with collision (no hardware deps)
add_executable(sim_motor_node src/sim_motor_node.cpp)
ament_target_dependencies(sim_motor_node
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
  tf2_ros
)
target_include_directories(sim_motor_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Install targets
install(TARGETS
  motor_controller
  sim_sensor_node
  sim_motor_node
  DESTINATION lib/${PROJECT_NAME}
)

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/include)
  install(DIRECTORY include/
    DESTINATION include
  )
endif()

# Testing
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_motor_controller
    ../../tests/test_motor_controller.cpp
  )
  ament_target_dependencies(test_motor_controller rclcpp geometry_msgs std_msgs)

  ament_add_gtest(test_sim_sensor_node
    ../../tests/test_sim_sensor_node.cpp
  )
  ament_target_dependencies(test_sim_sensor_node rclcpp sensor_msgs nav_msgs std_msgs)

  ament_add_gtest(test_sim_motor_node
    ../../tests/test_sim_motor_node.cpp
  )
  ament_target_dependencies(test_sim_motor_node rclcpp geometry_msgs nav_msgs std_msgs tf2_ros)
endif()

ament_package()
