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
find_package(std_msgs REQUIRED)

# esp32_sensor_node
add_executable(esp32_sensor_node src/esp32_sensor_node.cpp)
ament_target_dependencies(esp32_sensor_node
  rclcpp
  sensor_msgs
  std_msgs
)
target_include_directories(esp32_sensor_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

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

# Install targets
install(TARGETS
  esp32_sensor_node
  motor_controller
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

# Testing
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_esp32_sensor_node
    ../../tests/test_esp32_sensor_node.cpp
  )
  ament_target_dependencies(test_esp32_sensor_node rclcpp sensor_msgs std_msgs)

  ament_add_gtest(test_motor_controller
    ../../tests/test_motor_controller.cpp
  )
  ament_target_dependencies(test_motor_controller rclcpp geometry_msgs std_msgs)
endif()

ament_package()
