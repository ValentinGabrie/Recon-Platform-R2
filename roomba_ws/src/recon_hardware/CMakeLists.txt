cmake_minimum_required(VERSION 3.8)
project(recon_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++17)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Python module (esp32_uart_bridge + framing parser)
ament_python_install_package(${PROJECT_NAME})

# sim_sensor_node — simulated LIDAR source for SLAM bench testing
add_executable(sim_sensor_node src/sim_sensor_node.cpp)
ament_target_dependencies(sim_sensor_node
  rclcpp
  sensor_msgs
  nav_msgs
)
target_include_directories(sim_sensor_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS
  sim_sensor_node
  DESTINATION lib/${PROJECT_NAME}
)

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/include)
  install(DIRECTORY include/
    DESTINATION include
  )
endif()

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_sim_sensor_node
    ../../tests/test_sim_sensor_node.cpp
  )
  ament_target_dependencies(test_sim_sensor_node rclcpp sensor_msgs nav_msgs)
endif()

ament_package()
