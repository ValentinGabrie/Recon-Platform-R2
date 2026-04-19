cmake_minimum_required(VERSION 3.8)
project(roomba_navigation)

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

# recon_node
add_executable(recon_node src/recon_node.cpp)
ament_target_dependencies(recon_node
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  std_msgs
)
target_include_directories(recon_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# sim_goal_follower
add_executable(sim_goal_follower src/sim_goal_follower.cpp)
ament_target_dependencies(sim_goal_follower
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  std_msgs
)
target_include_directories(sim_goal_follower PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Install targets
install(TARGETS
  recon_node
  sim_goal_follower
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

  ament_add_gtest(test_recon_node
    ../../tests/test_recon_node.cpp
  )
  ament_target_dependencies(test_recon_node
    rclcpp sensor_msgs geometry_msgs nav_msgs std_msgs)

  ament_add_gtest(test_sim_goal_follower
    ../../tests/test_sim_goal_follower.cpp
  )
  ament_target_dependencies(test_sim_goal_follower
    rclcpp geometry_msgs nav_msgs)
endif()

ament_package()
