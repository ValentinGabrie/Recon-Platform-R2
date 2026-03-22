cmake_minimum_required(VERSION 3.8)
project(roomba_control)

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
find_package(std_srvs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate custom service
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/BtSimInject.srv"
  DEPENDENCIES std_msgs
)

# joy_control_node
add_executable(joy_control_node src/joy_control_node.cpp)
ament_target_dependencies(joy_control_node
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
)
target_include_directories(joy_control_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# bt_sim_node
add_executable(bt_sim_node src/bt_sim_node.cpp)
ament_target_dependencies(bt_sim_node
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  std_srvs
)
target_include_directories(bt_sim_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# draw_node
add_executable(draw_node src/draw_node.cpp)
ament_target_dependencies(draw_node
  rclcpp
  sensor_msgs
  nav_msgs
  std_msgs
)
target_include_directories(draw_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link generated service interfaces to bt_sim_node
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(bt_sim_node "${cpp_typesupport_target}")

# Install targets
install(TARGETS
  joy_control_node
  bt_sim_node
  draw_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

# Testing
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_joy_control_node
    ../../tests/test_joy_control_node.cpp
  )
  ament_target_dependencies(test_joy_control_node
    rclcpp sensor_msgs geometry_msgs std_msgs)

  ament_add_gtest(test_bt_sim_node
    ../../tests/test_bt_sim_node.cpp
  )
  ament_target_dependencies(test_bt_sim_node
    rclcpp sensor_msgs geometry_msgs std_msgs)

  ament_add_gtest(test_draw_node
    ../../tests/test_draw_node.cpp
  )
  ament_target_dependencies(test_draw_node
    rclcpp sensor_msgs nav_msgs std_msgs)
endif()

ament_package()
