cmake_minimum_required(VERSION 3.8)
project(recon_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# draw_node — web-driven OccupancyGrid for end-to-end DB testing
add_executable(draw_node src/draw_node.cpp)
ament_target_dependencies(draw_node
  rclcpp
  nav_msgs
  std_msgs
)
target_include_directories(draw_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS
  draw_node
  DESTINATION lib/${PROJECT_NAME}
)

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/include)
  install(DIRECTORY include/
    DESTINATION include
  )
endif()

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_draw_node
    ../../tests/test_draw_node.cpp
  )
  ament_target_dependencies(test_draw_node
    rclcpp nav_msgs std_msgs)
endif()

ament_package()
