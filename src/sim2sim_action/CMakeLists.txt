cmake_minimum_required(VERSION 3.8)
project(sim2sim_action)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(action_process_node
  src/action_process_node.cpp
)

ament_target_dependencies(action_process_node
  rclcpp
  std_msgs
)

install(TARGETS action_process_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
