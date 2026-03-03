cmake_minimum_required(VERSION 3.8)
project(motion_tracking_controller)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

find_path(ONNXRUNTIME_INCLUDE_DIR
  NAMES onnxruntime_cxx_api.h
  HINTS $ENV{ONNXRUNTIME_ROOT}
  PATH_SUFFIXES include include/onnxruntime onnxruntime
)
find_library(ONNXRUNTIME_LIBRARY
  NAMES onnxruntime
  HINTS $ENV{ONNXRUNTIME_ROOT}
  PATH_SUFFIXES lib lib64
)

add_library(sim2sim_core
  src/mujoco_state_subscriber.cpp
  src/observation_builder.cpp
  src/rl_onnx_policy.cpp
  src/action_codec.cpp
  src/sim2sim_controller_node.cpp
)

target_include_directories(sim2sim_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(sim2sim_core Eigen3::Eigen)
if(ONNXRUNTIME_INCLUDE_DIR AND ONNXRUNTIME_LIBRARY)
  target_include_directories(sim2sim_core PRIVATE ${ONNXRUNTIME_INCLUDE_DIR})
  target_link_libraries(sim2sim_core ${ONNXRUNTIME_LIBRARY})
  target_compile_definitions(sim2sim_core PRIVATE SIM2SIM_USE_ONNXRUNTIME)
  message(STATUS "ONNX Runtime enabled: ${ONNXRUNTIME_LIBRARY}")
else()
  message(WARNING "ONNX Runtime not found. sim2sim_node will compile, but policy inference will throw at runtime.")
endif()
ament_target_dependencies(sim2sim_core rclcpp sensor_msgs std_msgs)

add_executable(sim2sim_node src/main.cpp)
target_link_libraries(sim2sim_node sim2sim_core)
ament_target_dependencies(sim2sim_node rclcpp sensor_msgs std_msgs)

install(TARGETS
  sim2sim_core
  sim2sim_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
