cmake_minimum_required(VERSION 3.8)
project(sim2sim_platform)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(OpenGL_GL_PREFERENCE GLVND)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenGL QUIET)

set(MUJOCO_ROOT "" CACHE PATH "Root directory of MuJoCo install")
set(GLFW_ROOT "" CACHE PATH "Root directory of GLFW install")
set(_MUJOCO_HINTS)
if(MUJOCO_ROOT)
  list(APPEND _MUJOCO_HINTS ${MUJOCO_ROOT})
endif()
if(DEFINED ENV{MUJOCO_DIR})
  list(APPEND _MUJOCO_HINTS $ENV{MUJOCO_DIR})
endif()
if(DEFINED ENV{MUJOCO_ROOT})
  list(APPEND _MUJOCO_HINTS $ENV{MUJOCO_ROOT})
endif()
list(APPEND _MUJOCO_HINTS /usr /usr/local)

set(_GLFW_HINTS)
if(GLFW_ROOT)
  list(APPEND _GLFW_HINTS ${GLFW_ROOT})
endif()
if(DEFINED ENV{GLFW_ROOT})
  list(APPEND _GLFW_HINTS $ENV{GLFW_ROOT})
endif()
if(DEFINED ENV{GLFW_DIR})
  list(APPEND _GLFW_HINTS $ENV{GLFW_DIR})
endif()
list(APPEND _GLFW_HINTS /usr /usr/local)

find_path(MUJOCO_INCLUDE_DIR
  NAMES mujoco/mujoco.h
  HINTS ${_MUJOCO_HINTS}
  PATH_SUFFIXES include
)
find_library(MUJOCO_LIBRARY
  NAMES mujoco libmujoco
  HINTS ${_MUJOCO_HINTS}
  PATH_SUFFIXES lib lib64 bin
)

find_path(GLFW_INCLUDE_DIR
  NAMES GLFW/glfw3.h
  HINTS ${_GLFW_HINTS}
  PATH_SUFFIXES include
)
find_library(GLFW_LIBRARY
  NAMES glfw glfw3
  HINTS ${_GLFW_HINTS}
  PATH_SUFFIXES lib lib64
)

if(MUJOCO_INCLUDE_DIR AND MUJOCO_LIBRARY)
  add_executable(mujoco_node
    src/main.cpp
    src/mujoco_node.cpp
  )

  target_include_directories(mujoco_node PRIVATE
    include
    ${MUJOCO_INCLUDE_DIR}
  )

  if(GLFW_INCLUDE_DIR AND GLFW_LIBRARY AND OpenGL_FOUND)
    target_compile_definitions(mujoco_node PRIVATE SIM2SIM_HAS_GLFW=1)
    target_include_directories(mujoco_node PRIVATE ${GLFW_INCLUDE_DIR})
    target_link_libraries(mujoco_node ${MUJOCO_LIBRARY} ${GLFW_LIBRARY} OpenGL::GL)
    message(STATUS "GLFW/OpenGL found. MuJoCo viewer support enabled.")
  else()
    target_compile_definitions(mujoco_node PRIVATE SIM2SIM_HAS_GLFW=0)
    target_link_libraries(mujoco_node ${MUJOCO_LIBRARY})
    message(WARNING "GLFW/OpenGL not found. Building mujoco_node without viewer support.")
  endif()

  ament_target_dependencies(mujoco_node
    rclcpp
    sensor_msgs
    std_msgs
  )

  install(TARGETS mujoco_node
    DESTINATION lib/${PROJECT_NAME}
  )
else()
  message(WARNING "MuJoCo headers/library not found. Set MUJOCO_ROOT or MUJOCO_DIR. 'mujoco_node' target will be skipped.")
endif()

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
