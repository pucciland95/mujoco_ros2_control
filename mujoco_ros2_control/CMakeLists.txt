cmake_minimum_required(VERSION 3.5)
project(mujoco_ros2_control)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(controller_manager REQUIRED)
find_package(joint_limits REQUIRED)
find_package(urdf REQUIRED)
find_package(glfw3 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(control_toolbox REQUIRED)

set(THIS_PACKAGE_DEPENDS
  ament_cmake
  rclcpp
  std_msgs
  hardware_interface
  pluginlib
  controller_manager
  joint_limits
  urdf
  glfw3
  control_toolbox
)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

find_package(mujoco QUIET)
if(mujoco_FOUND)
  message(STATUS "Mujoco build from source has been found")
  set(MUJOCO_LIB mujoco::mujoco)
  set(MUJOCO_INCLUDE_DIR ${MUJOCO_INCLUDE_DIR})
  set(MUJOCO_BIN_DIR ${MUJOCO_BIN_DIR})
elseif(DEFINED ENV{MUJOCO_DIR})
  message(STATUS "Mujoco build from source has not been found. Attempting to find the binary in $ENV{MUJOCO_DIR} instead.")
  find_library(MUJOCO_LIB mujoco HINTS $ENV{MUJOCO_DIR}/lib)
  if(NOT MUJOCO_LIB)
    message(FATAL_ERROR "Failed to find binary in $ENV{MUJOCO_DIR}")
  endif()
  set(MUJOCO_INCLUDE_DIR $ENV{MUJOCO_DIR}/include)
  set(MUJOCO_BIN_DIR $ENV{MUJOCO_DIR}/bin)
else()
  message(FATAL_ERROR "Failed to find mujoco with find_package.
  Either build and install mujoco from source or set the MUJOCO_DIR environment variable to tell CMake where to find the binary install. ")
endif()


# Building the ros2_control hardware interface as a plugin
set(HW_IF_INCLUDE_DEPENDS
  pluginlib
  joint_limits
  rcpputils
  hardware_interface
  urdf
)

add_library(mujoco_system_plugins SHARED src/mujoco_system.cpp)
ament_target_dependencies(mujoco_system_plugins ${HW_IF_INCLUDE_DEPENDS})
target_link_libraries(mujoco_system_plugins ${MUJOCO_LIB})
target_include_directories(mujoco_system_plugins
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${MUJOCO_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIR})

install(TARGETS mujoco_system_plugins
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
pluginlib_export_plugin_description_file(mujoco_ros2_control mujoco_system_plugins.xml)

# add_executable(mujoco_ros2_control src/mujoco_ros2_control_node.cpp src/mujoco_ros2_control.cpp src/mujoco_ros2_control_plugin.cpp)
add_library(mujoco_ros2_control SHARED src/mujoco_ros2_control_plugin.cpp)
ament_target_dependencies(mujoco_ros2_control ${THIS_PACKAGE_DEPENDS})
target_link_libraries(mujoco_ros2_control ${MUJOCO_LIB} glfw)
target_include_directories(mujoco_ros2_control
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${MUJOCO_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIR})

install(TARGETS
  mujoco_ros2_control
  DESTINATION lib/${PROJECT_NAME})

# Exporting mujoco ros2 controller as a plugin
add_library(mujoco_plugins SHARED src/Plugin.cpp)
target_include_directories(mujoco_plugins PRIVATE ${MUJOCO_INCLUDE_DIR})
target_link_libraries(mujoco_plugins PUBLIC mujoco_ros2_control)

install(
  TARGETS mujoco_plugins
  EXPORT "${TARGETS_EXPORT_NAME}"
  LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}"
  ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}"
  RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}")

# add_custom_command(
#    TARGET mujoco_system
#    POST_BUILD
#    COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:mujoco_system>
#             ${MUJOCO_BIN_DIR}/mujoco_plugin)

# add_custom_command(
#   TARGET mujoco_ros2_control
#   POST_BUILD
#   COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:mujoco_ros2_control>
#           ${MUJOCO_BIN_DIR}/mujoco_plugin)

add_custom_command(
  TARGET mujoco_plugins
  POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:mujoco_plugins>
          ${MUJOCO_BIN_DIR}/mujoco_plugin)

ament_package()
