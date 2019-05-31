cmake_minimum_required(VERSION 2.8.3)
project(wibotic_gazebo_plugins)

# Load catkin and all dependencies required for this package
find_package(
  catkin REQUIRED
  COMPONENTS roscpp gazebo_ros
)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)

include_directories(
  include
  ${CMAKE_CURRENT_BINARY_DIR}
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)
link_directories(${GAZEBO_LIBRARY_DIRS} ${CMAKE_CURRENT_BINARY_DIR})
add_subdirectory(msgs)

add_library(wibotic_coil_plugin src/WiboticCoilPlugin.cc)
target_link_libraries(wibotic_coil_plugin
  wibotic_gazebo_plugins_msgs
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)
add_dependencies(wibotic_coil_plugin wibotic_gazebo_plugins_msgs)

catkin_package(DEPENDS roscpp gazebo_ros)
