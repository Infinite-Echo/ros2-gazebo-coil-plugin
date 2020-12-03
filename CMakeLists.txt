cmake_minimum_required(VERSION 2.8.3)
project(wibotic_gazebo_plugins)

add_compile_options(-std=c++11)

set(LIB_NAME wibotic_coil_plugin)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS roscpp gazebo_ros std_msgs genmsg)
find_package(gazebo REQUIRED)
find_package(Protobuf REQUIRED)

add_message_files(
  DIRECTORY
  wibotic_msgs
  FILES
  CoilPosition.msg
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(DEPENDS roscpp gazebo_ros)

include_directories(
  include
  ${CMAKE_CURRENT_BINARY_DIR}
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)
link_directories(${GAZEBO_LIBRARY_DIRS} ${CMAKE_CURRENT_BINARY_DIR})

add_library(
  ${LIB_NAME} SHARED
  src/WiboticCoilPlugin.cc

)
target_link_libraries(
  ${LIB_NAME}
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)

install(TARGETS ${LIB_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
