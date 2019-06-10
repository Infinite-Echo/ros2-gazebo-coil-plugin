cmake_minimum_required(VERSION 2.8.3)
project(wibotic_gazebo_plugins)

add_compile_options(-std=c++11)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS roscpp gazebo_ros)
find_package(gazebo REQUIRED)
find_package(Protobuf REQUIRED)

catkin_package(DEPENDS roscpp gazebo_ros)

include_directories(
  include
  ${CMAKE_CURRENT_BINARY_DIR}
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)
link_directories(${GAZEBO_LIBRARY_DIRS} ${CMAKE_CURRENT_BINARY_DIR})

add_library(wibotic_coil_plugin SHARED src/WiboticCoilPlugin.cc)
target_link_libraries(
  wibotic_coil_plugin
  wibotic_gazebo_plugins_msgs
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)
add_dependencies(wibotic_coil_plugin wibotic_gazebo_plugins_msgs)
install(TARGETS wibotic_coil_plugin
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

set(PROTOBUF_IMPORT_DIRS)
foreach(ITR ${GAZEBO_INCLUDE_DIRS})
  if(ITR MATCHES ".*gazebo-[0-9.]+$")
    set(PROTOBUF_IMPORT_DIRS "${ITR}/gazebo/msgs/proto")
  endif()
endforeach()

set (msgs
  msgs/coil_position.proto
  ${PROTOBUF_IMPORT_DIRS}/time.proto
  ${PROTOBUF_IMPORT_DIRS}/header.proto
)

PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ${msgs})

add_library(wibotic_gazebo_plugins_msgs SHARED ${PROTO_SRCS})
target_link_libraries(wibotic_gazebo_plugins_msgs ${PROTOBUF_LIBRARY})

install(TARGETS wibotic_gazebo_plugins_msgs
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
