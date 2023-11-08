cmake_minimum_required(VERSION 3.5)
project(wibotic_gazebo_plugins)

# set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/../gazebo_plugins")
# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/../gazebo_plugins")

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

set(LIB_NAME wibotic_coil_plugin)

# INTERFACES

set(msg_files
  "src/wibotic_interfaces/msg/CoilPosition.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
)



# PLUGIN

include_directories(include)

add_library(${LIB_NAME} SHARED src/WiboticCoilPlugin.cpp)
ament_target_dependencies(${LIB_NAME}
  "gazebo_dev"
  "gazebo_ros"
  "std_msgs"
  "rclcpp"
)

install(TARGETS ${LIB_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

target_link_libraries(${LIB_NAME} "${cpp_typesupport_target}")
ament_export_dependencies(rosidl_default_runtime)
ament_package()