cmake_minimum_required(VERSION 3.5)
project(bhv_teleoperation)

####################################################################
#####                       setup compiler                     #####
####################################################################

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

####################################################################
#####                       setup packages                     #####
####################################################################

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(mvp_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(behavior_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)

####################################################################
#####                   setup dependencies                     #####
####################################################################

set(dependencies
  rclcpp
  rclcpp_components
  std_msgs
  mvp_msgs
  nav_msgs
  geometry_msgs
  behavior_interface
  pluginlib
  sensor_msgs
  std_srvs
)

####################################################################
#####                      setup compiling                     #####
####################################################################

include_directories(
  include
)

# plugins
add_library(mvp_bhv_teleoperation SHARED
  src/bhv_teleoperation/bhv_teleoperation.cpp
)

ament_target_dependencies(mvp_bhv_teleoperation
  ${dependencies}
)

pluginlib_export_plugin_description_file(behavior_interface behavior_plugin.xml)

####################################################################
#####                       setup install                      #####
####################################################################


install(TARGETS 
  mvp_bhv_teleoperation
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

install(FILES behavior_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY src/
  DESTINATION share/${PROJECT_NAME}/src/
)

ament_export_include_directories(include)
ament_export_libraries(
  mvp_bhv_teleoperation
)
ament_export_dependencies(${dependencies})
ament_package()
