cmake_minimum_required(VERSION 3.5)
project(bhv_path_following)

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
find_package(visualization_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(robot_localization REQUIRED)
find_package(yaml-cpp REQUIRED)

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
  visualization_msgs
  std_srvs
  tf2_ros
  tf2
  tf2_eigen
  robot_localization
  yaml-cpp
)

####################################################################
#####                      setup compiling                     #####
####################################################################

include_directories(
  include
  ${YAML_CPP_INCLUDE_DIRS}  # Include yaml-cpp headers
)

# plugins
add_library(mvp_bhv_path_following SHARED
  src/bhv_path_following/bhv_path_following.cpp
  src/bhv_path_following/bhv_path_following_misc.cpp
)


ament_target_dependencies(mvp_bhv_path_following
  ${dependencies}
)

pluginlib_export_plugin_description_file(behavior_interface behavior_plugin.xml)

####################################################################
#####                       setup install                      #####
####################################################################


install(TARGETS 
  mvp_bhv_path_following
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
  mvp_bhv_path_following
)
ament_export_dependencies(${dependencies})
ament_package()
