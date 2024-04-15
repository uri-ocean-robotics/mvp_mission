cmake_minimum_required(VERSION 3.5)
project(bhv_template)

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
)

####################################################################
#####                      setup compiling                     #####
####################################################################

include_directories(
  include
)

# plugins
add_library(mvp2_bhv_template SHARED
  src/bhv_template/bhv_template.cpp
)

ament_target_dependencies(mvp2_bhv_template
  ${dependencies}
)

pluginlib_export_plugin_description_file(behavior_interface behavior_plugin.xml)

####################################################################
#####                       setup install                      #####
####################################################################


install(TARGETS 
  mvp2_bhv_template
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
  mvp2_bhv_template
)
ament_export_dependencies(${dependencies})
ament_package()
