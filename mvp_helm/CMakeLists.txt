cmake_minimum_required(VERSION 3.5)
project(mvp_helm)

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
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(behavior_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(robot_localization REQUIRED)
find_package(yaml-cpp REQUIRED)


####################################################################
#####                   setup dependencies                     #####
####################################################################
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/usr/share/cmake/geographiclib/")
find_package(GeographicLib REQUIRED)

set(library_name helm)
set(executable_name mvp_helm)

set(dependencies
  rclcpp
  rclcpp_components
  std_msgs
  mvp_msgs
  nav_msgs
  tf2
  tf2_geometry_msgs
  geometry_msgs
  behavior_interface
  robot_localization
  pluginlib
)

####################################################################
#####                      setup compiling                     #####
####################################################################

include_directories(
  include
)

# Library
add_library(${library_name} SHARED
  src/mvp_helm/sm.cpp
  src/mvp_helm/helm.cpp
  src/mvp_helm/parser.cpp
  src/mvp_helm/node_wrapper.cpp
  src/mvp_helm/behavior_container.cpp
)
ament_target_dependencies(${library_name}
  ${dependencies}
)
target_link_libraries( 
  ${library_name}
  ${YAML_CPP_LIBRARIES}
  ${GeographicLib_LIBRARIES}
)

# Executable
add_executable(${executable_name}
  src/main.cpp
)
target_link_libraries(${executable_name} 
  ${library_name}
  ${YAML_CPP_LIBRARIES}
  ${GeographicLib_LIBRARIES}
)
ament_target_dependencies(${executable_name}
  ${dependencies}
)

rclcpp_components_register_nodes(${library_name} "helm::Helm")

####################################################################
#####                       setup install                      #####
####################################################################

# Install
install(TARGETS ${library_name} ${executable_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch 
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config 
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY mvp_mission_config 
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})
ament_package()
