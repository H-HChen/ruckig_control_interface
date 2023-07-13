cmake_minimum_required(VERSION 2.8.3)
project(ruckig_control_interface)

if(${CMAKE_VERSION} VERSION_LESS "3.8.0")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
else()
	set(CMAKE_CXX_STANDARD 17)
endif()

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  geometry_msgs
  pluginlib
  roscpp
  sensor_msgs
  std_msgs
  tf
  moveit_core
  moveit_ros_planning_interface

  # tm_description
  tm_msgs
  tm_driver
)
find_package(ruckig REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS 
    geometry_msgs
    pluginlib
    roscpp
    sensor_msgs
    std_msgs
    tf
    tm_driver 
)


include_directories(
  include ${catkin_INCLUDE_DIRS}
  include include/
)

add_executable(velocity_controller src/controller.cpp)
add_dependencies(velocity_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(velocity_controller
  ruckig::ruckig
  ${catkin_LIBRARIES}
  ${EIGEN3_INCLUDE_DIRS}
)
