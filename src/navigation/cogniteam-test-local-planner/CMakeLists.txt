cmake_minimum_required(VERSION 2.8.3)
project(cogniteam-test-local-planner)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  tf
  #base_local_planner
  costmap_2d
  geometry_msgs
  nav_core
  nav_msgs
  navfn
  pluginlib
  roscpp
  tf2_geometry_msgs
  tf2_ros

)

catkin_package()

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_definitions(-std=c++11)

add_executable(cogniteam_test_local_planner_node 
    src/cogniteam_test_local_planner_node.cpp
  
)

add_dependencies(cogniteam_test_local_planner_node 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(cogniteam_test_local_planner_node
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}

)

