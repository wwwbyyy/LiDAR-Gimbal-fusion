cmake_minimum_required(VERSION 3.0.2)
project(voxel_motion_strategy)

set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  cyber_msgs
  sensor_msgs
)

find_package(PCL 1.8 REQUIRED COMPONENTS common io kdtree search features)
find_package(octomap REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy std_msgs cyber_msgs sensor_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
)

link_directories(${OCTOMAP_LIBRARY_DIRS})

# --- dummy node (placeholder) ---
add_executable(${PROJECT_NAME}_node src/voxel_motion_strategy_node.cpp)
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

# --- octomap_test ---
add_executable(octomap_test src/octomap_test.cpp)
target_link_libraries(octomap_test
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
)
