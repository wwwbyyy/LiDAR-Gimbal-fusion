cmake_minimum_required(VERSION 3.0.2)
project(gimbal_disturbance_test)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  std_msgs
  cyber_msgs
)

find_package(yaml-cpp REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp roslib std_msgs cyber_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/disturbance_test_node.cpp)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)
