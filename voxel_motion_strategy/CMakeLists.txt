cmake_minimum_required(VERSION 3.0.2)
project(voxel_motion_strategy)

set(CMAKE_CXX_STANDARD 17)

# ---- Build type (Debug during development; switch to Release for deployment) ----
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
endif()

if(CMAKE_BUILD_TYPE STREQUAL "Release")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2 -march=native -DNDEBUG")
  message(STATUS "[voxel_motion_strategy] Release build")
else()
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O0 -g -Wall")
  message(STATUS "[voxel_motion_strategy] Debug build")
endif()

# OpenMP and other optimizations deferred to deployment phase.

# --- integral_image library ---
add_library(integral_image
  src/integral_image.cpp
)
target_link_libraries(integral_image
  ${catkin_LIBRARIES}
)

# --- integral_image_test ---
add_executable(integral_image_test src/integral_image_test.cpp)
target_link_libraries(integral_image_test
  integral_image
  ${catkin_LIBRARIES}
)

# --- rectangle_search library ---
add_library(rectangle_search
  src/rectangle_search.cpp
)
target_link_libraries(rectangle_search
  integral_image
  ${catkin_LIBRARIES}
)

# --- rectangle_search_test ---
add_executable(rectangle_search_test src/rectangle_search_test.cpp)
target_link_libraries(rectangle_search_test
  rectangle_search
  integral_image
  erp_projector
  octomap_builder
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
)

# --- rect_viz (score-vs-yaw analysis + best-view depth map) ---
add_executable(rect_viz src/rect_viz.cpp)
target_link_libraries(rect_viz
  rectangle_search
  integral_image
  erp_projector
  octomap_builder
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
  yaml-cpp
)

# --- erp_viz (quick depth map visualization) ---
add_executable(erp_viz src/erp_viz.cpp)
target_link_libraries(erp_viz
  erp_projector
  octomap_builder
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  cyber_msgs
  sensor_msgs
  nav_msgs
)

find_package(PCL 1.8 REQUIRED COMPONENTS common io kdtree search features)
find_package(octomap REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy std_msgs cyber_msgs sensor_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
)

link_directories(${OCTOMAP_LIBRARY_DIRS})

# --- octomap_builder library ---
add_library(octomap_builder
  src/octomap_builder.cpp
)
target_link_libraries(octomap_builder
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
)

# --- voxel_motion_strategy_node ---
add_executable(${PROJECT_NAME}_node src/voxel_motion_strategy_node.cpp)
target_link_libraries(${PROJECT_NAME}_node
  rectangle_search
  integral_image
  erp_projector
  octomap_builder
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
)

# --- octomap_test (normal consistency analysis) ---
add_executable(octomap_test src/octomap_test.cpp)
target_link_libraries(octomap_test
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
)

# --- octomap_builder_test ---
add_executable(octomap_builder_test src/octomap_builder_test.cpp)
target_link_libraries(octomap_builder_test
  octomap_builder
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
)

# --- yaw_constraint_test (header-only, no lib dep) ---
add_executable(yaw_constraint_test src/yaw_constraint_test.cpp)

# --- erp_projector library ---
add_library(erp_projector
  src/erp_projector.cpp
)
target_link_libraries(erp_projector
  octomap_builder
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
)

# --- erp_projector_test ---
add_executable(erp_projector_test src/erp_projector_test.cpp)
target_link_libraries(erp_projector_test
  erp_projector
  octomap_builder
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
)
