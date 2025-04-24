cmake_minimum_required(VERSION 3.21)
project(FreeSLAM_ros VERSION 1.0.0)
# c++ standard 17
set(CMAKE_CXX_STANDARD 17)
# default build release
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()
# useful compile options
add_compile_options(-Wall)
add_compile_options(-Werror=return-type)
option(WITH_LTO "enable -flto optimization" OFF)
if(WITH_LTO)
    add_compile_options(-flto)
endif()
option(WITH_NATIVE "enable -march=native optimization" OFF)
if(WITH_NATIVE)
    add_compile_options(-march=native)
endif()
# find necessary packages
add_subdirectory(../FreeSLAM ${CMAKE_CURRENT_BINARY_DIR}/FreeSLAM)
find_package(catkin REQUIRED COMPONENTS
    roscpp
    rosbag
    std_msgs
    sensor_msgs
    nav_msgs
    geometry_msgs
    tf2_ros
    pcl_conversions
    gps_common
)


# catkin
catkin_package()
## build ros node
# build rosbag_tool
add_library(rosbag_tool OBJECT src/rosbag.cpp)
target_include_directories(rosbag_tool PRIVATE
    include
    ${catkin_INCLUDE_DIRS}
)
# build mapping node
add_executable(mapping_node src/mapping_node.cpp)
add_dependencies(mapping_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_include_directories(mapping_node PRIVATE
    include
    ${catkin_INCLUDE_DIRS}
)
target_link_libraries(mapping_node PRIVATE
    rosbag_tool
    ${catkin_LIBRARIES}
    FreeSLAM::FreeSLAM
    reflcpp::reflcpp
)
# build localization node
add_executable(localization_node src/localization_node.cpp)
add_dependencies(localization_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_include_directories(localization_node PRIVATE
    include
    ${catkin_INCLUDE_DIRS}
)
target_link_libraries(localization_node PRIVATE
    rosbag_tool
    ${catkin_LIBRARIES}
    FreeSLAM::FreeSLAM
)
## install
# install(TARGETS mapping_node
#     RUNTIME_DEPENDENCIES DIRECTORIES ${catkin_LIBRARY_DIRS}
#     ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#     LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#     RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
install(DIRECTORY configs data
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)