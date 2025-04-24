cmake_minimum_required(VERSION 3.21)
project(FreeSLAM VERSION 1.0.0)
# c++ standard 17
set(CMAKE_CXX_STANDARD 17)
# default build release
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()
# useful compile options
add_compile_options(-fPIC)
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

# find necessary libraries

find_package(tsl-robin-map REQUIRED)
# find_package(reflcpp REQUIRED)

include(cmake/CPM.cmake)
# set(CPM_USE_LOCAL_PACKAGES ON)
# CPMAddPackage("gh:Tessil/robin-map@1.2.1")
CPMAddPackage("gh:xinyang-go/reflcpp#main")

set(TBB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../thirdparty/oneapi-tbb-2021.10.0/lib/cmake/tbb)
find_package(TBB REQUIRED)
find_package(yaml-cpp 0.5.2 REQUIRED)
find_package(Eigen3 3.3 REQUIRED)
find_package(PCL 1.8 REQUIRED)
find_package(GTSAM 4.0.0 REQUIRED)
# build library
file(GLOB_RECURSE project_src "src/*.cpp")
add_library(${PROJECT_NAME} STATIC ${project_src})
add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})
target_include_directories(${PROJECT_NAME} PUBLIC
    include
    ${YAML_CPP_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
    ${GTSAM_INCLUDE_DIR}
)
target_link_libraries(${PROJECT_NAME} PUBLIC
    ${YAML_CPP_LIBRARIES}
    tsl::robin_map
    reflcpp::reflcpp
    ${PCL_LIBRARIES}
    gtsam
    TBB::tbb
    TBB::tbbmalloc
)
