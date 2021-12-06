cmake_minimum_required(VERSION 3.20)
project(pc_utils)

set(CMAKE_CXX_STANDARD 17)

find_package(PCL 1.8 COMPONENTS
        common
        filters
        segmentation
        visualization
        surface
        REQUIRED QUIET)
find_package(yaml-cpp REQUIRED)


add_library(pc_utils STATIC
        src/box_extract.cpp
        src/filter/cloud_filter.cpp
        src/common.cpp)

target_include_directories(
        pc_utils
        PUBLIC
        include
        ${YAML_CPP_INCLUDE_DIR}
        ${PCL_INCLUDE_DIRS}
        src
)


target_link_libraries(pc_utils
        PUBLIC
        ${YAML_CPP_LIBRARIES}
        ${PCL_LIBRARIES}
        )

add_subdirectory(test)