cmake_minimum_required(VERSION 3.5)

# 工程信息
project(PCL_MODULES_BASE)

include_directories(${CMAKE_SOURCE_DIR}/..)

add_subdirectory(1_DisplayCloudAsHeight)
add_subdirectory(2_octree)
add_subdirectory(3_kdtree)

