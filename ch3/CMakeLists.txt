cmake_minimum_required(VERSION 3.5)

# 工程信息
project(MY_PCL_PROJECT_CH3)

include_directories(${CMAKE_SOURCE_DIR}/..)

add_subdirectory(0_BilateralFilters)
add_subdirectory(1_DirectFilter)
add_subdirectory(2_VoxelGrid)

