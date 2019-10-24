cmake_minimum_required(VERSION 3.5)

# 工程信息
project(PCL_PROJECT_FILTER)

include_directories(${CMAKE_SOURCE_DIR}/..)

# add_subdirectory(0_BilateralFilters)
add_subdirectory(1_PassThrough)
add_subdirectory(2_VoxelGrid)
add_subdirectory(3_StatisticalOutlierRemoval)
add_subdirectory(4_ProjectInliers)



