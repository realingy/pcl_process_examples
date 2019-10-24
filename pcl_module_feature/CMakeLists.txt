cmake_minimum_required(VERSION 3.5)

# 工程信息
project(PCL_PROJECT_MODULES)

include_directories(${CMAKE_SOURCE_DIR}/..)

add_subdirectory(0_Features)
add_subdirectory(1_FeaturesSubset)
add_subdirectory(2_FeaturesDownsample)
add_subdirectory(3_NormalEstimateIntegralImages)
add_subdirectory(4_PFHEstimate)
add_subdirectory(5_FPFHEstimate)

