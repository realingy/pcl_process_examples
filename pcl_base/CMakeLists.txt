cmake_minimum_required(VERSION 3.5)

# 工程信息
project(MY_PCL_PROJECT_CH2)

include_directories(${CMAKE_SOURCE_DIR}/..)

add_subdirectory(0_pcd_write)
add_subdirectory(0_pcl_read)
add_subdirectory(1_pcd2txt)
add_subdirectory(2_txt2pcd)
add_subdirectory(3_mypcd2txt)
add_subdirectory(4_pcdwr)
add_subdirectory(5_kd-tree)
add_subdirectory(5_pcd_concatenating)

