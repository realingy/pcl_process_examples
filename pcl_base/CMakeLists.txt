cmake_minimum_required(VERSION 3.5)

# 工程信息
project(pcl_base)

include_directories(${CMAKE_SOURCE_DIR}/..)

add_subdirectory(0_pcd_write)
add_subdirectory(0_pcl_read)
add_subdirectory(1_pcd2txt)
add_subdirectory(2_txt2pcd)
add_subdirectory(3_mypcd2txt)
add_subdirectory(4_pcdwr)
add_subdirectory(5_kd-tree)
add_subdirectory(6_pcd_concatenating)
add_subdirectory(7_las2pcd)
add_subdirectory(8_pcd_ply)
add_subdirectory(9_kinect2)
add_subdirectory(10_linescanner)
# add_subdirectory(11_cloud_save)
add_subdirectory(12_pcd_openni)

