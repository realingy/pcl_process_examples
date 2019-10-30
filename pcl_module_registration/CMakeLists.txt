# cmake_minimum_required(VERSION 2.8)

# set( CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}" ${CMAKE_MODULE_PATH} )
# project( Surf_keypoints_detection_and_pointcloud_registration )

# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ./bin)

# set( CMAKE_CXX_FLAGS "-std=c++11" )

# find_package( OpenCV 3 REQUIRED )
# include_directories( ${OpenCV_INCLUDE_DIRS} )


# find_package(PCL 1.8 REQUIRED)
# include_directories( ${PCL_INCLUDE_DIRS} )
# add_definitions( ${PCL_DEFINITIONS} )
# add_definitions( -DPCL_NO_PRECOMPILE )
# link_directories( ${PCL_LIBRARY_DIRS} )


cmake_minimum_required(VERSION 3.5)

project(pointcloud_registration_demos)

set( CMAKE_CXX_FLAGS "-std=c++11" )
set( CMAKE_CXX_FLAGS "/EHsc ${CMAKE_CXX_FLAGS}" )

set(CMAKE_CONFIGRATION_TYPES "Release" CACHE STRING "" FORCE)

# find_package(PCL 1.9 REQUIRED COMPONENTS common io)
find_package(PCL 1.9 REQUIRED)

# include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

# 库目录
# link_directories(${PCL_LIBRARY_DIRS})
link_directories(${PCL_ROOT}/3rdParty/OpenNI2/Lib)
link_directories(${PCL_ROOT}/D:/PCL 1.9.1/3rdParty/VTK/lib)
link_directories(${PCL_ROOT}/D:/PCL 1.9.1/3rdParty/Qhull/lib)
link_directories(${PCL_ROOT}/D:/PCL 1.9.1/3rdParty/FLANN/lib)
link_directories(${PCL_ROOT}/D:/PCL 1.9.1/3rdParty/Boost/lib)
link_directories(${PCL_ROOT}/D:/PCL 1.9.1/lib)

# 包含目录
include_directories(${PCL_ROOT}/3rdParty/OpenNI2/Include)
include_directories(${PCL_ROOT}/3rdParty/VTK/include/vtk-8.1)
include_directories(${PCL_ROOT}/3rdParty/Qhull/include)
include_directories(${PCL_ROOT}/3rdParty/FLANN/include)
include_directories(${PCL_ROOT}/3rdParty/Eigen/eigen3)
include_directories(${PCL_ROOT}/3rdParty/Boost/include/boost-1_68)
include_directories(${PCL_ROOT}/include/pcl-1.9)

# opencv库查询
set(OpenCV_DIR $ENV{OPENCV_DIR})
find_package(OpenCV REQUIRED)

# 包含目录
include_directories(${OpenCV_DIR}/install/include)
include_directories(${OpenCV_DIR}/install/include/opencv)
include_directories(${OpenCV_DIR}/install/include/opencv2)

# 查找源文件
# aux_source_directory(. DIR_SRCS)

add_executable( 1_Surf_detection_with_Ransac 1_基于SURF的点云特征点检测.cpp )
target_link_libraries( 1_Surf_detection_with_Ransac ${OpenCV_LIBS} )

add_executable( 2_Surf_rgbd_pointcloud_Registration 2_基于SURF的点云配准.cpp )
# target_link_libraries( 2_Surf_rgbd_pointcloud_Registration ${OpenCV_LIBS} )
target_link_libraries( 2_Surf_rgbd_pointcloud_Registration ${OpenCV_LIBS} ${PCL_LIBRARIES})

add_executable( 3_Registration_IPC 3_基于IPC的点云配准.cpp )
target_link_libraries( 3_Registration_IPC ${OpenCV_LIBS} ${PCL_LIBRARIES})

add_executable( 4_Registration_normal_distributions 4_基于正态分布变换的点云配准.cpp )
target_link_libraries( 4_Registration_normal_distributions ${OpenCV_LIBS} ${PCL_LIBRARIES})

add_executable( 5_Registratin_IPC_visualizaion 5_基于IPC的点对齐.cpp )
target_link_libraries( 5_Registratin_IPC_visualizaion ${OpenCV_LIBS} ${PCL_LIBRARIES})

# set(EXE_NAME 01_DisplayCloudAsHeight)
# add_executable(${EXE_NAME} ${DIR_SRCS})

# target_link_libraries(${EXE_NAME} ${PCL_LIBRARIES})
# target_link_libraries(pcd_write_test ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES})