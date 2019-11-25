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

project(pcl_module_visualization)

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

add_executable( 1_Visualizer 可视化工具类.cpp )
target_link_libraries( 1_Visualizer ${OpenCV_LIBS} ${PCL_LIBRARIES})

add_executable( 2_MultiViewerPort 多点云可视化窗口.cpp )
target_link_libraries( 2_MultiViewerPort ${OpenCV_LIBS} ${PCL_LIBRARIES})

add_executable( 0_PoindCloudViewer 点云查看.cpp )
target_link_libraries( 0_PoindCloudViewer ${OpenCV_LIBS} ${PCL_LIBRARIES})