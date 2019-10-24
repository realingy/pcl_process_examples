// 点特征分布图估计
#include <pcl/point_types.h>
#include <pcl/features/pfh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>  //体素滤波相关
#include <iostream>

int main()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal> ());

	//... read, pass in or create a point cloud with normals ...
	//... (note: you can create a single PointCloud<PointNormal> if you want) ...
	if (pcl::io::loadPCDFile<pcl::PointXYZ>("rabbit.pcd", *cloud) == -1)
	{
		PCL_ERROR("Cloudn't read file!");
		return -1;
	}
	cout << "there are " << cloud->points.size() << " points before pfh estimate." << endl;

	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	ne.setInputCloud(cloud);
	ne.setSearchMethod(tree);
	ne.setRadiusSearch(0.03);
	ne.compute(*normals);

	// Create the PFH estimation class, and pass the input dataset+normals to it
	// 创建PFH估计对象pfh，并将输入点云数据集cloud和法线normals传递给它
	pcl::PFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::PFHSignature125> pfh;
	pfh.setInputCloud (cloud);
	pfh.setInputNormals (normals); 

	// alternatively, if cloud is of tpe PointNormal, do pfh.setInputNormals (cloud);
	// 如果点云是类型为PointNormal,则执行pfh.setInputNormals (cloud);

	// Create an empty kdtree representation, and pass it to the PFH estimation object.
	// 创建一个空的kd树表示法，并把它传递给PFH估计对象。
	// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
	// 基于已给的输入数据集，建立kdtree
	// pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
	// pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr tree (new pcl::KdTreeFLANN<pcl::PointXYZ> ()); -- older call for PCL 1.5-
	pfh.setSearchMethod (tree);

	pcl::PointCloud<pcl::PFHSignature125>::Ptr pfhs (new pcl::PointCloud<pcl::PFHSignature125> ());

	// Use all neighbors in a sphere of radius 5cm
	// IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
	// 计算pfh特征值,半径设为5cm（此半径要大于表面法线估计所使用的近邻搜索半径）
	pfh.setRadiusSearch (0.05);

	// Compute the features
	pfh.compute (*pfhs);

	// pfhs->points.size()应该与input cloud->points.size()有相同的大小，即每个点都有一个pfh特征向量
	cout << "there are " << pfhs->points.size() << " points after pfh estimate." << endl;
}

