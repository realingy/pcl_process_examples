#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>  //体素滤波相关
#include <iostream>


int main()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsampled (new pcl::PointCloud<pcl::PointXYZ>);

	// ... read, pass in or create a point cloud ...
	if (pcl::io::loadPCDFile<pcl::PointXYZ>("rabbit.pcd", *cloud) == -1)
	{
		PCL_ERROR("Cloudn't read file!");
		return -1;
	}
	cout << "there are " << cloud->points.size() << " points before estimate." << endl;

	//... create a downsampled version of it ...
	// 创建滤波器对象
	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud);
	//sor.setLeafSize(0.01f, 0.01f, 0.01f);// 单位：m
	sor.setLeafSize(0.3f, 0.3f, 0.3f);// 单位：m
	sor.filter(*cloud_downsampled);

	// Create the normal estimation class, and pass the input dataset to it
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	ne.setInputCloud (cloud_downsampled);

	// Pass the original data (before downsampling) as the search surface
	ne.setSearchSurface (cloud);

	// Create an empty kdtree representation, and pass it to the normal estimation object.
	// Its content will be filled inside the object, based on the given surface dataset.
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
	ne.setSearchMethod (tree);

	// Output datasets
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

	// Use all neighbors in a sphere of radius 3cm
	ne.setRadiusSearch (0.03);

	// Compute the features
	ne.compute (*cloud_normals);

	// cloud_normals->points.size () should have the same size as the input cloud_downsampled->points.size ()
	cout << "there are " << cloud_normals->points.size() << " points after estimate." << endl;
}





