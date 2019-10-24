#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/normal_refinement.h>
#include <pcl/visualization/cloud_viewer.h>  //点云可视化相关定义
	
typedef pcl::PointXYZ PointT;
typedef pcl::Normal NormalT;

int main()
{
	/*
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	if (pcl::io::loadPCDFile<pcl::PointXYZ>("rabbit.pcd", *cloud) == -1)
	{
		PCL_ERROR("Cloudn't read file!");
		return -1;
	}
	cout << "there are " << cloud->points.size() << " points before estimate." << endl;

	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	ne.setInputCloud(cloud);
	ne.setSearchMethod(tree);
	ne.setRadiusSearch(0.03);
	ne.compute(*cloud_normals);

	// 法向量重定义
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_refined(new pcl::PointCloud<pcl::Normal>);
	//pcl::NormalRefinement<pcl::Normal> nr(k_indices, k_sqr_distances);
	pcl::NormalRefinement<pcl::Normal> nr;
	nr.setInputCloud(cloud_normals);
	nr.filter(*cloud_normals_refined);

	pcl::visualization::CloudViewer viewer("Cloud Viewer"); //创建viewer对象
	//viewer.showCloud(cloud);
	//viewer.showCloud(cloud_filtered);
	*/

	/*
	// Input point cloud
	//pcl::PointCloud<PointT> cloud;
	pcl::PointCloud<pcl::PointXYZ> cloud;

	// Fill cloud...
	if (pcl::io::loadPCDFile<pcl::PointXYZ>("rabbit.pcd", cloud) == -1)
	{
		PCL_ERROR("Cloudn't read file!");
		return -1;
	}
	cout << "there are " << cloud.points.size() << " points before estimate." << endl;

	// Estimated and refined normals
	pcl::PointCloud<NormalT> normals;
	pcl::PointCloud<NormalT> normals_refined;

	// Search parameters
	const int k = 5;
	std::vector<std::vector<int> > k_indices;
	std::vector<std::vector<float> > k_sqr_distances;

	// Run search
	pcl::search::KdTree<pcl::PointXYZRGB> search;
	search.setInputCloud(cloud.makeShared());
	search.nearestKSearch(cloud, std::vector<int>(), k, k_indices, k_sqr_distances);

	// Use search results for normal estimation
	pcl::NormalEstimation<PointT, NormalT> ne;
	for (unsigned int i = 0; i < cloud.size(); ++i)
	{
		NormalT normal;
		ne.computePointNormal(cloud, k_indices[i]
			normal.normal_x, normal.normal_y, normal.normal_z, normal.curvature);
		pcl::flipNormalTowardsViewpoint(cloud[i], cloud.sensor_origin_[0], cloud.sensor_origin_[1], cloud.sensor_origin_[2],
			normal.normal_x, normal.normal_y, normal.normal_z);
		normals.push_back(normal);
	}

	// Run refinement using search results
	pcl::NormalRefinement<NormalT> nr(k_indices, k_sqr_distances);
	nr.setInputCloud(normals.makeShared());
	nr.filter(normals_refined);
	*/

	
	return (0);
}


