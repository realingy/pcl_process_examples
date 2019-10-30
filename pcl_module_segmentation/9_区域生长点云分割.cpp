#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>
#include <iostream>
#include <vector>

int main(int argc, char** argv)
{
	//点云的类型
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	//打开点云
	if (pcl::io::loadPCDFile <pcl::PointXYZ>("table_scene_lms400.pcd", *cloud) == -1)
	{
		std::cout << "Cloud reading failed." << std::endl;
		return -1;
	}

	//设置搜索的方式或者说是结构
	pcl::search::Search<pcl::PointXYZ>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> >(new pcl::search::KdTree<pcl::PointXYZ>);
	
	//求法线
	pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);

	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
	normal_estimator.setSearchMethod(tree);
	normal_estimator.setInputCloud(cloud);
	normal_estimator.setKSearch(50);
	normal_estimator.compute(*normals);

	//直通滤波在Z轴的0到1米之间
	pcl::IndicesPtr indices(new std::vector <int>);
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud(cloud);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(0.0, 1.0);
	pass.filter(*indices);  //聚类对象<点，法线>

	pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
	reg.setMinClusterSize(50);  //最小的聚类的点数
	reg.setMaxClusterSize(1000000);  //最大的
	reg.setSearchMethod(tree);    //搜索方式
	reg.setNumberOfNeighbours(30);    //设置搜索的邻域点的个数
	reg.setInputCloud(cloud);         //输入点  

	// reg.setIndices (indices);
	reg.setInputNormals (normals);     //输入的法线
	reg.setSmoothnessThreshold(3.0 / 180.0 * M_PI);  //设置平滑度
	reg.setCurvatureThreshold(1.0);     //设置曲率的阀值
	std::vector <pcl::PointIndices> clusters;
	reg.extract(clusters); std::cout << "Number of clusters is equal to " << clusters.size() << std::endl;
	std::cout << "First cluster has " << clusters[0].indices.size() << " points." << endl;
	std::cout << "These are the indices of the points of the initial" <<
		std::endl << "cloud that belong to the first cluster:" << std::endl;

	/*
	int counter = 0;
	while (counter < clusters[0].indices.size()) {
		std::cout << clusters[0].indices[counter] << ", ";
		counter++;
		if (counter % 10 == 0)
			std::cout << std::endl;
	}
	std::cout << std::endl;
	*/

	//可视化聚类的结果
	pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud();
	pcl::visualization::CloudViewer viewer("Cluster viewer");
	viewer.showCloud(colored_cloud);

	while (!viewer.wasStopped())
	{
	}
	
	return (0);
}

