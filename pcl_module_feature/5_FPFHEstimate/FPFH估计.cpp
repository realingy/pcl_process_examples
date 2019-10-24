// 点特征分布图(PFH)估计
#include <pcl/point_types.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>  //体素滤波相关
#include <pcl/visualization/histogram_visualizer.h> //直方图的可视化模块
#include <pcl/visualization/pcl_plotter.h>
#include <iostream>

int main()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal> ());

	if (pcl::io::loadPCDFile<pcl::PointXYZ>("room_scan1.pcd", *cloud) == -1)
	{
		PCL_ERROR("Cloudn't read file!");
		return -1;
	}
	cout << "there are " << cloud->points.size() << " points before fpfh estimate." << endl;

	// 计算法向量
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	ne.setInputCloud(cloud);
	ne.setSearchMethod(tree);
	ne.setRadiusSearch(0.03);
	ne.compute(*normals);

	pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh;
	fpfh.setInputCloud (cloud);
	fpfh.setInputNormals (normals);
	// 如果点云是类型为PointNormal,则执行pfh.setInputNormals (cloud);

	fpfh.setSearchMethod (tree);

	pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs (new pcl::PointCloud<pcl::FPFHSignature33> ());

	// 计算pfh特征值,半径设为5cm（此半径要大于表面法线估计所使用的近邻搜索半径）
	fpfh.setRadiusSearch (0.05);

	fpfh.compute (*fpfhs);

	// fpfhs->points.size () should have the same size as the input cloud->points.size ()*
	cout << "there are " << fpfhs->points.size() << " points after fpfh estimate." << endl;

	/*
	pcl::visualization::PCLHistogramVisualizer view; //直方图显示
	view.setBackgroundColor(255, 0, 0);
	view.addFeatureHistogram<pcl::FPFHSignature33>(*fpfhs, "fpfh", 1000);   //对下标为1000的元素可视化
	//view.addFeatureHistogram<pcl::FPFHSignature33>(*fpfhs, 33, "fpfh", 600, 200);   //对下标为1000的元素可视化
	*/

	pcl::visualization::PCLPlotter plotter;
	plotter.addFeatureHistogram(*fpfhs, 300); //设置的很坐标长度，该值越大，则显示的越细致
	plotter.plot();

	/*
	//定义绘图器
	pcl::visualization::PCLPlotter plotter;
	//设置特性
	plotter.setShowLegend(true);
	//显示
	plotter.addFeatureHistogram<pcl::FPFHSignature33>(*fpfhs, "fpfh", 0, "fpfh");
	plotter.setWindowSize(800, 600);
	plotter.spinOnce(30000);
	plotter.clearPlots();
	*/

	return 0;
}