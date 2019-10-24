// 点特征分布图(PFH)估计
#include <pcl/point_types.h>
#include <pcl/features/pfh.h>
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

	if (pcl::io::loadPCDFile<pcl::PointXYZ>("rabbit.pcd", *cloud) == -1)
	{
		PCL_ERROR("Cloudn't read file!");
		return -1;
	}
	cout << "there are " << cloud->points.size() << " points before pfh estimate." << endl;

	// 计算法向量
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	ne.setInputCloud(cloud);
	ne.setSearchMethod(tree);
	ne.setRadiusSearch(0.03);
	ne.compute(*normals);

	// 创建PFH估计对象pfh，并将输入点云数据集cloud和法线normals传递给它
	pcl::PointCloud<pcl::PFHSignature125>::Ptr pfhs (new pcl::PointCloud<pcl::PFHSignature125> ());
	pcl::PFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::PFHSignature125> pfh;
	pfh.setInputCloud (cloud);
	pfh.setInputNormals (normals);
	// 如果点云是类型为PointNormal,则执行pfh.setInputNormals (cloud);

	pfh.setSearchMethod (tree);

	// 计算pfh特征值,半径设为5cm（此半径要大于表面法线估计所使用的近邻搜索半径）
	pfh.setRadiusSearch (0.05);

	pfh.compute (*pfhs);

	// pfhs->points.size()应该与input cloud->points.size()有相同的大小，即每个点都有一个pfh特征向量
	cout << "there are " << pfhs->points.size() << " points after pfh estimate." << endl;

	/*
	pcl::visualization::PCLHistogramVisualizer view; //直方图显示
	view.setBackgroundColor(255, 0, 0);
	view.addFeatureHistogram<pcl::PFHSignature125>(*pfhs, "fpfh", 1000);   //对下标为1000的元素可视化
	*/

	pcl::visualization::PCLPlotter plotter;
	plotter.addFeatureHistogram(*pfhs, 300); //设置的很坐标长度，该值越大，则显示的越细致
	plotter.plot();

	return 0;
}

