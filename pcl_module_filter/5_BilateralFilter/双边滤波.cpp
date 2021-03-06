#include <pcl/io/pcd_io.h>  //文件输入输出
#include <pcl/point_types.h>  //点类型相关定义
#include <pcl/visualization/cloud_viewer.h>  //点云可视化相关定义
#include <pcl/filters/bilateral.h>
#include <pcl/common/common.h>  

#include <iostream>
#include <vector>

using namespace std;

int main()
{
	//1.读取点云
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPCDFile<pcl::PointXYZ>("rabbit.pcd", *cloud) == -1)
	{
		PCL_ERROR("Cloudn't read file!");
		return -1;
	}
	cout << "there are " << cloud->points.size() << " points before filtering." << endl;

	//2.体素栅格滤波
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filter(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());

	pcl::BilateralFilter<pcl::PointXYZ> bf; //双边滤波
	bf.setInputCloud(cloud);
	bf.setSearchMethod(tree);
	bf.setHalfSize(1);
	bf.setStdDev(0.5);
	bf.filter(*cloud_filter);
	//pcl::VoxelGrid<pcl::PointXYZ> sor;
	//sor.setInputCloud(cloud);
	//sor.setLeafSize(0.3f, 0.3f, 0.3f);//体素大小设置为30*30*30cm
	//sor.setLeafSize(0.01f, 0.01f, 0.01f);//体素大小设置为1*1*1cm
	//sor.filter(*cloud_filter);
	
	//3.滤波结果保存
	pcl::io::savePCDFile<pcl::PointXYZ>("filter.pcd", *cloud_filter);
	cout << "there are " << cloud_filter->points.size() << " points after filtering." << endl;

	pcl::visualization::CloudViewer viewer("Cloud Viewer"); //创建viewer对象
	viewer.showCloud(cloud);
	//viewer.showCloud(cloud_filter);

	system("pause");
	return 0;
}