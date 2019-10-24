#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>  //点类型相关定义
#include <pcl/visualization/cloud_viewer.h>  //点云可视化相关定义
#include <pcl/filters/median_filter.h>  //直通滤波相关
#include <pcl/common/common.h>

#include <iostream>
#include <vector>

using namespace std;

int main()
{
	//1.读取点云
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPCDFile<pcl::PointXYZ>("table_scene_lms400.pcd", *cloud) == -1)
	{
		PCL_ERROR("Cloudn't read file!");
		return -1;
	}
	cout << "there are " << cloud->points.size()<<" points before filtering." << endl;

	//2.取得点云坐标极值
	pcl::PointXYZ minPt, maxPt;
	pcl::getMinMax3D(*cloud, minPt, maxPt);

	//3.直通滤波
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::MedianFilter<pcl::PointXYZ> median;
	median.setInputCloud(cloud);		//设置待滤波的点云
	median.setMaxAllowedMovement(15.5);
	median.setWindowSize(3); 
	median.filter(*cloud_filtered);

	//4.滤波结果保存
	pcl::io::savePCDFile<pcl::PointXYZ>("filter.pcd", *cloud_filtered);
	cout << "there are " << cloud_filtered->points.size() << " points after filtering." << endl;

	pcl::visualization::CloudViewer viewer("Cloud Viewer"); //创建viewer对象
	// viewer.showCloud(cloud);
	viewer.showCloud(cloud_filtered);

	system("pause");
	return 0;
}