#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>  //点类型相关定义
#include <pcl/visualization/cloud_viewer.h>  //点云可视化相关定义
#include <pcl/filters/passthrough.h>  //直通滤波相关
#include <pcl/common/common.h>

#include <iostream>
#include <vector>

using namespace std;

int main()
{
	//1.读取点云
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPCDFile<pcl::PointXYZ>("person.pcd", *cloud) == -1)
	{
		PCL_ERROR("Cloudn't read file!");
		return -1;
	}
	cout << "there are " << cloud->points.size()<<" points before filtering." << endl;

	//2.取得点云坐标极值
	pcl::PointXYZ minPt, maxPt;
	pcl::getMinMax3D(*cloud, minPt, maxPt);

	//3.直通滤波
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filter(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PassThrough<pcl::PointXYZ> pass;     //创建滤波器对象
	pass.setInputCloud(cloud);                //设置待滤波的点云
	pass.setFilterFieldName("z");             //设置在Z轴方向上进行滤波
	//pass.setFilterLimits(0, maxPt.z - 12);    //设置滤波范围(从最高点向下12米去除)
	pass.setFilterLimits(0, maxPt.z - 1);    //设置滤波范围(从最高点向下12米去除)
	pass.setFilterLimitsNegative(false);      //保留
	pass.filter(*cloud_filter);               //滤波并存储

	//4.滤波结果保存
	pcl::io::savePCDFile<pcl::PointXYZ>("filter.pcd", *cloud_filter);
	cout << "there are " << cloud_filter->points.size() << " points after filtering." << endl;

	pcl::visualization::CloudViewer viewer("Cloud Viewer"); //创建viewer对象
	viewer.showCloud(cloud);
	// viewer.showCloud(cloud_filter);
	
	system("pause");
	return 0;
}