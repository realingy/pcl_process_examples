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
	/****************************************************************************************
	*****************************************************************************************
	** (一) 读取点云数据
	*****************************************************************************************
	****************************************************************************************/

	//1.读取点云
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPCDFile<pcl::PointXYZ>("rabbit.pcd", *cloud) == -1)
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
	pcl::PassThrough<pcl::PointXYZ> pass;		//创建滤波器对象
	pass.setInputCloud(cloud);					//设置待滤波的点云
	pass.setFilterFieldName("z");			    //设置在Z轴方向上进行滤波
	//pass.setFilterLimits(0, maxPt.z - 1);		//设置滤波范围
	pass.setFilterLimits(minPt.z, maxPt.z - 5); //设置滤波范围
	pass.setFilterLimitsNegative(false);		//保留
	pass.filter(*cloud_filter);					//滤波并存储

	//4.滤波结果保存
	pcl::io::savePCDFile<pcl::PointXYZ>("filter.pcd", *cloud_filter);
	cout << "there are " << cloud_filter->points.size() << " points after filtering." << endl;

	pcl::visualization::CloudViewer viewer("Cloud Viewer"); //创建viewer对象
	// viewer.showCloud(cloud);
	viewer.showCloud(cloud_filter);


	/****************************************************************************************
	*****************************************************************************************
	** (二) 自定义点云数据
	*****************************************************************************************
	****************************************************************************************/
	/*
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

	cloud->width  = 5;
	cloud->height = 1;
	cloud->points.resize (cloud->width * cloud->height);
	for (size_t i = 0; i < cloud->points.size (); ++i)   //填充数据
	{
		cloud->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
		cloud->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
		cloud->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
	}
	std::cerr << "cloud before filtering: " << std::endl;   //打印
	
	for (size_t i = 0; i < cloud->points.size (); ++i)
		std::cerr << "    " << cloud->points[i].x << " "
							<< cloud->points[i].y << " "
							<< cloud->points[i].z << std::endl;

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
	// 设置滤波器对象
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud (cloud);          //设置输入点云
	pass.setFilterFieldName ("z");       //设置过滤时所需要点云类型的Z字段
	pass.setFilterLimits (0.0, 500.0);   //设置在过滤字段的范围
	pass.setFilterLimitsNegative (true); //设置保留范围内还是过滤掉范围内 false: 保留 true：过滤
	pass.filter (*cloud_filtered);       //执行滤波
	std::cerr << "cloud after filtering: " << std::endl;   //打印
	for (size_t i = 0; i < cloud_filtered->points.size (); ++i)
	std::cerr << "    " << cloud_filtered->points[i].x << " "
					   << cloud_filtered->points[i].y << " "
					   << cloud_filtered->points[i].z << std::endl;
	*/

	system("pause");
	return 0;
}