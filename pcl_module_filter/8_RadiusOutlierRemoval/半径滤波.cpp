#include <pcl/io/pcd_io.h>  //文件输入输出
#include <pcl/point_types.h>  //点类型相关定义
#include <pcl/visualization/cloud_viewer.h>  //点云可视化相关定义
#include <pcl/filters/radius_outlier_removal.h>  //滤波相关
#include <pcl/common/common.h>  
#include <iostream>

using namespace std;

int main()
{
	//1.读取点云
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PCDReader r;
	r.read<pcl::PointXYZ>("data\\table_scene_lms400.pcd", *cloud);
	cout << "there are " << cloud->points.size() << " points before filtering." << endl;

	//2.半径滤波
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filter(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::RadiusOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud);
	sor.setRadiusSearch(0.02);
	sor.setMinNeighborsInRadius(15);
	sor.setNegative(false); 
	sor.filter(*cloud_filter);  

	//3.滤波结果保存
	pcl::PCDWriter w;
	w.writeASCII<pcl::PointXYZ>("data\\table_scene_lms400_Radius_filter.pcd", *cloud_filter);
	cout << "there are " << cloud_filter->points.size() << " points after filtering." << endl;

	system("pause");
	return 0;
}
