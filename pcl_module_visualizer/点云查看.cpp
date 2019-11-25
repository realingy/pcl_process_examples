#include <pcl/io/pcd_io.h>  //文件输入输出
#include <pcl/point_types.h>  //点类型相关定义
#include <pcl/visualization/cloud_viewer.h>  //点云可视化相关定义
#include <pcl/filters/voxel_grid.h>  //体素滤波相关
#include <pcl/common/common.h>  

#include <iostream>
#include <vector>

using namespace std;

int main()
{
	//1.读取点云
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPCDFile<pcl::PointXYZ>("boundary.pcd", *cloud) == -1)
	{
		PCL_ERROR("Cloudn't read file!");
		return -1;
	}
	cout << "there are " << cloud->points.size() << " points in point cloud." << endl;

	pcl::visualization::CloudViewer viewer("Cloud Viewer"); //创建viewer对象
	viewer.showCloud(cloud);
	//viewer.showCloud(cloud_filter);

	system("pause");
	return 0;
}