#define  _SCL_SECURE_NO_WARNINGS
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h> //pcd 读写类相关的头文件。
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h> //PCL中支持的点类型头文件。
#include <iostream>

int user_data;
using namespace std;

void viewerOneOff(pcl::visualization::PCLVisualizer & viewer)
{
	viewer.setBackgroundColor(1.0, 0.5, 1.0);   //设置背景颜色
}

int main()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	char strfilepath[256] = "rabbit.pcd";
	if (-1 == pcl::io::loadPCDFile(strfilepath, *cloud)) {
		cout << "error input!" << endl;
		return -1;
	}

	cout << cloud->points.size() << endl;
	pcl::visualization::CloudViewer viewer("Cloud Viewer"); //创建viewer对象

	viewer.showCloud(cloud);
	viewer.runOnVisualizationThreadOnce(viewerOneOff);

	for (int i = 0; i < 10; i++)
	{
		cout << (float)cloud->points[i].x << " " << (float)cloud->points[i].y << " " << (float)cloud->points[i].z << endl;
	}

	system("pause");
	return 0;
}

