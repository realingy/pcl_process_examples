#include <iostream>
#include <vector>
#include <pcl/io/pcd_io.h>  
#include <pcl/point_types.h>  
#include <pcl/visualization/cloud_viewer.h>  

using namespace std;

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

void displayCloudAsHeight(PointCloudT& cloud)
{
	//红绿蓝颜色设置
	int topColor[3];
	int midColor[3];
	int bottomColor[3];
	
	topColor[0] = 255; topColor[1] = 0; topColor[2] = 0;
	midColor[0] = 0;  midColor[1] = 255;  midColor[2] = 0;
	bottomColor[0] = 0; bottomColor[1] = 0; bottomColor[2] = 255;

	int r1 = midColor[0] - bottomColor[0];
	int g1 = midColor[1] - bottomColor[1];
	int b1 = midColor[2] - bottomColor[2];

	int r2 = topColor[0] - midColor[0];
	int g2 = topColor[1] - midColor[1];
	int b2 = topColor[2] - midColor[2];

	float maxz, minz, midz;
	maxz = -FLT_MAX;
	minz = FLT_MAX;

	//获取当前点云高程极值
	for (size_t i = 0; i < cloud.points.size(); ++i)
	{
		if (cloud.points[i].z > maxz) maxz = cloud.points[i].z;
		if (cloud.points[i].z < minz) minz = cloud.points[i].z;
	}
	midz = (maxz + minz) / 2;

	for (size_t i = 0; i < cloud.points.size(); ++i)
	{
		if (cloud.points[i].z < midz){
			float k1 = (cloud.points[i].z - minz) / (midz - minz);
			cloud.points[i].r = bottomColor[0] + r1 * k1;
			cloud.points[i].g = bottomColor[1] + g1 * k1;
			cloud.points[i].b = bottomColor[2] + b1 * k1;
		}
		else{
			float k2 = (cloud.points[i].z - minz) / (maxz - midz);
			cloud.points[i].r = midColor[0] + r2 * k2;
			cloud.points[i].g = midColor[1] + g2 * k2;
			cloud.points[i].b = midColor[2] + b2 * k2;
		}
	}
}


int main()
{
	//1.读取点云
	PointCloudT::Ptr cloud(new PointCloudT);
	if (pcl::io::loadPCDFile("rabbit.pcd", *cloud) == -1)
	{
		PCL_ERROR("Cloudn't read file!");
		return -1;
	}
	cout << "读取点个数: " << cloud->points.size() << endl;

	//2.按高程设置颜色
	displayCloudAsHeight(*cloud);

	//3.显示点云
	pcl::visualization::PCLVisualizer viewer("cloud viewer");
	viewer.addPointCloud<PointT>(cloud, "sample");
	while (!viewer.wasStopped())
	{
		viewer.spinOnce();
	}

	system("pause");
	return 0;
}

