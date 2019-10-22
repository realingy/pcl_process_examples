#define  _SCL_SECURE_NO_WARNINGS
#include <iostream>
#include <fstream>
#include <pcl/io/pcd_io.h>  //文件输入输出
#include <pcl/point_types.h>  //点类型相关定义
#include <pcl/visualization/cloud_viewer.h>  //点类型相关定义

using namespace std;
using namespace pcl;

int main()
{
	PointCloud <PointXYZ> ::Ptr cloud1(new PointCloud<PointXYZ>);
	if (io::loadPCDFile<PointXYZ>("rabbit.pcd", *cloud1) == -1)
	{
		PCL_ERROR("Cloudn't read file!");
		return -1;
	}
	cout << "1.loadPCDFile方式使用指针读取点个数: " << cloud1->points.size() << endl;

	PointCloud<PointXYZ> cloud;
	if (io::loadPCDFile<PointXYZ>("rabbit.pcd", cloud) == -1)
	{
		PCL_ERROR("Cloudn't read file!");
		return -1;
	}
	cout << "2.loadPCDFile方式使用对象读取点个数: " << cloud.points.size() << endl;

	PointCloud<PointXYZ>::Ptr cloud2(new PointCloud<PointXYZ>);
	PCDReader reader;
	reader.read<PointXYZ>("rabbit.pcd", *cloud2);
	cout << "3.PCDReader方式读取点个数: " << cloud2->points.size() << endl;

	io::savePCDFileASCII<PointXYZ>("rabbit_ascii_io.pcd", cloud); //ASCII方式保存
	io::savePCDFileBinary<PointXYZ>("rabbit_bin_io.pcd", cloud); //二进制方式保存
	io::savePCDFile<PointXYZ>("rabbit_io.pcd", cloud); //默认ASCII方式保存

	PCDWriter writer;
	writer.writeASCII<PointXYZ>("rabbit_ascii_pcdwrite.pcd", cloud); //ASCII方式保存
	writer.writeBinary<PointXYZ>("rabbit_bin_pcdwrite.pcd", cloud); //二进制方式保存
	writer.write<PointXYZ>("rabbit_pcdwrite.pcd", cloud); //默认ascii方式保存

	visualization::CloudViewer viewer("cloud viewer");
	viewer.showCloud(cloud1);

	system("pause");
	return 0;
}

