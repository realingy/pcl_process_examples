#define  _SCL_SECURE_NO_WARNINGS
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/bilateral.h>
#include <pcl/kdtree/flann.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/search/flann_search.h>
#include <pcl/search/kdtree.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>

#include <fstream>
#include <iostream>
#include <vector>
#include <iomanip>

typedef pcl::PointXYZI PointT;

using namespace std;

int main()
{
	vector<PointT> points;
	fstream fin("rabbit.txt");
	PointT p;
	while (!fin.eof())
	{
		fin >> setprecision(12) >> p.x >> p.y >> p.z >> p.intensity;
		if (fin.peek() == EOF)
		{
			break;
		}
		points.push_back(p);
	}
	fin.close();
	cout << "there are: " << points.size() << " points were readed!" << endl;

	// 读入点云文件
	pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
	cloud->height = 1;
	cloud->width = points.size();
	cloud->is_dense = false;

	for (int i = 0; i < points.size(); i++)
	{
		cloud->points.push_back(points[i]);
	}
	pcl::io::savePCDFileASCII<PointT>("rabbit.pcd", *cloud);

	pcl::PointCloud<PointT> outcloud;

	// 建立kdtree
	//pcl::KdTreeFLANN<PointT>::Ptr tree(new pcl::KdTreeFLANN<PointT>);
	pcl::search::KdTree<PointT>::Ptr tree1(new pcl::search::KdTree<PointT>);
	pcl::BilateralFilter<PointT> bf; //双边滤波
	bf.setInputCloud(cloud);
	bf.setSearchMethod(tree1);
	bf.setHalfSize(1);
	bf.setStdDev(0.5);
	bf.filter(outcloud);

	// 保存滤波输出点云文件
	pcl::io::savePCDFile("rabbit_filter.pcd", outcloud);

	//pcl::visualization::PCLVisualizer viewer("Cloud Viewer");
	//viewer.addPointCloud(outcloud);
	//viewer.setBackgroundColor(1, 0.5, 1);

	pcl::visualization::CloudViewer viewer("Cloud Viewer"); //创建viewer对象
	viewer.showCloud(cloud);
	//viewer.showCloud(outcloud);

	system("pause");
	return 0;
}
