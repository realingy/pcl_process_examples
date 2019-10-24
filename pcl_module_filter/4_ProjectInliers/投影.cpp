#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h> //模型系数头文件
#include <pcl/filters/project_inliers.h> //投影滤波类头文件
#include <pcl/visualization/cloud_viewer.h>  //点云可视化相关定义

int main()
{
	/*
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected(new pcl::PointCloud<pcl::PointXYZ>);  //创建点云并打印出来

	cloud->width = 5;
	cloud->height = 1;
	cloud->points.resize(cloud->width * cloud->height);  for (size_t i = 0; i < cloud->points.size(); ++i)
	{
		cloud->points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
		cloud->points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
		cloud->points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
	}

	std::cerr << "cloud before projection: " << std::endl;
	for (size_t i = 0; i < cloud->points.size(); ++i)
		std::cerr << "    " << cloud->points[i].x << " "
		<< cloud->points[i].y << " "
		<< cloud->points[i].z << std::endl;

	// 填充ModelCoefficients的值,使用ax+by+cz+d=0平面模型，其中 a=b=d=0,c=1 也就是X——Y平面
	//定义模型系数对象，并填充对应的数据
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
	coefficients->values.resize(4);
	coefficients->values[0] = coefficients->values[1] = 0;
	coefficients->values[2] = 1.0;
	coefficients->values[3] = 0;

	// 创建ProjectInliers对象，使用ModelCoefficients作为投影对象的模型参数
	pcl::ProjectInliers<pcl::PointXYZ> proj;     //创建投影滤波对象
	proj.setModelType(pcl::SACMODEL_PLANE);      //设置对象对应的投影模型
	proj.setInputCloud(cloud);                   //设置输入点云
	proj.setModelCoefficients(coefficients);     //设置模型对应的系数
	proj.filter(*cloud_projected);               //投影结果存储

	std::cerr << "cloud after projection: " << std::endl;
	for (size_t i = 0; i < cloud_projected->points.size(); ++i)
		std::cerr << "    " << cloud_projected->points[i].x << " "
		<< cloud_projected->points[i].y << " "
		<< cloud_projected->points[i].z << std::endl;
	*/

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected(new pcl::PointCloud<pcl::PointXYZ>);  //创建点云并打印出来

	if (pcl::io::loadPCDFile<pcl::PointXYZ>("rabbit.pcd", *cloud) == -1)
	{
		PCL_ERROR("Cloudn't read file!");
		return -1;
	}
	std::cout << "there are " << cloud->points.size() << " points before filtering." << std::endl;

	// 填充ModelCoefficients的值,使用ax+by+cz+d=0平面模型，其中 a=b=d=0,c=1 也就是X——Y平面
	//定义模型系数对象，并填充对应的数据
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
	coefficients->values.resize(4);
	coefficients->values[0] = coefficients->values[1] = 0;
	coefficients->values[2] = 1.0;
	coefficients->values[3] = 0;

	// 创建ProjectInliers对象，使用ModelCoefficients作为投影对象的模型参数
	pcl::ProjectInliers<pcl::PointXYZ> proj;  //创建投影滤波对象
	proj.setModelType(pcl::SACMODEL_PLANE);   //设置对象对应的投影模型
	proj.setInputCloud(cloud);                //设置输入点云
	proj.setModelCoefficients(coefficients);  //设置模型对应的系数
	proj.filter(*cloud_projected);            //投影结果存储

	std::cout << "there are " << cloud_projected->points.size() << " points after filtering." << std::endl;

	pcl::visualization::CloudViewer viewer("Cloud Viewer"); //创建viewer对象
	//viewer.showCloud(cloud);
	viewer.showCloud(cloud_projected);
	
	system("pause");
	return (0);
}

