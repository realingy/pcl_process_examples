#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/cloud_viewer.h>  //点云可视化相关定义
#include <iostream>

int main(int argc, char** argv)
{
	//载入pcd数据后，对数据进行下采样
	pcl::PCLPointCloud2::Ptr cloud_blob(new pcl::PCLPointCloud2)
		, cloud_filtered_blob(new pcl::PCLPointCloud2); //申明滤波前后的点云
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>)
		, cloud(new pcl::PointCloud<pcl::PointXYZ>)
		, cloud_p(new pcl::PointCloud<pcl::PointXYZ>)
		, cloud_f(new pcl::PointCloud<pcl::PointXYZ>);

	//读取pcd文件  
	pcl::PCDReader reader;
	reader.read("table_scene_lms400.pcd", *cloud_blob); //统计滤波前的点云个数
	//reader.read("table_scene_lms400_plane_1.pcd", *cloud_blob); //统计滤波前的点云个数

	std::cerr << "PointCloud before filtering: " << cloud_blob->width * cloud_blob->height << " data points." << std::endl;
	
	// 创建体素栅格下采样: 下采样的大小为1cm
	pcl::VoxelGrid<pcl::PCLPointCloud2> sor; //体素栅格下采样对象
	sor.setInputCloud(cloud_blob);           //原始点云
	sor.setLeafSize(0.01f, 0.01f, 0.01f);    //设置采样体素大小
	sor.filter(*cloud_filtered_blob);        //保存
	
	//转换为模板点云
	pcl::fromPCLPointCloud2(*cloud_filtered_blob, *cloud_filtered);
	pcl::fromPCLPointCloud2(*cloud_blob, *cloud);

	std::cerr << "PointCloud after filtering: " << cloud_filtered->points.size() << " data points." << std::endl;  // 保存下采样后的点云

	//pcl::PCDWriter writer;
	//writer.write<pcl::PointXYZ>("table_scene_lms400_downsampled.pcd", *cloud_filtered, false);

	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
	pcl::SACSegmentation<pcl::PointXYZ> seg;		//创建分割对象
	seg.setOptimizeCoefficients(true);				//设置对估计模型参数进行优化处理
	seg.setModelType(pcl::SACMODEL_PLANE);          //设置分割模型类别
	seg.setMethodType(pcl::SAC_RANSAC);				//设置用哪个随机参数估计方法
	seg.setMaxIterations(1000);                     //设置最大迭代次数
	seg.setDistanceThreshold(0.01);					//判断是否为模型内点的距离阀值  

	// 设置ExtractIndices的实际参数
	pcl::ExtractIndices<pcl::PointXYZ> extract;		//创建点云提取对象
	int nr_points = (int)cloud_filtered->points.size();  // While 30% of the original cloud is still there

	while (cloud_filtered->points.size() > 0.3 * nr_points)
	{
		// 为了处理点云包含的多个模型，在一个循环中执行该过程并在每次模型被提取后，保存剩余的点进行迭代
		seg.setInputCloud(cloud_filtered);
		seg.segment(*inliers, *coefficients);
		if (inliers->indices.size() == 0)
		{
			std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;      break;
		}

		// Extract the inliers  
		extract.setInputCloud(cloud_filtered);
		extract.setIndices(inliers);
		extract.setNegative(false);
		extract.filter(*cloud_p);
		std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;
		std::stringstream ss;
		// ss << "table_scene_lms400_plane_" << i << ".pcd";
		// writer.write<pcl::PointXYZ>(ss.str(), *cloud_p, false);

		// Create the filtering object
		extract.setNegative(true);
		extract.filter(*cloud_f);
		cloud_filtered.swap(cloud_f);
	}

	//pcl::visualization::CloudViewer viewer("cloud"); //创建viewer对象
	//viewer.showCloud(cloud);
	pcl::visualization::CloudViewer viewer2("cloud_filtered"); //创建viewer对象
	viewer2.showCloud(cloud_filtered);

	//viewer.showCloud(cloud_f);

	system("pause");
	return (0);
}
