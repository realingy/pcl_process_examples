// 计算下采样点云的法向量
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>  //体素滤波相关
#include <iostream>


int main()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsampled (new pcl::PointCloud<pcl::PointXYZ>);

	if (pcl::io::loadPCDFile<pcl::PointXYZ>("table_scene_lms400.pcd", *cloud) == -1)
	{
		PCL_ERROR("Cloudn't read file!");
		return -1;
	}
	cout << "there are " << cloud->points.size() << " points before estimate." << endl;

	// 创建体素网格滤波器对象
	pcl::VoxelGrid<pcl::PointXYZ> vgf;
	vgf.setInputCloud(cloud);
	vgf.setLeafSize(0.01f, 0.01f, 0.01f);// 单位：m
	//sor.setLeafSize(0.3f, 0.3f, 0.3f);// 单位：m
	vgf.filter(*cloud_downsampled);

	// 计算下采样点云的法向量
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	ne.setInputCloud (cloud_downsampled);

	// 原点云数据作为近邻搜索面
	ne.setSearchSurface (cloud);

	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
	ne.setSearchMethod (tree);

	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

	ne.setRadiusSearch (0.03);

	ne.compute (*cloud_normals);

	// cloud_normals->points.size () should have the same size as the input cloud_downsampled->points.size ()
	cout << "there are " << cloud_normals->points.size() << " points after estimate." << endl;

	/*图形显示模块*/
	//显示设置
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));

	//设置背景色
	viewer->setBackgroundColor(0, 0, 0.7);

	//设置点云颜色，该处为单一颜色设置
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, 0, 255, 0);

	//添加需要显示的点云数据
	viewer->addPointCloud<pcl::PointXYZ>(cloud_downsampled, single_color, "sample cloud");

	//设置点显示大小
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");

	//添加需要显示的点云法向。cloud为原始点云模型，normal为法向信息，10表示需要显示法向的点云间隔，即每10个点显示一次法向，５表示法向长度。
	//viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, cloud_normals, 10, 5, "normals");
	viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud_downsampled, cloud_normals);

	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}

	return 0;
}





