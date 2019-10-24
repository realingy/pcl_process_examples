// 计算点云的某子集的法向量
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <iostream>

int main()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	if (pcl::io::loadPCDFile<pcl::PointXYZ>("table_scene_lms400.pcd", *cloud) == -1)
	{
		PCL_ERROR("Cloudn't read file!");
		return -1;
	}
	cout << "there are " << cloud->points.size() << " points before estimate." << endl;

	// 创建索引集
	std::vector<int> indices(std::floor(cloud->points.size() / 10));
	for (std::size_t i = 0; i < indices.size(); ++i)
		indices[i] = i;

	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	ne.setInputCloud(cloud);

	boost::shared_ptr<std::vector<int> > indicesptr(new std::vector<int>(indices));
	ne.setIndices(indicesptr);

	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	ne.setSearchMethod(tree);

	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);

	ne.setRadiusSearch(0.03);

	ne.compute(*cloud_normals);

	cout << "there are " << cloud_normals->points.size() << " points after estimate." << endl;

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_extract(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	extract.setInputCloud(cloud);
	extract.setIndices(indicesptr);
	extract.setNegative(false);//如果设为true,可以提取指定index之外的点云
	extract.filter(*cloud_extract);

	/*图形显示模块*/
	//显示设置
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));

	//设置背景色
	viewer->setBackgroundColor(0, 0, 0.7);

	//设置点云颜色，该处为单一颜色设置
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, 0, 255, 0);

	//添加需要显示的点云数据
	viewer->addPointCloud<pcl::PointXYZ>(cloud_extract, single_color, "sample cloud");

	//设置点显示大小
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");

	//添加需要显示的点云法向。cloud为原始点云模型，normal为法向信息，10表示需要显示法向的点云间隔，即每10个点显示一次法向，５表示法向长度。
	//viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud_extract, cloud_normals, 10, 5, "normals");
	viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud_extract, cloud_normals);

	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}

	return 0;
}



