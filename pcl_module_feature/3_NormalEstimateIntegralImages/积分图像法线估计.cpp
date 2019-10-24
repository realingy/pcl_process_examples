// 通过积分图像的法线估计
#include <pcl/point_types.h>
#include <pcl/features/integral_image_normal.h> //积分图像法线估计
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>  //体素滤波相关
#include <iostream>

int main()
{
	// 加载点云
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPCDFile<pcl::PointXYZ>("rabbit.pcd", *cloud) == -1)
	{
		PCL_ERROR("Cloudn't read file!");
		return -1;
	}
	cout << "there are " << cloud->points.size() << " points before estimate." << endl;

	// 法线估计
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

	pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	ne.setInputCloud(cloud);
	ne.setNormalEstimationMethod(ne.AVERAGE_3D_GRADIENT);
	ne.setMaxDepthChangeFactor(0.02f);
	ne.setNormalSmoothingSize(10.0f);
	ne.compute(*normals);

	// visualize normals
	pcl::visualization::PCLVisualizer viewer("PCL Viewer");
	viewer.setBackgroundColor(0.0, 0.0, 0.5);
	viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, normals);

	while (!viewer.wasStopped())
	{
		viewer.spinOnce();
	}
	return 0;
}
