#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>  //kd-tree搜索对象的类定义的头文件
#include <pcl/surface/mls.h> //最小二乘法平滑处理类定义头文件
#include <pcl/visualization/cloud_viewer.h>

int main (int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ> ());
	pcl::io::loadPCDFile ("rabbit.pcd", *cloud);  // 创建 KD-Tree

	//打印处点云总数
	std::cout << "Saved " << cloud->points.size() << " data points to input:" << std::endl;

	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);  // Output has the PointNormal type in order to store the normals calculated by MLS
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_mls(new pcl::PointCloud<pcl::PointNormal>);  // 定义最小二乘实现的对象mls

	pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls; //
	mls.setComputeNormals (true);  //设置在最小二乘计算中需要进行法线估计  
	// Set parameters  
	mls.setInputCloud (cloud);
	mls.setPolynomialFit (true);
	mls.setSearchMethod (tree);
	//mls.setSearchRadius (0.03);  
	mls.setSearchRadius (0.11);  

	// Reconstruct  
	mls.process (*cloud_mls);  // Save output
	pcl::io::savePCDFile ("mls.pcd", *cloud_mls);

	std::cout << "Saved " << cloud_mls->points.size() << " data points to output:" << std::endl;

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ> ());
	pcl::io::loadPCDFile ("mls.pcd", *cloud_out);  // 创建 KD-Tree

	/*图形显示模块*/
	pcl::visualization::CloudViewer viewer("Cloud Viewer"); //创建viewer对象
	viewer.showCloud(cloud_out);

	while (!viewer.wasStopped())
	{
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}

	return 0;
}