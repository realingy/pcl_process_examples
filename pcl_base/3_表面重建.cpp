// 上采样平滑（去除颜色信息）
#include <pcl/io/pcd_io.h>
#include <pcl/surface/mls.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <boost/thread/thread.hpp>

int main(int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);    
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_smooth(new pcl::PointCloud<pcl::PointNormal>);    

	// if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *cloud) != 0)
	if (pcl::io::loadPCDFile<pcl::PointXYZ>("object_template_0.pcd", *cloud) != 0)
	{
	   return -1;
	}    
	cout << "cloud point count: " << cloud->points.size() << endl;

	// Smoothing object (we choose what point types we want as input and output).
	pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> filter;
	filter.setInputCloud(cloud);    // Use all neighbors in a radius of 3cm.
	filter.setSearchRadius(0.03);    // If true, the surface and normal are approximated using a polynomial estimation    
	// (if false, only a tangent one).
	filter.setPolynomialFit(true);    
	// We can tell the algorithm to also compute smoothed normals (optional).
	filter.setComputeNormals(true);   
	// kd-tree object for performing searches.
	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree;
	filter.setSearchMethod(kdtree);
	filter.process(*cloud_smooth);

	cout << "cloud_smooth point count: " << cloud_smooth->points.size() << endl;

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("smooth"));
	viewer->addPointCloud<pcl::PointNormal>(cloud_smooth,"smoothed");
	while(!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(1000000));
	}

	return 0;
}
