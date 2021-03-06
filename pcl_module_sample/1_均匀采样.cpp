#include <pcl/io/pcd_io.h>
#include <pcl/keypoints/uniform_sampling.h>

int main(int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZ>);

	//if (pcl::io::loadPCDFile<pcl::PointXYZ>("rabbit.pcd", *cloud) != 0)
	if (pcl::io::loadPCDFile<pcl::PointXYZ>("20160426_CALIB.pcd", *cloud) != 0)
	{
		return -1;
	}

	std::cout << "count : " << cloud->points.size() << "before filter" << std::endl;

	// Uniform sampling object.
	pcl::UniformSampling<pcl::PointXYZ> filter;
	filter.setInputCloud(cloud);
	filter.setRadiusSearch(0.01f);    
	// We need an additional object to store the indices of surviving points.
	pcl::PointCloud<int> keypointIndices;
	// filter.compute(keypointIndices);
	pcl::copyPointCloud(*cloud, keypointIndices.points, *filteredCloud);

	std::cout << "count: " << cloud->points.size() << "after filter" << std::endl;
	
	return 0;
}