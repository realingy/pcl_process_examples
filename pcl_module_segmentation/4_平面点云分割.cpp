#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>

int main (int argc, char** argv)
{
	// 读取文件   
	pcl::PCDReader reader;
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>), cloud_f (new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr final (new pcl::PointCloud<pcl::PointXYZRGBA>);
	reader.read ("object_template_0.pcd", *cloud);
	std::cout << "PointCloud before filtering has: " << cloud->points.size () << " data points." << std::endl;

	// 下采样，体素叶子大小为0.01
	pcl::VoxelGrid<pcl::PointXYZRGBA> vg;
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGBA>);
	vg.setInputCloud (cloud);
	vg.setLeafSize (0.01f, 0.01f, 0.01f);
	vg.filter (*cloud_filtered);
	std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size () << " data points." << std::endl;

	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);  

	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZRGBA> seg;  
	seg.setOptimizeCoefficients (true);   
	seg.setModelType (pcl::SACMODEL_PLANE);
	// seg.setModelType(pcl::SACMODEL_LINE ); 
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setDistanceThreshold (0.01);
	seg.setInputCloud (cloud_filtered);
	seg.segment (*inliers, *coefficients);

	if (inliers->indices.size () == 0)
	{
		PCL_ERROR ("Could not estimate a planar model for the given dataset.");
		return (-1);
	}
	
	std::cerr << "Model coefficients: " << coefficients->values[0] << " "
										<< coefficients->values[1] << " "
										<< coefficients->values[2] << " "
										<< coefficients->values[3] << std::endl;
	return (0);
}

