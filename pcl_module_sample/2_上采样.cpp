#include <pcl/io/pcd_io.h>
#include <pcl/surface/mls.h>
#include <pcl/visualization/cloud_viewer.h>  //点云可视化相关定义

int main(int argc,char** argv)
{
	// 新建点云存储对象
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);   
	// 读取文件
	if (pcl::io::loadPCDFile<pcl::PointXYZ>("object_template_0.pcd", *cloud) != 0)
	{
        return -1;
	}
	cout << "cloud point count: " << cloud->points.size() << endl;
	
	// 滤波对象
	pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> filter;
	filter.setInputCloud(cloud); //建立搜索对象
	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree;
	filter.setSearchMethod(kdtree);
	filter.setSearchRadius(0.03); //设置搜索邻域的半径为3cm    
	// Upsampling 采样的方法有 DISTINCT_CLOUD, RANDOM_UNIFORM_DENSITY
	filter.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ>::SAMPLE_LOCAL_PLANE); // 采样的半径是
	filter.setUpsamplingRadius(0.03); // 采样步数的大小
	filter.setUpsamplingStepSize(0.02);
	filter.process(*cloud_filtered);	

	pcl::visualization::CloudViewer viewer("Cloud Viewer1"); //创建viewer对象
	//viewer.showCloud(cloud);
	viewer.showCloud(cloud_filtered);

	cout << "cloud_filtered point count: " << cloud_filtered->points.size() << endl;

	while (!viewer.wasStopped())
	{
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}

	return 0;
}
