#include <pcl/io/pcd_io.h>  //�ļ��������
#include <pcl/point_types.h>  //��������ض���
#include <pcl/visualization/cloud_viewer.h>  //���ƿ��ӻ���ض���
#include <pcl/filters/voxel_grid.h>  //�����˲����
#include <pcl/common/common.h>  

#include <iostream>
#include <vector>

using namespace std;

int main()
{
	//1.��ȡ����
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	//if (pcl::io::loadPCDFile<pcl::PointXYZ>("mls_points.pcd", *cloud) == -1)
	if (pcl::io::loadPCDFile<pcl::PointXYZ>("pnts3D_pcd.pcd", *cloud) == -1)
	{
		PCL_ERROR("Cloudn't read file!");
		return -1;
	}
	cout << "there are " << cloud->points.size() << " points in point cloud." << endl;

	pcl::visualization::CloudViewer viewer("Cloud Viewer"); //����viewer����
	viewer.showCloud(cloud);
	//viewer.showCloud(cloud_filter);

	system("pause");
	return 0;
}