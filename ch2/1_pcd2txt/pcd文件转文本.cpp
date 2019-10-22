#include <iostream>
#include <pcl/io/pcd_io.h>  
#include <pcl/point_types.h>  

using namespace std;

int main(int argc, char *argv[])
{
	time_t begin = clock();

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    // Fill in the cloud data  
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("rabbit.pcd", *cloud) == -1)
    {
        PCL_ERROR("Couldn't read file\n");
        return (-1);
    }

    int Num = cloud->points.size();
    double *X = new double[Num] {0};
    double *Y = new double[Num] {0};
    double *Z = new double[Num] {0};

    for (size_t i = 0; i < cloud->points.size(); ++i)
    {
        X[i] = cloud->points[i].x;
        Y[i] = cloud->points[i].y;
        Z[i] = cloud->points[i].z ;
    }

    ofstream zos("rabbit.txt");
    for (int i = 0; i<Num; i++)
    {
        zos << X[i] << " " << Y[i] << " " << Z[i] << endl;
    }

    cout << "trans has done!" << endl;

	time_t end = clock();
	double interval = double(end - begin) / CLOCKS_PER_SEC;
	cout << "interval: " << interval << endl;

    return 0;
}


