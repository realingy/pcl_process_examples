#include <pcl/range_image/range_image.h> //深度图像的头文件

int main (int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZ> pointCloud;   //定义点云的对象  
	// 循环产生点云的数据
	for (float y=-0.5f; y<=0.5f; y+=0.01f) {
		for (float z=-0.5f; z<=0.5f; z+=0.01f) {
			pcl::PointXYZ point;
			point.x = 2.0f - y;
			point.y = y;
			point.z = z;
			pointCloud.points.push_back(point); //循环添加点数据到点云对象
		}
	}
	
	pointCloud.width = (uint32_t) pointCloud.points.size();
	pointCloud.height = 1;   //设置点云对象的头信息    //实现一个呈矩形形状的点云 
	// We now want to create a range image from the above point cloud, with a 1deg angular resolution   
	//angular_resolution模拟深度传感器的角度分辨率，即深度图像中一个像素对应的角度大小
	float angularResolution = (float) (  1.0f * (M_PI/180.0f));  //   1.0 degree in radians   //max_angle_width为模拟的深度传感器的水平最大采样角度，
	float maxAngleWidth     = (float) (360.0f * (M_PI/180.0f));  // 360.0 degree in radians  //max_angle_height为模拟传感器的垂直方向最大采样角度  都转为弧度
	float maxAngleHeight    = (float) (180.0f * (M_PI/180.0f));  // 180.0 degree in radians   //传感器的采集位置
	Eigen::Affine3f sensorPose = (Eigen::Affine3f)Eigen::Translation3f(0.0f, 0.0f, 0.0f);   //深度图像遵循坐标系统
	pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;  float noiseLevel=0.00;    //noise_level获取深度图像深度时，近邻点对查询点距离值的影响水平
	float minRange = 0.0f;     //min_range设置最小的获取距离，小于最小获取距离的位置为传感器的盲区
	int borderSize = 1;        //border_size获得深度图像的边缘的宽度  
	pcl::RangeImage rangeImage;
	rangeImage.createFromPointCloud(pointCloud, angularResolution, maxAngleWidth, maxAngleHeight,sensorPose, coordinate_frame, noiseLevel, minRange, borderSize);
	std::cout << "range image: \n" << rangeImage << "\n";
	
}