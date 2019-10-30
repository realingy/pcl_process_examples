#include <iostream>  
#include <vector>
#include <string>

#include "opencv2/core.hpp"  
#include "opencv2/core/utility.hpp"  
#include "opencv2/core/ocl.hpp"  
#include "opencv2/imgcodecs.hpp"  
#include "opencv2/highgui.hpp"  
#include "opencv2/features2d.hpp"  
#include "opencv2/calib3d.hpp"  
#include "opencv2/imgproc.hpp"  
#include "opencv2/flann.hpp"  
#include "opencv2/xfeatures2d.hpp"  
#include "opencv2/ml.hpp"  

#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/transformation_estimation.h>
#include <pcl/registration/correspondence_estimation.h>

using namespace cv;
using namespace std;
using namespace cv::xfeatures2d;
using namespace cv::ml;

// --------------------------------------
// --------------Parameters--------------
// --------------------------------------
const double camera_factor = 1000;
const double camera_cx = 979.674;
const double camera_cy = 535.383;
const double camera_fx = 1043.02;
const double camera_fy = 1047.78;
string imagelocation_rgb1 = ".\\r-1.png";
string imagelocation_rgb2 = ".\\r-2.png";
string imagelocation_depth1 = ".\\d-1.png";
string imagelocation_depth2 = ".\\d-2.png";
string outputfilename = "transformed.ply";
int min_number_keypoints = 100;


void printUsage(const char* progName);
int getSurfMatches(const string& rgb1_filelocation, const string& rgb2_filelocation, vector<KeyPoint>& keypoints1, vector<KeyPoint>& keypoints2, vector<DMatch>& matches);
int ransacSurfMatchesuseFundamentalMat(vector<KeyPoint>& keypoint1, vector<KeyPoint>& keypoint2, vector<DMatch>& matches);
int keypoints2cloud(vector<KeyPoint>& keypoints1, pcl::PointCloud<pcl::PointXYZ>::Ptr& keycloud1, vector<KeyPoint>& keypoints2, pcl::PointCloud<pcl::PointXYZ>::Ptr& keycloud2);
int rgb_depth2cloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& cloud, const string& rgb_filelocation, const string& imagelocation_depth);
int CorrespondenceRejectorSC(pcl::PointCloud<pcl::PointXYZ>::Ptr& keycloud1, pcl::PointCloud<pcl::PointXYZ>::Ptr& keycloud2, boost::shared_ptr<pcl::Correspondences>& Correspon_in, boost::shared_ptr<pcl::Correspondences>& Correspon_out);
void Transform_use_RTmatrixandSave(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& sourcecloud, Eigen::Matrix4f& Transform, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& transformed_cloud);
boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis(pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr transformed_sourcecloud1, pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr sourcecloud2);


int main(int argc, char** argv)
{
	// --------------------------------------
	// -----Parse Command Line Arguments-----
	// --------------------------------------
	if (pcl::console::find_argument(argc, argv, "-h") >= 0)
	{
		printUsage(argv[0]);
		return 0;
	}

	if (pcl::console::parse(argc, argv, "-n", min_number_keypoints) >= 0)
		cout << "Setting min_number_keypoints to: " << min_number_keypoints << ".\n";
	else
		cout << "Use default min_number_keypoints : " << min_number_keypoints << ".\n";

	if (pcl::console::parse(argc, argv, "-o", outputfilename) >= 0)
		cout << "Setting transformed cloud output filename is: " << outputfilename << ".\n";
	else
		cout << "Use default output filename : " << outputfilename << ".\n";

	vector<int> image_filename_indices = pcl::console::parse_file_extension_argument(argc, argv, "png");
	if (image_filename_indices.size() == 4)
	{
		cout << "Use Input data.\n"
			<< "imagelocation_rgb1------>" << (imagelocation_rgb1 = argv[image_filename_indices[0]]) << endl
			<< "imagelocation_rgb2------>" << (imagelocation_rgb2 = argv[image_filename_indices[1]]) << endl
			<< "imagelocation_depth1------>" << (imagelocation_depth1 = argv[image_filename_indices[2]]) << endl
			<< "imagelocation_depth2------>" << (imagelocation_depth2 = argv[image_filename_indices[3]]) << endl
			<< endl << endl;
	}
	else
	{
		cout << "Use default data.\n"
			<< "imagelocation_rgb1------>" << imagelocation_rgb1 << endl
			<< "imagelocation_rgb2------>" << imagelocation_rgb2 << endl
			<< "imagelocation_depth1------>" << imagelocation_depth1 << endl
			<< "imagelocation_depth2------>" << imagelocation_depth2 << endl
			<< endl << endl;
	}

	clock_t startTime, endTime;
	startTime = clock();

	// --------------------------------------
	// --------------得到粗匹配--------------
	// --------------------------------------
	vector<KeyPoint>key1, key2;
	vector<DMatch> matches;
	getSurfMatches(imagelocation_rgb1, imagelocation_rgb2, key1, key2, matches);

	// --------------------------------------
	// ---------FundamentalMatRansac---------
	// --------------------------------------
	int n_keypoints = 0;
	while (1) {
		if ((n_keypoints = ransacSurfMatchesuseFundamentalMat(key1, key2, matches)) < min_number_keypoints)
			break;
		if (ransacSurfMatchesuseFundamentalMat(key1, key2, matches) == n_keypoints)
			break;
	}

	// --------------------------------------------
	// ------Generate the matches pointcloud-------
	// --------------------------------------------
	pcl::PointCloud<pcl::PointXYZ>::Ptr key1cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr key2cloud(new pcl::PointCloud<pcl::PointXYZ>);
	// 关键点转换点云
	keypoints2cloud(key1, key1cloud, key2, key2cloud);

	// ------------------------------------------------------
	// ---------Ransac use keypoint cloud 误匹配校正---------
	// ------------------------------------------------------
	boost::shared_ptr<pcl::Correspondences> correspondence_in(new pcl::Correspondences);
	boost::shared_ptr<pcl::Correspondences> correspondence_out(new pcl::Correspondences);
	CorrespondenceRejectorSC(key1cloud, key2cloud, correspondence_in, correspondence_out);
	//CorrespondenceRejectorSC(key1cloud, key2cloud, correspondence_in, correspondence_out);

	// ------------------------------------------------------------
	// -----------Compute the RT matrix 计算旋转平移矩阵-----------
	// ------------------------------------------------------------
	// 3维空间中的变换矩阵是4*4的，因为涉及到旋转和平移
	// 2维平面上的变换矩阵是3*3的，因为只涉及旋转不涉及平移(平移向量)
	Eigen::Matrix4f Transform = Eigen::Matrix4f::Identity();
	pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ, float>::Ptr
		trans(new pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ, float>);
	trans->estimateRigidTransformation(*key1cloud, *key2cloud, *correspondence_out, Transform);
	endTime = clock();
	cout << endl << "旋转平移矩阵为: " << endl;
	cout << Transform << endl << endl;
	cout << "得到旋转平移耗时: " << (endTime - startTime) / CLOCKS_PER_SEC << "(s)" << endl << endl;

	// ------------------------------------------------------
	// ------Generate the source pointcloud 生成源点云-------
	// ------------------------------------------------------
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr sourcecloud1(new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr sourcecloud2(new pcl::PointCloud<pcl::PointXYZRGBA>);
	// 根据rgb图像和深度图生成点云数据
	rgb_depth2cloud(sourcecloud1, imagelocation_rgb1, imagelocation_depth1);
	rgb_depth2cloud(sourcecloud2, imagelocation_rgb2, imagelocation_depth2);

	// -----------------------------------------------------
	// ------Transfrom sourcecloud and save 点云变换--------
	// -----------------------------------------------------
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
	// 根据变换矩阵进行点云数据透视变换
	Transform_use_RTmatrixandSave(sourcecloud1, Transform, transformed_cloud);

	// -----------------------------------------------------
	// ---------Visualization the result 查看配准结果-------
	// -----------------------------------------------------
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	viewer = simpleVis(transformed_cloud, sourcecloud2);
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}

	return 0;
}

// 对粗选择部分进行Ransac操作，进行误匹配校正
int ransacSurfMatchesuseFundamentalMat
(vector<KeyPoint>& keypoint1, vector<KeyPoint>& keypoint2, vector<DMatch>& matches)
{
	static int count = 0;
	vector<Point2f>p1, p2;
	for (size_t i = 0; i < matches.size(); i++)
	{
		p1.push_back(keypoint1[i].pt);
		p2.push_back(keypoint2[i].pt);
	}

	vector<uchar> RansacStatus;
	Mat Fundamental = findFundamentalMat(p1, p2, RansacStatus, FM_RANSAC);

	vector<KeyPoint> tmp_Ransac_keypoint1, tmp_Ransac_keypoint2;
	vector<DMatch> tmp_Ransac_matches;

	int index = 0;
	for (size_t i = 0; i < matches.size(); i++)
	{
		if (RansacStatus[i] != 0)
		{
			tmp_Ransac_keypoint1.push_back(keypoint1[i]);
			tmp_Ransac_keypoint2.push_back(keypoint2[i]);
			matches[i].queryIdx = index;
			matches[i].trainIdx = index;
			tmp_Ransac_matches.push_back(matches[i]);
			index++;
		}
	}

	keypoint1.clear();
	keypoint2.clear();
	matches.clear();
	keypoint1 = tmp_Ransac_keypoint1;
	keypoint2 = tmp_Ransac_keypoint2;
	matches = tmp_Ransac_matches;

	cout << "=================================[2]=================================" << endl
			<< "对粗选择部分进行" << ++count << "次Ransac操作后的匹配数量为: " << matches.size() << endl;
	return matches.size();
}

void printUsage(const char* progName)
{
	cout << "\n\nUsage: " << progName << " [options] rgb1.pgn rgb2.pgn depth1.png depth2.png\n\n"
		<< "Options:\n"
		<< "-------------------------------------------------------------------\n"
		<< "-n <int>    keep the last time FundamentalMatRansac have matches uper than (default " << min_number_keypoints << " )\n"
		<< "-o <char*>  output filename (default " << outputfilename << " )\n"
		<< "-h          this help\n"
		<< "\n\n";
}

int getSurfMatches(const string& rgb1_filelocation, const string& rgb2_filelocation, vector<KeyPoint>& keypoints1, vector<KeyPoint>& keypoints2,
	vector<DMatch>& matches)
{
	Mat rgb1 = imread(rgb1_filelocation, IMREAD_GRAYSCALE);
	Mat rgb2 = imread(rgb2_filelocation, IMREAD_GRAYSCALE);

	if (rgb1.empty() || rgb2.empty()) {
		cout << "load image filed! Please check rgb image file location!\n\n";
		exit(-1);
	}

	Ptr<SURF> surf;
	surf = SURF::create(800);
	BFMatcher matcher;
	Mat c, d;
	vector<KeyPoint>tmp_key1, tmp_key2;
	vector<DMatch> tmp_matches;

	surf->detectAndCompute(rgb1, Mat(), tmp_key1, c);
	surf->detectAndCompute(rgb2, Mat(), tmp_key2, d);
	matcher.match(c, d, tmp_matches);
	sort(tmp_matches.begin(), tmp_matches.end());

	//将所有Surf粗匹配作为后续基础矩阵Ransac的输入， 
	//也可以将此部分参数化， 选择部分靠前粗匹配进行基础矩阵的Ransac输入。
	//考虑到有些时候检测到的正确匹配的数量占比较少，如果在此处cut掉一部分，
	//然而，该部分却有可能含有本来就为数不多的正确匹配，所以我在此处选取
	//所有的粗匹配。
	//int ptsPairs = std::min((int)(matches.size() / times), (int)matches.size());

	int ptsPairs = std::min((int)(tmp_matches.size()), (int)tmp_matches.size());
	for (int i = 0; i < ptsPairs; i++)
		matches.push_back(tmp_matches[i]);


	for (size_t i = 0; i < matches.size(); i++)
	{
		keypoints1.push_back(tmp_key1[matches[i].queryIdx]);
		keypoints2.push_back(tmp_key2[matches[i].trainIdx]);
	}
#ifndef NDEBUG
	cout << "=================================[1]=================================" << endl
		<< "粗选择部分总匹配点数为： " << tmp_matches.size() << endl;
#endif
	return matches.size();
}

// 关键点转换点云(利用深度图使关键点带深度信息，即z信息)
int keypoints2cloud(vector<KeyPoint>& keypoints1, pcl::PointCloud<pcl::PointXYZ>::Ptr& keycloud1, vector<KeyPoint>& keypoints2,
	pcl::PointCloud<pcl::PointXYZ>::Ptr& keycloud2)
{
	vector<Point2i> coor_setp1, coor_setp2; // 用于float转int
	Point2i p1, p2;
	for (size_t i = 0; i < keypoints1.size(); i++) {

		p1.x = int(keypoints1[i].pt.x + 0.5);
		p1.y = int(keypoints1[i].pt.y + 0.5);
		p2.x = int(keypoints2[i].pt.x + 0.5);
		p2.y = int(keypoints2[i].pt.y + 0.5);

		coor_setp1.push_back(p1);
		coor_setp2.push_back(p2);

		/*
		cout << i + 1 << "    "
			<< "(" << keypoints1[i].pt.x << ", " << keypoints1[i].pt.y << ")" << "    "
			<< "(" << keypoints2[i].pt.x << ", " << keypoints2[i].pt.y << ")" << "        "
			"(" << coor_setp1[i].x << ", " << coor_setp1[i].y << ")" << "    "
			<< "(" << coor_setp2[i].x << ", " << coor_setp2[i].y << ")" << endl;
		*/
	}

	Mat depth1 = cv::imread(imagelocation_depth1, -1);
	Mat depth2 = cv::imread(imagelocation_depth2, -1);

	if (depth1.empty() || depth2.empty()) {
		cout << "load image filed! Please check rgb image file location!\n\n";
		exit(-1);
	}

	pcl::PointXYZ p_1, p_2;
	for (int i = 0; i < coor_setp1.size(); i++) {

		ushort d_1 = depth1.ptr<ushort>(coor_setp1[i].y)[coor_setp1[i].x];
		ushort d_2 = depth2.ptr<ushort>(coor_setp2[i].y)[coor_setp2[i].x];
		// 剔除不存在深度信息的关键点
		if (d_1 == 0 || d_2 == 0)
			continue;

		p_1.z = double(d_1) / camera_factor;
		p_1.x = (coor_setp1[i].x - camera_cx) * p_1.z / camera_fx; // 计算视差x分量
		p_1.y = (coor_setp1[i].y - camera_cy) * p_1.z / camera_fy; // 计算视差y分量
		keycloud1->points.push_back(p_1);


		p_2.z = double(d_2) / camera_factor;
		p_2.x = (coor_setp2[i].x - camera_cx) * p_2.z / camera_fx; // 计算视差x分量
		p_2.y = (coor_setp2[i].y - camera_cy) * p_2.z / camera_fy; // 计算视差y分量
		keycloud2->points.push_back(p_2);
	}

	keycloud1->height = 1;
	keycloud1->width = keycloud1->points.size();
	keycloud1->is_dense = false; // 稀疏

	keycloud2->height = 1;
	keycloud2->width = keycloud2->points.size();
	keycloud2->is_dense = false;

	cout << "=================================[3]=================================" << endl
			<< "剔除不存在深度信息的匹配点，剔除后点云大小为: " << keycloud1->size() << endl;

	return keycloud1->size();
}

// 根据rgb图像和深度图生成带rgb信息的点云数据, 深度图用于生成位置信息，彩色图用于生成颜色信息
int rgb_depth2cloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& cloud, const string& rgb_filelocation, const string& imagelocation_depth)
{
	Mat rgb = imread(rgb_filelocation);
	Mat depth = cv::imread(imagelocation_depth, -1);
	if (rgb.empty() || depth.empty()) {
		cout << "load image filed! Please check rgb image file location!\n\n";
		exit(-1);
	}

	pcl::PointXYZRGBA p;
	for (int m = 0; m < depth.rows; m++) {
		for (int n = 0; n < depth.cols; n++) {
			ushort d = depth.ptr<ushort>(m)[n]; // opencv指针方式遍历像素
			if (d == 0)
				continue;
			p.z = double(d) / camera_factor;
			p.x = (n - camera_cx) * p.z / camera_fx; // 计算视差x分量
			p.y = (m - camera_cy) * p.z / camera_fy; // 计算视差y分量
			p.b = rgb.ptr<uchar>(m)[n * 3];
			p.g = rgb.ptr<uchar>(m)[n * 3 + 1];
			p.r = rgb.ptr<uchar>(m)[n * 3 + 2];
			cloud->points.push_back(p);
		}
	}

	cloud->height = 1;
	cloud->width = cloud->points.size();
	cloud->is_dense = false; // 稀疏点云

	return cloud->size();
}

// 误匹配特征点校正
int CorrespondenceRejectorSC(pcl::PointCloud<pcl::PointXYZ>::Ptr& keycloud1, pcl::PointCloud<pcl::PointXYZ>::Ptr& keycloud2,
	boost::shared_ptr<pcl::Correspondences>& Correspon_in, boost::shared_ptr<pcl::Correspondences>& Correspon_out)
{
	Correspon_out->clear();
	static int times = 0;
	boost::shared_ptr<pcl::Correspondence> corr1(new pcl::Correspondence);
	if (times == 0)
	{
		for (int i = 0; i < keycloud1->size(); i++)
		{
			corr1->index_query = i;
			corr1->index_match = i;
			Correspon_in->push_back(*corr1);
		}
	}

	//cout << "进行三维RANC之前的关键点数为: " << Correspon_in->size() << endl;

	pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZ>::Ptr
		ransac_rejector(new pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZ>);
	ransac_rejector->setInputSource(keycloud1);
	ransac_rejector->setInputTarget(keycloud2);
	ransac_rejector->getRemainingCorrespondences(*Correspon_in, *Correspon_out);
	Correspon_in->clear();
	cout << "=================================[4]=================================" << endl
				<< "进行第" << ++times << "次三维Ransac之后的关键点数为: " << Correspon_out->size() << endl;
	return Correspon_out->size();
}

void Transform_use_RTmatrixandSave(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& sourcecloud, Eigen::Matrix4f& Transform,
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& transformed_cloud)
{
	pcl::transformPointCloud(*sourcecloud, *transformed_cloud, Transform); // 透视变换
	transformed_cloud->height = 1;
	transformed_cloud->width = transformed_cloud->points.size();
	transformed_cloud->is_dense = false; // 稀疏点云
	pcl::io::savePLYFile(outputfilename, *transformed_cloud);
	cout << endl << "saved transformedcloud as :" << outputfilename << endl;

}

// 在可视化工具中添加两个点云数据用于显示
boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis(pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr transformed_sourcecloud1,
	pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr sourcecloud2)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0); //设置背景色
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> color(transformed_sourcecloud1); //点云颜色(保持点云的rgb值)
	// pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> color(transformed_sourcecloud1, 0, 255, 0); //点云颜色（单一颜色设置）
	viewer->addPointCloud<pcl::PointXYZRGBA>(transformed_sourcecloud1, color, "transformed_sourcecloud1"); //添加点云
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "transformed_sourcecloud1"); //设置显示点的大小
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> color2(sourcecloud2); //点云颜色(保持点云的rgb值)
	viewer->addPointCloud<pcl::PointXYZRGBA>(sourcecloud2, color2, "sourcecloud2"); //添加点云
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sourcecloud2"); //设置显示点的大小
	viewer->initCameraParameters();

	return viewer;
}