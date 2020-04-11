#include <iostream>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

void printUsage(const char* progName)
{
	std::cout << "\n\nUsage: " << progName << " [options]\n\n"
		<< "Options:\n"
		<< "-------------------------------------------\n"
		<< "-h           this help\n"
		<< "-s           Simple visualisation example\n"
		<< "-r           RGB colour visualisation example\n"
		<< "-c           Custom colour visualisation example\n"
		<< "-n           Normals visualisation example\n"
		<< "-a           Shapes visualisation example\n"
		<< "-v           Viewports example\n"
		<< "-i           Interaction Customization example\n"
		<< "\n\n";
}

/**���ӻ��������ƣ�Ӧ��PCL Visualizer���ӻ�����ʾ��������XYZ��Ϣ�ĵ���*/
//simpleVis����ʵ��������ĵ��ƿ��ӻ�������
boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{  //Open 3D viewer and add point cloud //�����Ӵ����󲢸�����������һ�����ơ�3D Viewer������������Ϊboost::shared_ptr���ܹ���ָ�룬�������Ա�ָ֤���ڳ�����ȫ��ʹ�ã����������ڴ����
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));   //�����Ӵ��ı���ɫ��������������RGB����ɫ������������Ϊ��ɫ
	viewer->setBackgroundColor(0, 0, 0);
	/*��������Ҫ��һ�У����ǽ�������ӵ��Ӵ������У�����һ��Ψһ���ַ�����ΪID �ţ����ô��ַ�����֤��������Ա��Ҳ�ܱ�־���øõ��ƣ���ε���addPointCloud����ʵ�ֶ�����Ƶ���ӣ���ÿ����һ�ξͻᴴ��һ���µ�ID�ţ���������һ���Ѿ���ʾ�ĵ��ƣ������ȵ�removePointCloud���������ṩ��Ҫ���µĵ���ID ��*/
	viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud");
	//���ڸı���ʾ���Ƶĳߴ磬�������ø÷������Ƶ������Ӵ��е���ʾ������
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");/*
	 �鿴���ӵĵ��ƣ��������˸е�û�з���У�Ϊ�˱�����ȷ�������жϣ���Ҫ��ʾ����ϵͳ���򣬿���ͨ��ʹ��X����ɫ�� Y����ɫ ��Z ����ɫ��Բ����������������ʾ��ʽ�������Բ����Ĵ�С����ͨ��scale���������ƣ�������scale����Ϊ1.0*/
	viewer->addCoordinateSystem(1.0); //ͨ���������������ʹ�ô�Ĭ�ϵĽǶȺͷ���۲����
	viewer->initCameraParameters();  return (viewer);
}/*���ӻ�������ɫ����*��������µ�����ʾ�����ü򵥵�XYZ���ͣ����õĵ���������XYZRGB�㣬������ɫ���ݣ�����֮�⣬�����Ը�ָ���ĵ��ƶ�����ɫ��ʾ�õ������Ӵ��бȽ��������֡��㸳�費ͬ����ɫ�������Ӧ��Z��ֵ��ͬ��PCL Visualizer�ɸ������洢����ɫ����Ϊ����
��ɫ�� ��������豸kinect���Ի�ȡ����RGB���ݵĵ��ƣ�PCL Vizualizer���ӻ����ʹ��������ɫ����Ϊ������ɫ��rgbVis�����еĴ�������������ֲ�����*/
/*��ǰ���ʾ����ȵ��Ƶ����ͷ����˱仯������ʹ�õĵ��ƴ���RGB���ݵ������ֶΣ�*/
boost::shared_ptr<pcl::visualization::PCLVisualizer> rgbVis(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{  //Open 3D viewer and add point cloud // 
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	/*���ô��ڵı�����ɫ�󣬴���һ����ɫ�������PointCloudColorHandlerRGBField���������Ķ�����ʾ�Զ�����ɫ���ݣ�PointCloudColorHandlerRGBField ����õ�ÿ�����Ƶ�RGB��ɫ�ֶΣ�*/

	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
	viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "sample cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();  return (viewer);
}/*���ӻ������Զ�����ɫ����*/
/*��ʾ�������������ϵ�����һ����ɫ���������øü�����ָ���ĵ�����ɫ�������������ĵ��ƣ�*/
//��������ΪXYZ���ͣ�customColourVis���������Ƹ�ֵΪ��ɫ��
boost::shared_ptr<pcl::visualization::PCLVisualizer> customColourVis(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{  //Open 3D viewer and add point cloud-----  //
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);  //����һ���Զ������ɫ������PointCloudColorHandlerCustom���󣬲�������ɫΪ����ɫ
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, 0, 255, 0);  //addPointCloud<>()��ɶ���ɫ����������Ĵ���
	viewer->addPointCloud<pcl::PointXYZ>(cloud, single_color, "sample cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();  return (viewer);
}//���ӻ����Ʒ��ߺ���������*/
/* ��ʾ�����������Ƶ�һ����Ҫ���裬���Ʒ��������Ƿǳ���Ҫ�Ļ���������PCL visualizer���ӻ�������ڻ��Ʒ��ߣ�Ҳ���Ի��Ʊ������Ƶ��������������������ʺͼ���������normalsVis��������ʾ�����ʵ�ֵ��Ƶķ��ߣ�*/
boost::shared_ptr<pcl::visualization::PCLVisualizer> normalsVis(
	pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, pcl::PointCloud<pcl::Normal>::ConstPtr normals)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
	viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "sample cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");  //ʵ�ֶԵ��Ʒ��ߵ���ʾ
	viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(cloud, normals, 10, 0.05, "normals");
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();  return (viewer);
}  //*****************������ͨ��״************************************************// 

/**************************************************************************************************************
PCL visualizer���ӻ��������û����Ӵ��л���һ��ͼԪ������ೣ������ʾ���ƴ����㷨�Ŀ��ӻ���������� ͨ�����ӻ�����
��Χ����õ��ĵ��Ƽ�����ʾ��������shapesVis��������ʵ�������״���Ӵ��У������������״���ӵ����е�һ���㵽���һ����
֮������ߣ�ԭ�����ڵ�ƽ�棬�Ե����е�һ����Ϊ���ĵ����壬��Y���׵��*/
boost::shared_ptr<pcl::visualization::PCLVisualizer> shapesVis(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
	viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "sample cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();
	/*������״��ʵ�����룬���Ƶ�֮������ߣ�*/
	viewer->addLine<pcl::PointXYZRGB>(cloud->points[0],
		cloud->points[cloud->size() - 1], "line");  //��ӵ����е�һ����Ϊ���ģ��뾶Ϊ0.2�����壬ͬʱ�����Զ�����ɫ
	viewer->addSphere(cloud->points[0], 0.2, 0.5, 0.5, 0.0, "sphere");   pcl::ModelCoefficients coeffs;
	coeffs.values.push_back(0.0);
	coeffs.values.push_back(0.0);
	coeffs.values.push_back(1.0);
	coeffs.values.push_back(0.0);
	viewer->addPlane(coeffs, "plane");  //���׶�εĲ���  coeffs.values.clear ();
	coeffs.values.push_back(0.3);
	coeffs.values.push_back(0.3);
	coeffs.values.push_back(0.0);
	coeffs.values.push_back(0.0);
	coeffs.values.push_back(1.0);
	coeffs.values.push_back(0.0);
	coeffs.values.push_back(5.0);
	viewer->addCone(coeffs, "cone");  return (viewer);
}

/*
���ӽ���ʾ��PCL  visealizer���ӻ��������û�ͨ����ͬ�Ĵ��ڣ�Viewport�����ƶ��������������Ե��ƱȽ�
viewportsVis������ʾ����ö��ӽ�����ʾ���Ƽ��㷨�ߵķ�������Ա�*/
boost::shared_ptr<pcl::visualization::PCLVisualizer> viewportsVis(
	pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, pcl::PointCloud<pcl::Normal>::ConstPtr normals1, pcl::PointCloud<pcl::Normal>::ConstPtr normals2)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->initCameraParameters();   //�����Ǵ�����ͼ�ı�׼����
	int v1(0);  //�����µ��ӿ�
	viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);  //4�������ֱ���X�����Сֵ�����ֵ��Y�����Сֵ�����ֵ��ȡֵ0-1��v1�Ǳ�ʶ
	viewer->setBackgroundColor(0, 0, 0, v1);    //�����ӿڵı�����ɫ
	viewer->addText("Radius: 0.01", 10, 10, "v1 text", v1);  //���һ����ǩ������������  ����RGB��ɫ��ɫ������ӵ��Ƶ��ӿ���
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
	viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "sample cloud1", v1);   //�Եڶ��ӿ���ͬ���Ĳ�����ʹ���������ĵ��Ʒֲ����Ұ봰�ڣ������ӿڱ�����ֵ�ڻ�ɫ���Ա�����������Ȼ���ͬ���ĵ��ƣ��������Զ�����ɫ��ɫ
	int v2(0);
	viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
	viewer->setBackgroundColor(0.3, 0.3, 0.3, v2);
	viewer->addText("Radius: 0.1", 10, 10, "v2 text", v2);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> single_color(cloud, 0, 255, 0);
	viewer->addPointCloud<pcl::PointXYZRGB>(cloud, single_color, "sample cloud2", v2);  //Ϊ�����ӿ��������ԣ�
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud1");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud2");
	viewer->addCoordinateSystem(1.0);  //��ӷ���  ÿ����ͼ����һ���Ӧ�ķ���
	viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(cloud, normals1, 10, 0.05, "normals1", v1);
	viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(cloud, normals2, 10, 0.05, "normals2", v2);  return (viewer);
}

/*�����Ǵ�������¼��ĺ�����ÿ����Ӧ���ʱ�䶼��ص纯������Ҫ��eventʵ����ȡ�¼���Ϣ�������в������������ͷ��¼�
ÿ����Ӧ�����¼���������갴�µ�λ��������һ���ı���ǩ��*/
unsigned int text_id = 0; void keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event, void* viewer_void)
{
	pcl::visualization::PCLVisualizer* viewer = static_cast<pcl::visualization::PCLVisualizer*> (viewer_void);  if (event.getKeySym() == "r" && event.keyDown())
	{
		std::cout << "r was pressed => removing all text" << std::endl;    char str[512];    for (unsigned int i = 0; i < text_id; ++i)
		{
			sprintf(str, "text#%03d", i);
			viewer->removeShape(str);
		}
		text_id = 0;
	}
}

/* �����¼� ���ǰ����ĸ�����  �������r��   ��ɾ��ǰ��������������ı���ǩ����Ҫע����ǣ�������R��ʱ 3D�����Ȼ������
 ������PCL���Ӵ���ע���¼���Ӧ�ص����������Ḳ��������Ա��ͬһ�¼�����Ӧ*/
void mouseEventOccurred(const pcl::visualization::MouseEvent& event, void* viewer_void)
{
	pcl::visualization::PCLVisualizer* viewer = static_cast<pcl::visualization::PCLVisualizer*> (viewer_void);  if (event.getButton() == pcl::visualization::MouseEvent::LeftButton && event.getType() == pcl::visualization::MouseEvent::MouseButtonRelease)
	{
		std::cout << "Left mouse button released at position (" << event.getX() << ", " << event.getY() << ")" << std::endl;    char str[512];
		sprintf(str, "text#%03d", text_id++);
		viewer->addText("clicked here", event.getX(), event.getY(), str);
	}
}

/*��������£�Ĭ�ϵ����ͼ��̽������ò��������û��������û�����չ������ĳһЩ���ܣ�  ���簴�¼���ʱ������Ƶ���Ϣ������ͨ�����ȷ�����Ƶ�λ��   interactionCustomizationVis����������ʾ��β�׽���ͼ����¼����ڴ��ڵ����������ʾһ��2D���ı���ǩ������r����ȥ�ı�*/
boost::shared_ptr<pcl::visualization::PCLVisualizer> interactionCustomizationVis()
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);  //������ʵ�����Ӵ��ı�׼����
	viewer->addCoordinateSystem(1.0);  //�ֱ�ע����Ӧ���̺�����¼���keyboardEventOccurred  mouseEventOccurred�ص���������Ҫ��boost::shared_ptrǿ��ת��Ϊvoid*
	viewer->registerKeyboardCallback(keyboardEventOccurred, (void*)viewer.get());
	viewer->registerMouseCallback(mouseEventOccurred, (void*)viewer.get());
	return (viewer);
}

int main(int argc, char** argv)
{
	if (pcl::console::find_argument(argc, argv, "-h") >= 0)
	{
		printUsage(argv[0]);    return 0;
	}
	
	bool simple(false), rgb(false), custom_c(false), normals(false),
		shapes(false), viewports(false), interaction_customization(false);  if (pcl::console::find_argument(argc, argv, "-s") >= 0)
	{
		simple = true;
		std::cout << "Simple visualisation example\n";
	}
		else if (pcl::console::find_argument(argc, argv, "-c") >= 0)
	{
		custom_c = true;
		std::cout << "Custom colour visualisation example\n";
	}
		else if (pcl::console::find_argument(argc, argv, "-r") >= 0)
	{
		rgb = true;
		std::cout << "RGB colour visualisation example\n";
	}
		else if (pcl::console::find_argument(argc, argv, "-n") >= 0)
	{
		normals = true;
		std::cout << "Normals visualisation example\n";
	}
		else if (pcl::console::find_argument(argc, argv, "-a") >= 0)
	{
		shapes = true;
		std::cout << "Shapes visualisation example\n";
	}
		else if (pcl::console::find_argument(argc, argv, "-v") >= 0)
	{
		viewports = true;
		std::cout << "Viewports example\n";
	}
		else if (pcl::console::find_argument(argc, argv, "-i") >= 0)
	{
		interaction_customization = true;
		std::cout << "Interaction Customization example\n";
	}
		else
	{
		printUsage(argv[0]);    return 0;
	}
	pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
	std::cout << "Genarating example point clouds.\n\n";
	uint8_t r(255), g(15), b(15);  for (float z(-1.0); z <= 1.0; z += 0.05)
	{
		for (float angle(0.0); angle <= 360.0; angle += 5.0)
		{
			pcl::PointXYZ basic_point;
			basic_point.x = 0.5 * cosf(pcl::deg2rad(angle));
			basic_point.y = sinf(pcl::deg2rad(angle));
			basic_point.z = z;
			basic_cloud_ptr->points.push_back(basic_point);     pcl::PointXYZRGB point;
			point.x = basic_point.x;
			point.y = basic_point.y;
			point.z = basic_point.z;
			uint32_t rgb = (static_cast<uint32_t>(r) << 16 |
				static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
			point.rgb = *reinterpret_cast<float*>(&rgb);
			point_cloud_ptr->points.push_back(point);
		}    if (z < 0.0)
		{
			r -= 12;
			g += 12;
		}
		else
		{
			g -= 12;
			b += 12;
		}
	}
	basic_cloud_ptr->width = (int)basic_cloud_ptr->points.size();
	basic_cloud_ptr->height = 1;
	point_cloud_ptr->width = (int)point_cloud_ptr->points.size();
	point_cloud_ptr->height = 1;
	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
	ne.setInputCloud(point_cloud_ptr);
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
	ne.setSearchMethod(tree);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals1(new pcl::PointCloud<pcl::Normal>);
	ne.setRadiusSearch(0.05);
	ne.compute(*cloud_normals1);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2(new pcl::PointCloud<pcl::Normal>);
	ne.setRadiusSearch(0.1);
	ne.compute(*cloud_normals2); boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;  if (simple)
	{
		viewer = simpleVis(basic_cloud_ptr);
	}
	else if (rgb)
	{
		viewer = rgbVis(point_cloud_ptr);
	}
	else if (custom_c)
	{
		viewer = customColourVis(basic_cloud_ptr);
	}
	else if (normals)
	{
		viewer = normalsVis(point_cloud_ptr, cloud_normals2);
	}
	else if (shapes)
	{
		viewer = shapesVis(point_cloud_ptr);
	}
	else if (viewports)
	{
		viewer = viewportsVis(point_cloud_ptr, cloud_normals1, cloud_normals2);
	}
	else if (interaction_customization)
	{
		viewer = interactionCustomizationVis();
	}
	//--------------------
	// -----Main loop-----
	//--------------------
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
}