#include <pcl/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/console/parse.h>
#include <pcl/io/io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>  //体素滤波相关

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <ctime>
#include <math.h>

#define PI 3.14159265

typedef struct tagPOINT_3D
{
	double x;  //mm world coordinate x  
	double y;  //mm world coordinate y  
	double z;  //mm world coordinate z  
	double r;
}POINT_WORLD;

class Node{
/*
 * Contains a point in point cloud and stores neighbor nodes in a pointer
 * array. Various boolean values mark whether the node instance has been 
 * checked (in calculate_hole). Finally, it holds a value to indicate 
 * the probability it is a boundary point of a hole
 */
public:
	float x;
	float y;
	float z;
	int max_angle;  // 0 - 360
	int n_count;
	int id; // not used right now, but assigning an id to each node may be helpful in the future
	bool checked;
	bool boundary;
	bool possible_boundary;
	Node(pcl::PointXYZ ptr, int id_number);
	Node* neighbors[10];
	void print_neighbors();
	void add_neighbor(Node* neighbor);
};

Node::Node(pcl::PointXYZ point, int id_number)
{ 
	n_count = 0;
	x = point.x;
	y = point.y;
	z = point.z;
	max_angle = 0;
	possible_boundary = false;
	boundary = false;
	checked = false;
	id = id_number;
}

// adds pointer to neighbor to neighbor array
void Node::add_neighbor(Node* neighbor)
{
	neighbors[n_count++] = neighbor;
}

// prints out x,y,z for all neighbors
void Node::print_neighbors()
{
	for(int i = 0; i < n_count; i++)
	{
		std::cout<<neighbors[i]->x<<", "<<neighbors[i]->y<<", "<<neighbors[i]->z<<std::endl;
	}
}

// prints help menu
void showHelp()
{
	std::cout << std::endl;
	std::cout << "Commands:" << std::endl;
	std::cout << "\nhelp:	Show this help." << std::endl;
	std::cout << "tree(t):	Runs kd tree algorithm with specified K number of closest points and/or with a specified radius." << std::endl;
	std::cout << "quit(q):	Exits program." << std::endl;	
	std::cout << "points(p):	Prints points contained in point cloud." << std::endl;
	std::cout << "normals(n):	Prints normals of points in cloud." << std::endl;
	std::cout << "visualize(v):	Displays point cloud of .pcd file that has been processed in calculate_hole" << std::endl;
	std::cout << "holes(h):	Calculates hole (returns set of points on boundary and visualizes image with boundary points red).\n" << std::endl;
}

// prints out all points in PCL dataset
void points(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
	for(unsigned i = 0; i < cloud->points.size(); i++){
		std::cout<<cloud->points[i]<<std::endl;
	}
}


// finds the normals of each point and prints it to standard out
// 计算每一个点的法线，并将其输出到校准输出
void normals(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
	// Create the normal estimation class, and pass the input dataset to it
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;	

	ne.setInputCloud (cloud);

	// Create an empty kdtree representation, and pass it to the normal estimation object.
	// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree3 (new pcl::search::KdTree<pcl::PointXYZ> ());

	ne.setSearchMethod (tree3);

	// Output datasets
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

	// Use all neighbors in a sphere of radius 3cm
	ne.setRadiusSearch (5);

	// Compute the features
	ne.compute (*cloud_normals);

	std::cout << "size of the points " << cloud->points.size() << std::endl ; 		
	std::cout << "size of the normals " << cloud_normals->points.size() << std::endl ; 		
	/*
	for(unsigned i = 0; i < cloud_normals->points.size(); i++){
		std::cout<< cloud_normals->points[i] <<std::endl;
	}
	*/
}

// finds angle between two three dimensional vectors
float angle_between_vectors (float *nu, float *nv)
{
	// if the cross product is negative the angle was measured clockwise and has
	// to be substracted from zero
	float signed_value = (nu[1]*nv[2]-nu[2]*nv[1]) + (nu[2]*nv[0]-nu[0]*nv[2]) + (nu[0]*nv[2]-nu[1]*nv[0]);
	
	// find angle using dot product
	float l1 = sqrt(nu[0]*nu[0] + nu[1]*nu[1] + nu[2]*nu[2]);
	float l2 = sqrt(nv[0]*nv[0] + nv[1]*nv[1] + nv[2]*nv[2]);
	float dot = nu[0]*nv[0] + nu[1]*nv[1] + nu[2]*nv[2];
	float param = dot/(l1*l2);
	float angle = std::acos(param);
	angle = angle*180/PI;
	angle = floor(angle*100 + 0.5)/100 ;  // round off to two decimal places

	if(signed_value < 0){
		signed_value = -1;
		angle = 360-angle;
	}
	else{
		signed_value = 1;
	}

	return angle ;
}

// constructs a kd tree with a k number of closest points or a radius of a
// boundary that encloses all points or both
void kd_tree(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, int x=1, int y=1, int z=1)
{
	bool r = false;
	bool k = false;
	std::cout<<"Would you like a radius(r) search, a K(k) search or both(b): "<<std::endl;
	char input;
	while(true){
		std::cin >> input;	
		if(input == 'k'){
			k = true;
			break;
		}
		else if(input == 'r'){
			r = true;
			break;
		}
		else if(input == 'b'){
			k = true;
			r = true;
			break;
		}	
		else{
			std::cout<<"option '"<<input<<"' doesn't exist"<<std::endl;
		}
	}

	srand (time (NULL));

	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;

	kdtree.setInputCloud (cloud);

	pcl::PointXYZ searchPoint;

	searchPoint.x = x;
	searchPoint.y = y;
	searchPoint.z = z;

	if(k){
		std::cout<< "Enter a K value: " <<std::endl;
		int K = 0;
		std::cin >> K;

		// K nearest neighbor search
		std::vector<int> pointIdxNKNSearch(K);
		std::vector<float> pointNKNSquaredDistance(K);

		std::cout << "K nearest neighbor search at (" << searchPoint.x 
			<< ", " << searchPoint.y 
			<< ", " << searchPoint.z
			<< ") with K=" << K << std::endl;

		std::ofstream new_file;
		new_file.open ("new_file.pcd", std::ios_base::app);
    
		if ( kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 ){
			for (size_t i = 0; i < pointIdxNKNSearch.size (); ++i){
				std::cout << "    "  <<   cloud->points[ pointIdxNKNSearch[i] ].x 
					<< ", " << cloud->points[ pointIdxNKNSearch[i] ].y 
					<< ", " << cloud->points[ pointIdxNKNSearch[i] ].z 
					<< " (squared distance: " << pointNKNSquaredDistance[i] << ")" << std::endl; 
				new_file << "    "  <<   cloud->points[ pointIdxNKNSearch[i] ].x 
					<< ", " << cloud->points[ pointIdxNKNSearch[i] ].y 
					<< ", " << cloud->points[ pointIdxNKNSearch[i] ].z 
					<< " (squared distance: " << pointNKNSquaredDistance[i] << ")" << std::endl;
      
			}
		}
		new_file.close();
	}

	if(r){
		std::cout<< "Enter a radius: " <<std::endl;
		float radius = 0;
		std::cin >> radius;

		// Neighbors within radius search
		std::vector<int> pointIdxRadiusSearch;
		std::vector<float> pointRadiusSquaredDistance;

		std::cout << "Neighbors within radius search at (" << searchPoint.x 
			<< ", " << searchPoint.y 
			<< ", " << searchPoint.z
			<< ") with radius=" << radius << std::endl;

		if ( kdtree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 ){
			for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i){
				std::cout << "    "  <<   cloud->points[ pointIdxRadiusSearch[i] ].x 
					<< ", " << cloud->points[ pointIdxRadiusSearch[i] ].y 
					<< ", " << cloud->points[ pointIdxRadiusSearch[i] ].z 
					<< " (squared distance: " << pointRadiusSquaredDistance[i] << ")" << std::endl;
			}
			std::cout << "points: " << pointIdxRadiusSearch.size() << std::endl;
		}
	}
}

// compares nodes (i.e. the points they contain)
bool is_equal(Node* first, Node* second)
{
	return first->x == second->x && first->y == second->y && first->z == second->z;
}

// Recursive function that searches for boundary points
bool is_boundary_point(Node* before, Node* node)
{
	// generally not necessary, but a safecheck for a point being checked without a
	// parent node that wouldn't've checked its maximum probability before it called
	// this function
  
	if(node->max_angle < 120){
		return false;
	}

	// if we come across a node that was already checked we can evaluate whether
	// we have encircled a hole or we have hit a dead end
	if(node->checked){
		if(node->possible_boundary || node->boundary){
			return true;
		}
  
		if(!node->possible_boundary){
			return false;
		}
	}

	// point is now checked and a possible boundary point
	node->possible_boundary = true;
	node->checked = true;

	// array for indexes of possible boundary points of the current node we are
	// examining
	int * possible_nodes = new int[node->n_count]; // slightly inefficient, but we don't know how many will be boundary points
	int count = 0;
	int boundary_count = 0;

	for(int i = 0; i < node->n_count; i++){
		// if this point was called from calculate_hole and not recursively by this
		// method itself, then we have to find two neighbor nodes that are boundary
		// points in order for this to be a boundary point
		if(before == 0 || !is_equal(before, node->neighbors[i])){ // 0 is passed from calculate_hole, otherwise before will be pointer node calling function
			if(node->neighbors[i]->boundary){
				if(before == 0){
					boundary_count++;
				}
				else{
					// if a bound point is encountered a hole must have been found
					return true;
				}

				if(boundary_count > 1){
					return true;
				}
			}
			if(node->neighbors[i]->possible_boundary){
				// if this function was called recursively then a circle (hole) has
				// been iterated and you can return true so all nodes become boundary
				// points
				return true;
			}
      
			// add to array of possible boundary points if it has a max neighbor angle
			// of greater than 120
			if(node->neighbors[i]->max_angle > 120){
				possible_nodes[count] = i;
				count++;
			}
		}
	}
  
	if(count > 1){
		for(int j = 0; j < count; j++){
			// check each possible point with recursive function, if true is returned
			// that means a hole was iterated (found)
			if(is_boundary_point(node, node->neighbors[possible_nodes[j]])){
				return true;
			}
		}
		// if we get here then for future reference mark this point as not a
		// candidate for a boundary point
		node->possible_boundary = false;
		return false;
	}

	delete [] possible_nodes;
  
	// same as above
	node->possible_boundary = false;

	return false;
}

void down_sample(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
	cout << "there are " << cloud->points.size() << " points before down sample." << endl;

	//体素栅格滤波
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filter(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud);
	sor.setLeafSize(3.0f, 3.0f, 3.0f);//体素大小设置为3*3*3m
	sor.filter(*cloud_filter);

	//滤波结果保存
	//pcl::io::savePCDFile<pcl::PointXYZ>("filter.pcd", *cloud_filter);

	cloud = cloud_filter;
	cout << "there are " << cloud->points.size() << " points after down sample." << endl;
}

/****************K POINTS & RADIUS POINTS*****************************/
// k-points & radius-points
void calculate_hole(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
	// 下采样
	down_sample(cloud);

	// creating tree and linking cloud of points to it
	// 创建kd树
	srand (time (NULL));
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud (cloud);

	// Number of possible nearest points found by each search.
	// That is we search for 100 closest points to each points in cloud until we
	// find 10 neighbors (or we iterate through entire cloud).
	// for each neighbor in question we search their 10 closest neighbors too see
	// if the original point is in that set
	// 每次搜索可能找到的最近点的数目。
	// 也就是说，我们在点云中搜索离每个点最近的100个点，找到10个近邻点(或者遍历整个云)。
	// 对于每个有问题的邻居，我们也搜索它们的10个最近的邻居，看看初始点是否在那个集合中
	int outer_k = 100;
	int inner_k = 10;

	// K nearest neighbor search
	// K近邻搜索
	std::vector<int> k_search(outer_k);
	std::vector<float> squared_dist(outer_k);
	std::vector<int> inner_k_search(inner_k);
	std::vector<float> inner_squared_dist(inner_k);
  
	// array of pointers to nodes we will create for each point in cloud
	// 为每个点创建节点Node，将所有的节点存放到vector中
	Node ** node_points = new Node*[cloud->points.size()];
	// int ** node_points = new int*[cloud->points.size()];
	// std::vector<Node> node_points(cloud->points.size);

	for(int h = 0; h < cloud->points.size(); h++){
		node_points[h] = new Node(cloud->points[h], h);
	}

	int cnt = 0;
	// iterate through cloud.
	// 遍历整个点云
	for(int j = 0; j < cloud->points.size(); j++) {
		Node* current = node_points[j];
    
		// finds and stores outer_k closest point and stored their indexes
		// into the point cloud in an array k_search
		// 查找和存储outer_k最近的点，并将它们的索引存储到k_search数组中的点云中
		if ( kdtree.nearestKSearch (cloud->points[j], outer_k, k_search, squared_dist) > 0 )
		{
			// iterate this until we find 10 viable neighbors or run out of points
			// 重复这个步骤，直到找到10个可行的近邻点，直到遍历完所有的点
			for (int i = 0; i < k_search.size() && node_points[j]->n_count < 10; i++){
				bool contains_point = false;
				Node* neighbor = node_points[k_search[i]];
        
				if( kdtree.nearestKSearch(cloud->points[k_search[i]], inner_k , inner_k_search, inner_squared_dist) > 0 && !is_equal(current, neighbor)){
					// if kdtree returns something and if the neighbor isn't actually just our current point
					// iterate through neighbor's neighbors
					// 如果kdtree未搜索到正确的近邻点，那么它会遍历邻居的邻居
					for (int k = 0; k < inner_k_search.size(); k++){
						Node* neigh_bound = node_points[inner_k_search[k]];

						// if possible neighbor point contains the point in question
						// (current) in its own neighborhood then it is in fact a neighbor
						// point to current
						// 如果可能的近邻点的邻居点中包含问题点，那么它就是当前点的近邻点
						if (is_equal(current, neigh_bound)){
							contains_point = true;
						} 
					}
				}
				if(contains_point){
					// add to currents neighborhood
					// 找到了近邻点
					current->add_neighbor(neighbor);
				}
			}
			float v1[3];
			float v2[3];
			float max = 0;
			float angle = 0;
			Node* vertex = node_points[j]; // main point 主点
			Node* start = vertex->neighbors[0]; // first point in neighborhood 初始点
			Node* next = vertex->neighbors[1]; // next point 下一个点

			// initial vector from point to first neighbor
			// 点到第一个近邻点的初始向量
			v1[0] = start->x - vertex->x;
			v1[1] = start->y - vertex->y;
			v1[2] = start->z - vertex->z;

			// angles between vectors
			// float angles[vertex->n_count];
			// 向量夹角是float类型
			std::vector<float> angles(vertex->n_count);
			int a_index = 0;

			// iterating over neighbor points
			// 在近邻点中搜索
			for(int i = 0; i < vertex->n_count-1; i++){
				next = vertex->neighbors[i+1];
				v2[0] = next->x - vertex->x;
				v2[1] = next->y - vertex->y;
				v2[2] = next->z - vertex->z;
				angle = angle_between_vectors(v1,v2);
				angles[a_index] = angle;
        
				// sort angles as you add them to array
				// 夹角排序并添加到数组
				for(int k = a_index; k-1 >=0; k--){
					if(angles[k] < angles[k-1]){
						int temp = angles[k];
						angles[k] = angles[k-1];
						angles[k-1] = temp;
					}
				}
				a_index++; 

				//last angle is 360 technically because we calculate difference in
				//angles
				// 最后一个角是360度因为我们计算了角度差
				angles[a_index] = 360;
			}
			float difference;
			max = angles[0];

			// find angles between vectors since they are in order already
			// 求有序向量之间的夹角
			for(int i = 0; i < a_index; i++){
				difference = angles[i+1] - angles[i];
				if(difference > max){
					max = difference;
				}
			}
			vertex->max_angle = max;
		}
		//std::cout<<"\n"<<cloud->points[j]<<"\n"<<std::endl;
		//node_points[j]->print_neighbors();
		//std::cout<<"\n" <<std::endl;

		/*
		if (0 == ++cnt % 2) {
			std::cout<< cnt << " ";
		}
		*/
	}

	// Iterating through points to see which ones satisfy the angle criterion and have two neighboring 
	// neighbor points
	// 遍历各个点，看看哪些点满足角度判据，并且有两个近邻点
	// 遍历点云，统计边界点
	int count = 0;

	for(int j = 0; j < cloud->points.size(); j++){
		node_points[j]->boundary = is_boundary_point(0, node_points[j]);
		std::ofstream new_file;
		new_file.open ("boundary.txt", std::ios_base::app);
		//std::cout<<"boundary point: "<<node_points[j]->boundary<<std::endl;
		if(node_points[j]->boundary){
			new_file << node_points[j]->x<<" "<<node_points[j]->y<<" "<<node_points[j]->z<<std::endl;
			count++;
		}
		new_file.close();
	}

	// 保存边缘点
	int number_Txt;
	FILE* fp_txt;
	tagPOINT_3D TxtPoint;
	std::vector<tagPOINT_3D> m_vTxtPoints;
	fp_txt = fopen("boundary.txt", "r");
	if (fp_txt)
	{
		while (fscanf(fp_txt, "%lf %lf %lf", &TxtPoint.x, &TxtPoint.y, &TxtPoint.z) != EOF)
		{
			m_vTxtPoints.push_back(TxtPoint);
		}
	} else {
		cout << "boundary.txt加载失败！" << endl;
	}

	number_Txt = m_vTxtPoints.size();
	//pcl::PointCloud<pcl::PointXYZ> cloud;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_boundary(new pcl::PointCloud<pcl::PointXYZ>);
	// Fill in the cloud data  
	cloud_boundary->width = number_Txt;
	cloud_boundary->height = 1;
	cloud_boundary->is_dense = false;
	cloud_boundary->points.resize(cloud_boundary->width * cloud_boundary->height);
	for (size_t i = 0; i < cloud_boundary->points.size(); ++i)
	{
		cloud_boundary->points[i].x = m_vTxtPoints[i].x;
		cloud_boundary->points[i].y = m_vTxtPoints[i].y;
		cloud_boundary->points[i].z = m_vTxtPoints[i].z;
	}
	pcl::io::savePCDFileASCII("boundary.pcd", *cloud_boundary);

	std::cerr << "Saved " << cloud_boundary->points.size() << " data points to boundary.pcd." << std::endl;

	std::cout<<"count "<<count<<std::endl;

	// Viewer
	pcl::visualization::PCLVisualizer viewer("Cloud Viewer");
	viewer.addPointCloud(cloud_boundary);
	viewer.setBackgroundColor(1, 0.5, 1);

	viewer.spin();

}

//function needed for viewing pcl
void viewerOneOff (pcl::visualization::PCLVisualizer& viewer)
{
    viewer.setBackgroundColor (1.0, 0.5, 1.0);
    /*pcl::PointXYZ o;
    o.x = 1.0;
    o.y = 0;
    o.z = 0;
    viewer.addSphere (o, 0.25, "sphere", 0);
    std::cout << "i only run once" << std::endl;*/
    
}

// launches graphical interface displaying point cloud in new_file.pcd
// 启动图形界面显示点云new_file.pcd
void visualize(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
	/*
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>); 
	pcl::io::loadPCDFile ("new_file.pcd", *cloud); 
	*/

	pcl::visualization::CloudViewer viewer("Cloud Viewer"); 

	//blocks until the cloud is actually rendered 
	viewer.showCloud(cloud);   
	//use the following functions to get access to the underlying more advanced/powerful 
	//PCLVisualizer 

	//This will only get called once 
	viewer.runOnVisualizationThreadOnce (viewerOneOff); 
	std::cout << "count of points: " << cloud->points.size() << std::endl;

	//This will get called once per visualization iteration 
	//viewer.runOnVisualizationThread (viewerPsycho); 
	while (!viewer.wasStopped ()) 
	{ 
		//you can also do cool processing here 
		//FIXME: Note that this is running in a separate thread from viewerPsycho 
		//and you should guard against race conditions yourself... 
	}
}

/*********************************************************************************************/

int main(int argc, char** argv)
{
	/*
	// Fetch point cloud filename in arguments | Works with PCD and PLY files
	// 获取点云文件名
	std::vector<int> filenames;
	bool file_is_pcd = false;

	filenames = pcl::console::parse_file_extension_argument (argc, argv, ".ply");

	if (filenames.size () != 1)  {
		filenames = pcl::console::parse_file_extension_argument (argc, argv, ".pcd");

		if (filenames.size () != 1) {
			std::cout << std::endl;
			std::cout << "Usage: " << argv[0] << "cloud_filename.[pcd][ply]" << std::endl;
			return -1;
		} else {
			file_is_pcd = true;
		}
	}
	*/

	// Load file | Works with PCD and PLY files
	// 加载点云文件
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());

	bool file_is_pcd = true;
	// std::string filename = "new_file.pcd";
	std::string filename = "20160426_CALIB.pcd";
	// std::string filename = "filter.pcd";

	if (file_is_pcd) {
		//if (pcl::io::loadPCDFile (argv[filenames[0]], *cloud) < 0)  {
		if (pcl::io::loadPCDFile(filename, *cloud) < 0) {
			//std::cout << "Error loading point cloud " << argv[filenames[0]] << std::endl << std::endl;
			std::cout << "Error loading point cloud " << filename << std::endl << std::endl;
			std::cout << std::endl;
			std::cout << "Usage: " << argv[0] << "cloud_filename.[pcd][ply]" << std::endl;
			return -1;
		}
	}
	else {
		//if (pcl::io::loadPLYFile (argv[filenames[0]], *cloud) < 0)  {
		if (pcl::io::loadPLYFile(filename, *cloud) < 0) {
			//std::cout << "Error loading point cloud " << argv[filenames[0]] << std::endl << std::endl;
			std::cout << "Error loading point cloud " << filename << std::endl << std::endl;
			std::cout << std::endl;
			std::cout << "Usage: " << argv[0] << "cloud_filename.[pcd][ply]" << std::endl;
			return -1;
		}
	}

	showHelp();

	while (true) {
		std::cout << ">> ";
		std::string line;
		getline(std::cin, line);
		// exit program
		// 退出程序
		if (line == "quit" || line == "q") {
			break;
		}
		// show help menu
		// 显示help菜单
		else if (line == "help") {
			showHelp ();
		}
		// calculate kdtree
		// 计算kdtree
		else if (line == "tree" || line == "t") {
			kd_tree (cloud);
			std::cin.ignore(INT_MAX, '\n');
		}
		// streams points to file
		// 指向文件的流
		else if (line == "points" || line == "p") {
			points (cloud);
		}
		// calculate normals
		// 计算法线
		else if (line == "normals" || line == "n") {
			normals (cloud);
		}
		// visualize 'new_file.pcd'
		// 点云可视化
		else if (line == "visualize" || line == "v") {
			visualize(cloud);
		}
		// calculate holes and stream to boundary.pcd
		// 计算孔洞
		else if (line == "holes" || line == "h") {
			calculate_hole (cloud);
			std::cin.ignore(INT_MAX, '\n');
		}
		else{
			std::cout <<"problem -> "<<line<< " *Error: Command not recognized enter '_help' to view help menu" << std::endl;
		}
	}
	return 0;
}

/*********************************************************************************************/
