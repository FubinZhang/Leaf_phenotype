#include "BeanBox2.h"

namespace pybind11 { namespace detail{
template<>
struct type_caster<cv::Mat>{
public:   
 
    PYBIND11_TYPE_CASTER(cv::Mat, _("numpy.ndarray")); 
   
    //! 1. cast numpy.ndarray to cv::Mat    
    bool load(handle obj, bool){        
        array b = reinterpret_borrow<array>(obj);        
        buffer_info info = b.request();    
    
        int nh = 1;        
        int nw = 1;        
        int nc = 1;        
        int ndims = info.ndim;        
        if(ndims == 2){           
           nh = info.shape[0];           
           nw = info.shape[1];       
        } 
        else if(ndims == 3){            
            nh = info.shape[0];           
            nw = info.shape[1];           
            nc = info.shape[2];        
        }else{            
            throw std::logic_error("Only support 2d, 2d matrix");            
            return false;       
        }       

        int dtype;        
        if(info.format == format_descriptor<unsigned char>::format()){            
            dtype = CV_8UC(nc);        
        }else if (info.format == format_descriptor<int>::format()){            
            dtype = CV_32SC(nc);       
        }else if (info.format == format_descriptor<float>::format()){           
            dtype = CV_32FC(nc);        
        }else{            
            throw std::logic_error("Unsupported type, only support uchar, int32, float"); 
            return false;
        }   
        value = cv::Mat(nh, nw, dtype, info.ptr);
        return true;    
    }    

    //! 2. cast cv::Mat to numpy.ndarray    
    static handle cast(const cv::Mat& mat, return_value_policy, handle defval){        
        std::string format = format_descriptor<unsigned char>::format();
        size_t elemsize = sizeof(unsigned char);
        int nw = mat.cols;
        int nh = mat.rows;
        int nc = mat.channels();
        int depth = mat.depth();
        int type = mat.type();
        int dim = (depth == type)? 2 : 3;
        if(depth == CV_8U){
            format = format_descriptor<unsigned char>::format();
            elemsize = sizeof(unsigned char);
        }else if(depth == CV_32S){
            format = format_descriptor<int>::format();
            elemsize = sizeof(int);
        }else if(depth == CV_32F){
            format = format_descriptor<float>::format();
            elemsize = sizeof(float);
        }else{            
            throw std::logic_error("Unsupport type, only support uchar, int32, float");
        }        

        std::vector<size_t> bufferdim;
        std::vector<size_t> strides;
        if (dim == 2) {
            bufferdim = {(size_t) nh, (size_t) nw};
            strides = {elemsize * (size_t) nw, elemsize};
        } else if (dim == 3) {
            bufferdim = {(size_t) nh, (size_t) nw, (size_t) nc};
            strides = {(size_t) elemsize * nw * nc, (size_t) elemsize * nc, (size_t) elemsize};
        }
        return array(buffer_info( mat.data,  elemsize,  format, dim, bufferdim, strides )).release();    
}};

}}

BeanBox2::BeanBox2()
{
}


BeanBox2::~BeanBox2()
{
}


Intrinsics BeanBox2::string2rsintrinsics(string intrinsics2)
{
	Intrinsics intrinsics;
	vector<std::string> tokens;
	string token;
	istringstream tokenStream(intrinsics2);
	while (getline(tokenStream, token, '/'))
	{
		tokens.push_back(token);
	}
	intrinsics.width = stof(tokens[0]);
	intrinsics.height = stof(tokens[1]);
	intrinsics.fx = stof(tokens[2]);
	intrinsics.fy = stof(tokens[3]);
	intrinsics.cx = stof(tokens[4]);
	intrinsics.cy = stof(tokens[5]);
	return intrinsics;
}

int BeanBox2::countPngFiles(const std::string& folderPath) {

	int count = 0;

	const char* path = folderPath.c_str();

	DIR* dir = opendir(path);
	if (dir == NULL) {
		return -1;
	}

	struct dirent* entry;
	while ((entry = readdir(dir)) != NULL) {

		std::string name = entry->d_name;
		if (name.size() >= 4 &&
			name.substr(name.size() - 4) == ".png") {
			count++;
		}

	}

	closedir(dir);

	return count;
}

pcl_ptr BeanBox2::leaf_3d(Intrinsics intrinsics, Mat color_im, Mat depth_im)
{
	try {
		float fx = intrinsics.fx, fy = intrinsics.fy, cx = intrinsics.cx, cy = intrinsics.cy;
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
		const int h = color_im.rows;
		const int w = color_im.cols;
		float point3d[3];
		for (int y = 0; y < h; ++y) {
			for (int x = 0; x < w; ++x) {
				Vec3b color = color_im.at<Vec3b>(y, x);
				if (color[0] != 0 || color[1] != 0 || color[2] != 0)
				{
					// Get depth value
					uint16_t d = depth_im.at<uint16_t>(y, x);

					// Skip pixels with a depth value of 0
					//if (d == 0) continue;

					// Compute 3D coordinates
					pcl::PointXYZ point;
					point.z = d;
					point.x = (x - cx) * point.z / fx;
					point.y = (y - cy) * point.z / fy;

					// Get color information
					/*cv::Vec3b color_pixel = color_im.at<cv::Vec3b>(y, x);
					point.r = color_pixel[2];
					point.g = color_pixel[1];
					point.b = color_pixel[0];*/
					//cout << color_pixel << endl;
					// Add point to the cloud
					cloud->push_back(point);

					/*
					unsigned char depth_value = depth_im.at<unsigned char>(y, x);
					pcl::PointXYZRGB p;
					p.x = (x - h / 2) / intrinsics.fx*depth_value;
					p.y = (y - w / 2) / intrinsics.fy*depth_value;
					p.z = depth_value;
					p.r = color[0];
					p.g = color[1];
					p.b = color[2];
					cloud->push_back(p);
					*/
				}
			}
		}
		return cloud;
	}
	catch (const std::exception& e)
	{
		std::cerr << "�� leaf_3d() �з�������" << e.what() << std::endl;
		return nullptr; // �ڳ��ִ���ʱ���� nullptr
	}
}

double BeanBox2::calculateTriangleArea(pcl::PointXYZ point1, pcl::PointXYZ point2, pcl::PointXYZ point3) {
	// ����������֮��ľ���
	double d1 = std::sqrt(std::pow(point2.x - point1.x, 2) + std::pow(point2.y - point1.y, 2) + std::pow(point2.z - point1.z, 2));
	double d2 = std::sqrt(std::pow(point3.x - point1.x, 2) + std::pow(point3.y - point1.y, 2) + std::pow(point3.z - point1.z, 2));
	double d3 = std::sqrt(std::pow(point3.x - point2.x, 2) + std::pow(point3.y - point2.y, 2) + std::pow(point3.z - point2.z, 2));

	// ������ܳ�
	double s = (d1 + d2 + d3) / 2;

	// �������������
	double area = std::sqrt(s * (s - d1) * (s - d2) * (s - d3));

	return area;
}

bool BeanBox2::compareByPolarAngle(const pcl::PointXYZ& p1, const pcl::PointXYZ& p2, const pcl::PointXYZ& reference)
{
	double angle1 = std::atan2(p1.y - reference.y, p1.x - reference.x);
	double angle2 = std::atan2(p2.y - reference.y, p2.x - reference.x);
	return angle1 < angle2;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr BeanBox2::predeal_1(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	try {

		pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
		voxel_filter.setInputCloud(cloud);
		voxel_filter.setLeafSize(3, 3, 3);
		voxel_filter.filter(*downsampled);

		//pcl::io::savePCDFileASCII("./downsampled.pcd", *downsampled);
		long total_points = downsampled->points.size();
		long extract_points = total_points * 0.2;
		pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor_filter;
		sor_filter.setInputCloud(downsampled);
		sor_filter.setMeanK(extract_points);
		sor_filter.setStddevMulThresh(0.5);
		sor_filter.filter(*filtered);

		//pcl::io::savePCDFileASCII("./filtered.pcd", *filtered);

		/*
		pcl::RadiusOutlierRemoval<pcl::PointXYZ> radius_filter;
		radius_filter.setInputCloud(downsampled);
		radius_filter.setRadiusSearch(5.0);
		radius_filter.setMinNeighborsInRadius(40);

		pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
		radius_filter.filter(*filtered);*/
		return filtered;
	}
	catch (const std::exception& e)
	{
		std::cerr << "�� predeal1() �з�������" << e.what() << std::endl;
		return nullptr; // �ڳ��ִ���ʱ���� nullptr
	}
}

pcl::PointCloud<pcl::PointXYZ>::Ptr BeanBox2::predeal_2(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	try {
		// ʹ�����ĵ��ƴؽ��й�һ������
		pcl::PointXYZ centroid;
		pcl::computeCentroid(*cloud, centroid);
		Eigen::Affine3f transform = Eigen::Affine3f::Identity();
		transform.translation() << -centroid.x, -centroid.y, -centroid.z;
		pcl::PointCloud<pcl::PointXYZ>::Ptr normalized_cloud(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::transformPointCloud(*cloud, *normalized_cloud, transform);

		return normalized_cloud;
	}
	catch (const std::exception& e)
	{
		std::cerr << "�� predeal2() �з�������" << e.what() << std::endl;
		return nullptr; // �ڳ��ִ���ʱ���� nullptr
	}
}

pcl::PointCloud<pcl::PointXYZ>::Ptr BeanBox2::getboundaryCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	try {
		pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
		pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
		normalEstimation.setInputCloud(cloud);
		normalEstimation.setRadiusSearch(5);  // ���÷��߹��Ƶ������뾶
		normalEstimation.compute(*normals);

		pcl::PointCloud<pcl::Boundary>::Ptr boundaries(new pcl::PointCloud<pcl::Boundary>);
		pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> boundaryEstimation;
		boundaryEstimation.setInputCloud(cloud);
		boundaryEstimation.setInputNormals(normals);
		boundaryEstimation.setRadiusSearch(5);  // ���ñ߽���ȡ�������뾶
		boundaryEstimation.compute(*boundaries);

		pcl::PointCloud<pcl::PointXYZ>::Ptr boundaryCloud(new pcl::PointCloud<pcl::PointXYZ>);
		for (std::size_t i = 0; i < cloud->size(); ++i) {
			if ((*boundaries)[i].boundary_point > 0)
				boundaryCloud->push_back((*cloud)[i]);
		}

		//pcl::io::savePCDFile<pcl::PointXYZ>("boundary_points.pcd", *boundaryCloud);

		// ��Ⱥ���Ƴ���ͳ���˲���
		pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor_filter;
		sor_filter.setInputCloud(boundaryCloud);
		sor_filter.setMeanK(10);  // �����������
		sor_filter.setStddevMulThresh(1.0);  // ���ñ�׼�����ֵ
		sor_filter.filter(*filtered);

		return filtered;
	}
	catch (const std::exception& e)
	{
		std::cerr << "�� getboundaryCloud() �з�������" << e.what() << std::endl;
		return nullptr; // �ڳ��ִ���ʱ���� nullptr
	}
}

pcl::PointCloud<pcl::PointXYZ>::Ptr BeanBox2::get_sortboundaryCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	try {
		// ����������ݵ�����
		pcl::PointXYZ centroid;
		pcl::computeCentroid(*cloud, centroid);
		pcl::PointXYZ reference(centroid.x, centroid.y, centroid.z);

		// ���ݼ�������߽��
		std::sort(cloud->points.begin(), cloud->points.end(),
			[&](const pcl::PointXYZ& p1, const pcl::PointXYZ& p2) {
			return compareByPolarAngle(p1, p2, reference);
		});
		return cloud;
	}
	catch (const std::exception& e)
	{
		std::cerr << "�� get_sortboundaryCloud() �з�������" << e.what() << std::endl;
		return nullptr; // �ڳ��ִ���ʱ���� nullptr
	}
}

void BeanBox2::get_len_wid(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float &lenth, float &width)
{
	try {
		// ִ�����ɷַ���
		pcl::PCA<pcl::PointXYZ> pca;
		pca.setInputCloud(cloud);

		Eigen::Vector3f eigenvalues = pca.getEigenValues();
		Eigen::Matrix3f eigenvectors = pca.getEigenVectors();

		// ѡ���������ֵ��Ӧ������������Ϊ���᷽��
		Eigen::Vector3f major_axis = eigenvectors.col(0);

		std::vector<float> projected_lengths;

		for (const auto& point : cloud->points) {
			Eigen::Vector3f point_vector(point.x, point.y, point.z);
			float projected_length = point_vector.dot(major_axis);
			projected_lengths.push_back(projected_length);
		}
		float min_length = *std::min_element(projected_lengths.begin(), projected_lengths.end());
		float max_length = *std::max_element(projected_lengths.begin(), projected_lengths.end());
		lenth = max_length - min_length;

		Eigen::Vector3f up_vector(0.0, 0.0, 1.0);  // ����ҶƬ�Ŀ��ȷ�������z�ᴹֱ��

		Eigen::Vector3f width_axis = major_axis.cross(up_vector);
		width_axis.normalize();

		std::vector<float> projected_widths;

		for (const auto& point : cloud->points) {
			Eigen::Vector3f point_vector(point.x, point.y, point.z);
			float projected_width = point_vector.dot(width_axis);
			projected_widths.push_back(projected_width);
		}

		float min_width = *std::min_element(projected_widths.begin(), projected_widths.end());
		float max_width = *std::max_element(projected_widths.begin(), projected_widths.end());
		width = max_width - min_width;
	}
	catch (const std::exception& e)
	{
		std::cerr << "�� get_len_wid() �з�������" << e.what() << std::endl;
	}
}

void BeanBox2::get_perimeter_sort(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float &perimeter)
{
	try {
		//�����ܳ�
		perimeter = 0.0;
		for (int i = 0; i < cloud->size(); i++)
		{
			pcl::PointXYZ p1 = cloud->points[i];
			pcl::PointXYZ p2 = cloud->points[(i + 1) % cloud->size()];
			double dx = p2.x - p1.x;
			double dy = p2.y - p1.y;
			double dz = p2.z - p1.z;
			double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
			perimeter += distance;
		}
	}
	catch (const std::exception& e)
	{
		std::cerr << "�� get_perimeter_sort() �з�������" << e.what() << std::endl;
	}
}

void BeanBox2::get_area(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float &area)
{
	try {
		// ����������ݵ�����
		pcl::PointXYZ centroid;
		pcl::computeCentroid(*cloud, centroid);
		pcl::PointXYZ reference(centroid.x, centroid.y, centroid.z);


		if (cloud->size() < 3) {
			std::cout << "�����������㣬�޷����������" << std::endl;
		}

		area = 0.0;

		for (size_t i = 0; i < cloud->size(); ++i) {
			pcl::PointXYZ p1, p2;
			if (i == cloud->size() - 1) {
				p1 = cloud->at(i);
				p2 = cloud->at(0);
			}
			else {
				p1 = cloud->at(i);
				p2 = cloud->at(i + 1);
			}


			double traingleArea;
			traingleArea = calculateTriangleArea(reference, p1, p2);
			area += traingleArea;
		}
	}
	catch (const std::exception& e)
	{
		std::cerr << "�� get_area() �з�������" << e.what() << std::endl;
	}
}

void BeanBox2::run(string intrinsics,string depth_path, string leaf_path, string date_outpath)
{
	Beandate Bean_leaf;
	Intrinsics intrinsics1 = string2rsintrinsics(intrinsics);

	// ���屣�����ݵ��ı��ļ���
	std::string filename = date_outpath+"/leaf_data.txt";
	// ���������ı��ļ������
	std::ofstream outputFile(filename);
	outputFile.is_open();

	Mat depth = imread(depth_path,IMREAD_UNCHANGED);
	int leaf_number = countPngFiles(leaf_path);
	Bean_leaf.leaf.resize(leaf_number);
	Bean_leaf.length.resize(Bean_leaf.leaf.size());
	Bean_leaf.weight.resize(Bean_leaf.leaf.size());
	Bean_leaf.perimeter.resize(Bean_leaf.leaf.size());
	Bean_leaf.area.resize(Bean_leaf.leaf.size());
	for (int j = 0; j < leaf_number; j++)
	{
		Mat img = imread(leaf_path+"/leaf_"+std::to_string(j)+".png");
			//cvtColor(img, img, COLOR_RGB2BGR);
			//pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
			//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);
		cloud = leaf_3d(intrinsics1, img, depth);
		Bean_leaf.leaf[j] = cloud;
		    //pcl::io::savePCDFileASCII(leaf_path+"/leaf_" + std::to_string(j) + ".pcd", *cloud);
			//pcl::io::loadPCDFile<pcl::PointXYZ>("./image/leaf_pcd_000000/" + std::to_string(j) + ".pcd", *cloud);
		    //pcl::io::loadPCDFile<pcl::PointXYZ>(leaf_path + "/leaf_" + std::to_string(j) + ".pcd", *cloud);
		pcl::PointCloud<pcl::PointXYZ>::Ptr pred_cloud1(new pcl::PointCloud<pcl::PointXYZ>);
		pred_cloud1 = predeal_1(cloud);
			//pcl::io::savePCDFileASCII(leaf_path + "/pcd/leaf_" + std::to_string(j) + "pre1.pcd", *pred_cloud1);

		pcl::PointCloud<pcl::PointXYZ>::Ptr pred_cloud2(new pcl::PointCloud<pcl::PointXYZ>);
		pred_cloud2 = predeal_2(pred_cloud1);
			//pcl::io::savePCDFileASCII(leaf_path + "/pcd/leaf_" + std::to_string(j) + "pre2.pcd", *pred_cloud2);

		pcl::PointCloud<pcl::PointXYZ>::Ptr boundaryCloud(new pcl::PointCloud<pcl::PointXYZ>);
		boundaryCloud = getboundaryCloud(pred_cloud2);
			//pcl::io::savePCDFileASCII(leaf_path + "/pcd/leaf_" + std::to_string(j) + "bou.pcd", *boundaryCloud);

		pcl::PointCloud<pcl::PointXYZ>::Ptr sort_boundaryCloud(new pcl::PointCloud<pcl::PointXYZ>);
		sort_boundaryCloud = get_sortboundaryCloud(boundaryCloud);

		float lenth, width, perimeter, area;
		get_len_wid(sort_boundaryCloud, lenth, width);
		get_perimeter_sort(sort_boundaryCloud, perimeter);
		get_area(sort_boundaryCloud, area);
		Bean_leaf.length[j] = lenth;
		Bean_leaf.weight[j] = width;
		Bean_leaf.perimeter[j] = perimeter;
		Bean_leaf.area[j] = area;
		outputFile << "Leaf " << j << "; "
			<< "Length: " << Bean_leaf.length[j] << " mm, "
			<< "Width: " << Bean_leaf.weight[j] << " mm, "
			<< "Perimeter: " << Bean_leaf.perimeter[j] << " mm, "
			<< "Area: " << Bean_leaf.area[j] << " square mm"
			<< endl;
		cout << j << endl;
			/*
			cout << Bean_leaf[i].length[j] << endl;
			cout << Bean_leaf[i].weight[j] << endl;
			cout << Bean_leaf[i].perimeter[j] << endl;
			cout << Bean_leaf[i].area[j] << endl;*/
	}
	outputFile.close();
}


PYBIND11_MODULE(pybeanbox, m) {
    pybind11::class_<BeanBox2>(m, "BeanBox2")
        .def(pybind11::init<>())
        .def("run", &BeanBox2::run);
}