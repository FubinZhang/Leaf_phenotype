#pragma once
#include <fstream>
#include <dirent.h>
#include <sstream>
#include <iostream>
#include <vector>

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <pcl/common/copy_point.h>
#include <pcl/common/transforms.h>
#include <pcl/common/distances.h>
#include <pcl/common/pca.h>
#include <pcl/common/centroid.h>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/features/normal_3d.h>
#include <pcl/features/boundary.h>
#include <pcl/features/moment_of_inertia_estimation.h>

#include <Eigen/Core>

#include <pybind11/stl.h>
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>

using namespace cv;
using namespace std;

using pcl_ptr = pcl::PointCloud<pcl::PointXYZ>::Ptr;
using pclRGB_ptr = pcl::PointCloud<pcl::PointXYZRGB>::Ptr;
using pclNormal_ptr = pcl::PointCloud<pcl::Normal>::Ptr;

struct Beandate {
	vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> leaf;
	vector<float> area;
	vector<float> perimeter;
	vector<float> length;
	vector<float> weight;
};

struct Intrinsics {
	double width;
	double height;
	double fx;
	double fy;
	double cx;
	double cy;
};

class BeanBox2
{
	
public:
	BeanBox2();
	~BeanBox2();
	
	Intrinsics string2rsintrinsics(string intrinsics2);

	int countPngFiles(const std::string& folderPath);

	pcl_ptr leaf_3d(Intrinsics intrinsics, Mat color_im, Mat depth_im);

	double calculateTriangleArea(pcl::PointXYZ point1, pcl::PointXYZ point2, pcl::PointXYZ point3);

	bool compareByPolarAngle(const pcl::PointXYZ& p1, const pcl::PointXYZ& p2, const pcl::PointXYZ& reference);

	pcl::PointCloud<pcl::PointXYZ>::Ptr predeal_1(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

	pcl::PointCloud<pcl::PointXYZ>::Ptr predeal_2(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

	pcl::PointCloud<pcl::PointXYZ>::Ptr getboundaryCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

	pcl::PointCloud<pcl::PointXYZ>::Ptr get_sortboundaryCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

	void get_len_wid(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float &lenth, float &width);

	void get_perimeter_sort(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float &perimeter);

	void get_area(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float &area);

	void run(string intrinsics,string depth_path,string leaf_path,string date_outpath);

};

