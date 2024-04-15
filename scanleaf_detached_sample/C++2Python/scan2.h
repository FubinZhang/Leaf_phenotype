#pragma once

#include <iostream>
#include <stdio.h>
#include <vector>
#include <time.h>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
// 关于pybind11的头文件
#include <pybind11/stl.h>  
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>

using namespace std;
using namespace cv;

struct leaf
{
	string ERROS="";
	Mat src_gray;
	Mat src_otsu;
	Mat src_hsv;
	double leaf_length;
	double leaf_width;
	double length_widith_ratio;
	double leaf_perimeter;
	double leaf_area;
	double rectangularity;
	double densification;
	double circularity;
	double sphericity;
	double boundary_energy;
	string leaf_color;
};
class scan2
{
public:
	scan2();
	~scan2();
	leaf Process_src(Mat src);
	static bool cmp(vector<Point> const& A, vector<Point> const& B);
	void findLongestLine(vector<vector<Point>>& lines);
private:
	void setPadBlack(Mat& image, int d);
	void changecontours(vector<Point>& linein, vector<Point>& lineout, Point center, double rangle);
	void drawContour(Mat& image, vector<Point>& Line);
	double arc_circularity(vector<Point>& line, Point center);
	double arc_sphericity(vector<Point>& line, Point center, Rect rec, int reduce);
	double arc_boundary_energy(vector<Point>& line, Mat inarray);
	void Count_color(Mat img, Point2d left, Point2d right, string& color);
};

