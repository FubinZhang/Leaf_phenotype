#pragma once

#include <iostream>
#include <stdio.h>
#include <vector>
#include <time.h>
#include <cmath>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;
struct leaf
{
	string ERROS="";
	Mat src_gray;
	Mat src_otsu;
	Mat src_hsv;
	double leaf_length;//长
	double leaf_width;//宽
	double length_widith_ratio;//长宽比
	double leaf_perimeter;//周长
	double leaf_area;//二阶矩求面积
	double rectangularity;//矩形度：叶片面积与MER面积比值；
	double densification;//致密度：周长平方与面积比值；
	double circularity;//圆形性：质心到边界点的平均距离/距离均方差
	double sphericity;//球状性：2D为内接圆半径/外接圆半径；
	double boundary_energy;//边界能量
	string leaf_color;//颜色
};
class scan2
{
public:
	scan2();
	~scan2();
	leaf Process_src(string path);
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

