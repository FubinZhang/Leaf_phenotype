#include "scan2.h"


scan2::scan2()
{
}


scan2::~scan2()
{
}


void scan2::setPadBlack(Mat& image, int d)
{
	int rows = image.rows;
	int cols = image.cols;
	for (int i = 0; i < rows; i++)
	{
		for (int j = 0; j < cols; j++)
		{
			if (i < d) image.at<uchar>(i, j) = 0;
			if (j < d) image.at<uchar>(i, j) = 0;
			if (rows - i < d) image.at<uchar>(i, j) = 0;
			if (cols - j < d) image.at<uchar>(i, j) = 0;
		}
	}
}

bool scan2::cmp(vector<Point> const& A, vector<Point> const& B)
{
	return A.size() > B.size();
}

void scan2::findLongestLine(vector<vector<Point>>& lines)
{
	sort(lines.begin(), lines.end(), scan2::cmp);
	lines.resize(1);
}


void scan2::changecontours(vector<Point>& linein, vector<Point>& lineout, Point center, double rangle)
{
	const double pi = acos(-1.0);
	for (size_t i = 0; i < linein.size(); i++)
	{
		double x1 = (linein[i].x - center.x) * cos(rangle / 180 * pi) - (linein[i].y - center.y) * sin(rangle / 180 * pi) + center.x;
		double y1 = (linein[i].x - center.x) * sin((rangle / 180 * pi)) + (linein[i].y - center.y) * cos(rangle / 180 * pi) + center.y;
		lineout[i] = Point(x1, y1);
	}
}

void scan2::drawContour(Mat& image, vector<Point>& Line)
{
	size_t len = Line.size();
	for (size_t i = 0; i < len; i++)
	{
		Point pt = Line[i];
		image.at<uchar>(pt.y, pt.x) = 255;
	}
}

double scan2::arc_circularity(vector<Point>& line, Point center)
{
	double avedistance = 0;//质心到边界点的平均距离
	double vardistance = 0;//距离均方差
	double circularity;//圆形性：质心到边界点的平均距离/距离均方差
	for (int i = 0; i < line.size(); i++)
	{
		avedistance += sqrt(pow(line[i].y - center.y, 2) + pow(line[i].x - center.x, 2));
	}
	avedistance = avedistance / line.size();
	for (int i = 0; i < line.size(); i++)
	{
		vardistance += pow(sqrt(pow(line[i].y - center.y, 2) + pow(line[i].x - center.x, 2)) - avedistance, 2);
	}
	vardistance = vardistance / line.size();
	circularity = avedistance / vardistance;
	return circularity;
}

double scan2::arc_sphericity(vector<Point>& line, Point center, Rect rec, int reduce)
{
	//球状性：2D为内接圆半径/外接圆半径；
	double dist, maxdist = 0;
	Point2f maxcir_center;
	Point mincir_center;
	float  outcir_radius, incir_radius;
	double sphericity;
	minEnclosingCircle(line, maxcir_center, outcir_radius);

	vector<Point>narrow_line;
	for (int i = 0; i < line.size(); i++)
	{
		Point narrowpoint;
		narrowpoint.x = center.x - ((center.x - line[i].x) / 10);
		narrowpoint.y = center.y - ((center.y - line[i].y) / 10);
		narrow_line.push_back(narrowpoint);
	}

	for (int i = center.x - (rec.width / reduce); i < center.x + (rec.width / reduce); i++)
	{
		for (int j = center.y - (rec.height / reduce); j < center.y + (rec.height / reduce); j++)
		{
			dist = pointPolygonTest(narrow_line, Point(i, j), true);
			if (dist > maxdist) {
				maxdist = dist;
				mincir_center = Point(i, j);
			}
		}
	}
	mincir_center.x = center.x - ((center.x - mincir_center.x) * reduce);
	mincir_center.y = center.y - ((center.y - mincir_center.y) * reduce);
	incir_radius = pointPolygonTest(line, mincir_center, true);
	sphericity = incir_radius / outcir_radius;
	return sphericity;
}

double scan2::arc_boundary_energy(vector<Point>& line, Mat inarray)
{
	double E = 0;
	int halfstep2 = 30;
	int last, next;
	vector<Point>three;
	vector<double>curvature;
	double radius;
	for (int i = 0; i < line.size(); i++)
	{
		last = i - halfstep2;
		next = i + halfstep2;
		if (last < 0) { last = line.size() + last; }
		if (next >= line.size()) { next = next - line.size(); }
		Point a, b, c;
		a = line[last];
		b = line[i];
		c = line[next];
		three.push_back(a);
		three.push_back(b);
		three.push_back(c);
		if (a.x == b.x == c.x || a.y == b.y == c.y)
		{
			curvature.push_back(0);
			three.clear();
		}
		else {
			double dis1, dis2, dis3;
			double cosA, sinA, dis;
			dis1 = sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
			dis2 = sqrt((a.x - c.x) * (a.x - c.x) + (a.y - c.y) * (a.y - c.y));
			dis3 = sqrt((b.x - c.x) * (b.x - c.x) + (b.y - c.y) * (b.y - c.y));
			dis = dis1 * dis1 + dis3 * dis3 - dis2 * dis2;
			cosA = dis / (2 * dis1 * dis3);
			sinA = sqrt(1 - cosA * cosA);
			radius = 0.5 * dis2 / sinA;
			curvature.push_back(1 / (radius));
			three.clear();
			if (radius < (inarray.cols))
			{
				E += pow(curvature[curvature.size() - 1], 2);
			}
		}
	}
	E = E / line.size();
	return E;
}

void scan2::Count_color(Mat img, Point2d left, Point2d right, string& color)
{
	int x1 = left.y, x2 = right.y, y1 = left.x, y2 = right.x;
	double Greens = 0, Yellows = 0, Light_Yellows = 0, Light_Greens = 0;
	for (int i = x1; i <= x2; ++i)
	{
		for (int j = y1; j <= y2; ++j)
		{
			double H = img.at<Vec3b>(i, j)[0];
			if (25 <= H && H <= 32)
				Light_Yellows++;
			else if (19 <= H && H <= 24)
				Yellows++;
			else if (H >= 35 && H <= 37)
				Light_Greens++;
			else if (H >= 37 && H <= 61)
				Greens++;
		}
	}
	double all = Greens + Yellows + Light_Greens + Light_Yellows;
	double GSize = Greens / all;
	double YSize = Yellows / all;
	double LGSize = Light_Greens / all;
	double LYSize = Light_Yellows / all;
	double MAX = max(GSize, max(YSize, max(LGSize, LYSize)));
	if (MAX == GSize)
		color = "Green";
	else if (MAX == YSize)
		color = "Yellow";
	else if (MAX == LGSize)
		color = "Light Green";
	else
		color = "Light Yellow";
}

leaf scan2::Process_src(string path)
{

	leaf set;
	Mat src = imread(path, IMREAD_COLOR);
	if (src.empty())
	{
		set.ERROS+="未获得图片源";
		return set;
	}
	const double kc = 29.7 / src.rows;
	cvtColor(src, set.src_gray, COLOR_BGR2GRAY);
	threshold(set.src_gray, set.src_otsu, 60, 255, THRESH_OTSU);
	cvtColor(src, set.src_hsv, COLOR_BGR2HSV);

	for (int i = 0; i < set.src_otsu.rows; i++)
	{
		for (int j = 0; j <set.src_otsu.cols; j++)
		{
			uchar val = set.src_otsu.at<uchar>(i, j);
			if (!val) set.src_otsu.at<uchar>(i, j) = 255;
			else set.src_otsu.at<uchar>(i, j) = 0;
		}
	}
	setPadBlack(set.src_otsu, min(set.src_otsu.rows,set.src_otsu.cols)/100);

	vector<vector<Point>> lines;
	findContours(set.src_otsu, lines, RETR_EXTERNAL, CHAIN_APPROX_NONE);
	findLongestLine(lines);
	Mat test = Mat::zeros(set.src_gray.size(), set.src_gray.type());
	drawContours(test, lines, 0, 255, 2);

	Mat temp(lines.at(0));
	Moments moment = moments(temp, false);
	Point centroid;//质心
	if (moment.m00 != 0) {
		centroid.x = cvRound(moment.m10 / moment.m00);
		centroid.y = cvRound(moment.m01 / moment.m00);
	}

	//椭圆拟合方法求叶片轮廓主轴方向
	RotatedRect box = fitEllipse(lines[0]); //椭圆拟合当前的轮廓线
	vector<Point> line_transform; //创建变换后的点
	line_transform.resize(lines[0].size());
	double rangle = 90 - box.angle;
	changecontours(lines[0], line_transform, box.center, rangle);//以椭圆中心旋转轮廓
	Rect rec = boundingRect(line_transform);//矩形拟合

	set.leaf_length = rec.width * kc;//长
	set.leaf_width = rec.height * kc;//宽
	set.length_widith_ratio = set.leaf_length / set.leaf_width;//长宽比
	set.leaf_perimeter = arcLength(lines[0], true) * kc;//周长
	set.leaf_area = moment.m00 * kc * kc;//二阶矩求面积
	set.rectangularity = moment.m00 / (rec.width * rec.height);//矩形度：叶片面积与MER面积比值；
	set.densification = pow(set.leaf_perimeter, 2) / (set.leaf_area);//致密度：周长平方与面积比值；
	set.circularity = arc_circularity(lines[0], centroid);//圆形性：质心到边界点的平均距离/距离均方差
	set.sphericity = arc_sphericity(line_transform, box.center, rec, 10);//球状性：2D为内接圆半径/外接圆半径；
	set.boundary_energy = arc_boundary_energy(line_transform, src);//边界能量
	Count_color(set.src_hsv, rec.tl(), rec.br(), set.leaf_color);//颜色
	return set;
}
