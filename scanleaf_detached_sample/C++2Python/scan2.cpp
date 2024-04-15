#include "scan2.h"
#include <pybind11/pybind11.h>

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
	double avedistance = 0;
	double vardistance = 0;
	double circularity;
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

leaf scan2::Process_src(Mat src)
{

	leaf set;
	if (src.empty())
	{
		set.ERROS+="δ���ͼƬԴ";
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
	Point centroid;//����
	if (moment.m00 != 0) {
		centroid.x = cvRound(moment.m10 / moment.m00);
		centroid.y = cvRound(moment.m01 / moment.m00);
	}

	
	RotatedRect box = fitEllipse(lines[0]); 
	vector<Point> line_transform; 
	line_transform.resize(lines[0].size());
	double rangle = 90 - box.angle;
	changecontours(lines[0], line_transform, box.center, rangle);
	Rect rec = boundingRect(line_transform);

	set.leaf_length = rec.width * kc;
	set.leaf_width = rec.height * kc;
	set.length_widith_ratio = set.leaf_length / set.leaf_width;
	set.leaf_perimeter = arcLength(lines[0], true) * kc;
	set.leaf_area = moment.m00 * kc * kc;
	set.rectangularity = moment.m00 / (rec.width * rec.height);
	set.densification = pow(set.leaf_perimeter, 2) / (set.leaf_area);
	set.circularity = arc_circularity(lines[0], centroid);
	set.sphericity = arc_sphericity(line_transform, box.center, rec, 10);
	set.boundary_energy = arc_boundary_energy(line_transform, src);
	Count_color(set.src_hsv, rec.tl(), rec.br(), set.leaf_color);
	return set;
}

// Pybind11 module definition
PYBIND11_MODULE(pycv, m) {
    pybind11::class_<leaf>(m, "leaf")
        .def(pybind11::init<>())
        .def_readwrite("ERROS", &leaf::ERROS)
        .def_readwrite("src_gray", &leaf::src_gray)
        .def_readwrite("src_otsu", &leaf::src_otsu)
        .def_readwrite("src_hsv", &leaf::src_hsv)
        .def_readwrite("leaf_length", &leaf::leaf_length)
        .def_readwrite("leaf_width", &leaf::leaf_width)
        .def_readwrite("length_widith_ratio", &leaf::length_widith_ratio)
        .def_readwrite("leaf_perimeter", &leaf::leaf_perimeter)
        .def_readwrite("leaf_area", &leaf::leaf_area)
        .def_readwrite("rectangularity", &leaf::rectangularity)
        .def_readwrite("densification", &leaf::densification)
        .def_readwrite("circularity", &leaf::circularity)
        .def_readwrite("sphericity", &leaf::sphericity)
        .def_readwrite("boundary_energy", &leaf::boundary_energy)
        .def_readwrite("leaf_color", &leaf::leaf_color);

    pybind11::class_<scan2>(m, "scan2")
        .def(pybind11::init<>())
        .def("Process_src", &scan2::Process_src);
}
