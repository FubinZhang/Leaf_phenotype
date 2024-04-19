#include"scan2.h"

int main()
{
	string path = "./leaf.png";
	scan2 B;
	leaf A = B.Process_src(path);
	if (A.ERROS != "")
	{
		cout << A.ERROS ;
	}
	else {
		cout << "ok" << endl;
	}
	cout << A.leaf_length << endl;//长
	cout << A.leaf_width << endl;//宽
	cout << A.length_widith_ratio << endl;//长宽比
	cout << A.leaf_perimeter << endl;//周长
	cout << A.leaf_area << endl;//面积
	cout << A.rectangularity << endl;//矩形度
	cout << A.densification << endl;//致密度
	cout << A.circularity << endl;//圆形度
	cout << A.sphericity << endl;//球度
	cout << A.boundary_energy << endl;//边界能量
	cout << A.leaf_color << endl;//颜色
	return 0;
}