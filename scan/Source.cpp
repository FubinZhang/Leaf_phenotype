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
	cout << A.leaf_area << endl;//二阶矩求面积
	cout << A.rectangularity << endl;//矩形度：叶片面积与MER面积比值；
	cout << A.densification << endl;//致密度：周长平方与面积比值；
	cout << A.circularity << endl;//圆形性：质心到边界点的平均距离/距离均方差
	cout << A.sphericity << endl;//球状性：2D为内接圆半径/外接圆半径；
	cout << A.boundary_energy << endl;//边界能量
	cout << A.leaf_color << endl;//颜色
	while (true) {}
	return 0;
}