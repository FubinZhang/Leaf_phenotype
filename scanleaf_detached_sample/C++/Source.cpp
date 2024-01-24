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
	cout << A.leaf_length << endl;//��
	cout << A.leaf_width << endl;//��
	cout << A.length_widith_ratio << endl;//������
	cout << A.leaf_perimeter << endl;//�ܳ�
	cout << A.leaf_area << endl;//���׾������
	cout << A.rectangularity << endl;//���ζȣ�ҶƬ�����MER�����ֵ��
	cout << A.densification << endl;//���ܶȣ��ܳ�ƽ���������ֵ��
	cout << A.circularity << endl;//Բ���ԣ����ĵ��߽���ƽ������/���������
	cout << A.sphericity << endl;//��״�ԣ�2DΪ�ڽ�Բ�뾶/���Բ�뾶��
	cout << A.boundary_energy << endl;//�߽�����
	cout << A.leaf_color << endl;//��ɫ
	while (true) {}
	return 0;
}