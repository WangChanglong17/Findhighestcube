#include "calculate.h"
using namespace std;
using namespace cv;

const double pi = 3.141592653;

double template_standard(template_type x, vector<double> &distance)
{
	Mat a,input, gray, imgcanny;
	vector<vector<Point>> contours1;
	vector<Vec4i> hierarchy1;
	bool rand_label = false;
	switch (x){
	case square_:
		a = imread("E:\\研究生课题\\王汝宁课题\\形状模板图像\\square.png");
		break;
	case rectangle_:
		a = imread("E:\\研究生课题\\王汝宁课题\\形状模板图像\\rectangle.jpg");
		break;
	case triangle_:
		a = imread("E:\\研究生课题\\王汝宁课题\\形状模板图像\\triangle.png");
		break;
	case circle_:
		a = imread("E:\\研究生课题\\王汝宁课题\\形状模板图像\\circle.png");
		rand_label = true;
		break;
	}
		//cvtColor(a, gray, CV_RGB2GRAY);
	a.copyTo(gray);
	//Canny(gray, imgcanny, 30, 40, 3);
	//("Canny", WINDOW_AUTOSIZE);
	//imshow("Canny", a);
	//cvWaitKey(0);
	Mat binary;
	threshold(gray, binary, 254, 255, THRESH_BINARY_INV);
	//namedWindow("binary", WINDOW_AUTOSIZE);
	//imshow("binary", binary);
	//cvWaitKey(0);
	Canny(binary, imgcanny, 30, 40, 3);
	
	Mat imgdilate;
	Mat element1 = getStructuringElement(MORPH_DILATE, Size(5, 5));
	dilate(imgcanny, imgdilate, element1);
	//imshow("binary", imgdilate);
	//cvWaitKey(0);
	findContours(imgdilate, contours1, hierarchy1, RETR_TREE, CHAIN_APPROX_NONE);
	Moments target_moment;
	target_moment = moments(contours1[0], false);
	int cx = int(target_moment.m10 / target_moment.m00);
	int cy = int(target_moment.m01 / target_moment.m00);
	Point centroid = Point(cx, cy);
		

	vector<double> angle_set;
		
	for (size_t i = 0; i < contours1[0].size(); i++)
	{
		angle_set.push_back(calculate_angle(centroid, contours1[0][i]));
		
	}
	vector<double> angle_set_copy = angle_set;
	sort(angle_set_copy.begin(), angle_set_copy.end());

	//将轮廓内点全部重新排序，按照角度从0到360度，逆时针旋转,并构建距离的序列
	vector<Point> point_contours;
	//vector<double> distance;
	double angle_step = 0.2; double dis = 0;
	srand((unsigned)time(NULL));
	for (double i = 0; i < 360; i = i + angle_step)
	{
		size_t sequence_num_1; size_t sequence_num_2; int tem_x; int tem_y; bool label = false;
		vector<double>::iterator angle_iterator_1; vector<double>::iterator angle_iterator_2;
		for (sequence_num_1 = 0; sequence_num_1<angle_set_copy.size(); sequence_num_1++)
		{
			if (angle_set_copy[sequence_num_1] > i)
			{
				label = true; break;
			}
		}

		if (label == true)
		{
			sequence_num_1 = sequence_num_1 - 1;
			angle_iterator_1 = find(angle_set.begin(), angle_set.end(), angle_set_copy[sequence_num_1]);
			angle_iterator_2 = find(angle_set.begin(), angle_set.end(), angle_set_copy[sequence_num_1 + 1]);
			sequence_num_2 = angle_iterator_2 - angle_set.begin();
			sequence_num_1 = angle_iterator_1 - angle_set.begin();
				
			tem_x = int(((i - angle_set[sequence_num_1]) * contours1[0][sequence_num_2].x + (-i + angle_set[sequence_num_2]) * contours1[0][sequence_num_1].x)
				/ (angle_set[sequence_num_2] - angle_set[sequence_num_1]));
			tem_y = int(((i - angle_set[sequence_num_1]) * contours1[0][sequence_num_2].y + (-i + angle_set[sequence_num_2]) * contours1[0][sequence_num_1].y)
				/ (angle_set[sequence_num_2] - angle_set[sequence_num_1]));

		}
		else
		{
			sequence_num_1 = sequence_num_1 - 1;
			angle_iterator_1 = find(angle_set.begin(), angle_set.end(), angle_set_copy[sequence_num_1]);
			angle_iterator_2 = find(angle_set.begin(), angle_set.end(), angle_set_copy[0]);
			sequence_num_2 = angle_iterator_2 - angle_set.begin();
			sequence_num_1 = angle_iterator_1 - angle_set.begin();

				
			tem_x = int(((i - angle_set[sequence_num_1]) * contours1[0][sequence_num_2].x + (360 - i + angle_set[sequence_num_2]) * contours1[0][sequence_num_1].x)
				/ (360 + angle_set[sequence_num_2] - angle_set[sequence_num_1]));
			tem_y = int(((i - angle_set[sequence_num_1]) * contours1[0][sequence_num_2].y + (360 - i + angle_set[sequence_num_2]) * contours1[0][sequence_num_1].y)
				/ (360 + angle_set[sequence_num_2] - angle_set[sequence_num_1]));

		}
		
		Point tem = Point(tem_x, tem_y);
		if (rand_label)
		{
			if (i == 0) dis = sqrt((tem_x - centroid.x)*(tem_x - centroid.x) + (tem_y - centroid.y)*(tem_y - centroid.y));
			
			int x_rand = rand() % 100;
		   //	cout << " " << x_rand;
			double y_rand = double(x_rand) * dis * 0.00001 ;
			//cout << dis << endl;
			//cout << y_rand << " ";
			point_contours.push_back(tem);
			double dis_temp = dis + y_rand;
			distance.push_back(dis_temp);
		}
		else
		{
			dis = sqrt((tem_x - centroid.x)*(tem_x - centroid.x) + (tem_y - centroid.y)*(tem_y - centroid.y));
			point_contours.push_back(tem);
			distance.push_back(dis);
		}
	}

	ofstream file1("E:\\研究生课题\\王汝宁课题\\template_distance" + to_string(x)+".txt", ios_base::out);
	if (!file1.is_open())
	{
		cout << "打开template_distance.txt文件失败" << endl;
	}

	double sum = 0;

	for (int i = 0; i < distance.size(); i++)
	{
		sum += distance[i];
		file1 << distance[i] << " ";
	}
	file1.close();
	double mean = sum / distance.size();
	for (int i = 0; i < distance.size(); i++)
	{
		distance[i] = distance[i] / 1;
	}
	return mean;
}


double calculate_angle(Point P1, Point P2)
{
	if (P1.y >= P2.y)
	{
		double angle = double(atan2(P1.y - P2.y, P2.x - P1.x));
		return angle * 180 / pi;
	}
	else
	{
		double angle = double(atan2(P1.y - P2.y, P2.x - P1.x));
		return (angle + 2 * (pi)) * 180 / pi;
	}
}