#include "calculate.h"
using namespace std;
using namespace cv;


double * Contours_matching(vector<vector<Point>> contours, int min_depth_seq_int, template_type x)
{
	Moments target_moment;
	target_moment = moments(contours[min_depth_seq_int], false);
	int cx = int(target_moment.m10 / target_moment.m00);
	int cy = int(target_moment.m01 / target_moment.m00);
	Point centroid = Point(cx, cy);
	std::cout << "轮廓形心为" << centroid.x << centroid.y << endl;
	//画出形心
	//cv::circle(depth2, centroid, 2, Scalar(0, 0, 0));
	for (int i = 0; i < contours[min_depth_seq_int].size(); i++)
	{
		//circle(depth2, contours[min_depth_seq_int][i], 2, Scalar(0, 0, 0));


	}

	//对轮廓上每一点，求其与形心连线与水平线的夹角
	vector<double> angle_set;
	std::cout << "size: " << contours[min_depth_seq_int].size() << endl;
	for (int i = 0; i < contours[min_depth_seq_int].size(); i++)
	{
		angle_set.push_back(calculate_angle(centroid, contours[min_depth_seq_int][i]));
		//cout << angle_set[i] << " " ;
	}
	vector<double> angle_set_copy = angle_set;
	sort(angle_set_copy.begin(), angle_set_copy.end());

	//将轮廓内点全部重新排序，按照角度从0到360度，逆时针旋转,并构建距离的序列
	vector<Point> point_contours;
	vector<double> distance;
	double angle_step = 1;
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

			//cout << angle_set[sequence_num_1] << "  " << angle_set[sequence_num_2];
			tem_x = int(((i - angle_set[sequence_num_1]) * contours[min_depth_seq_int][sequence_num_1].x + (-i + angle_set[sequence_num_2]) * contours[min_depth_seq_int][sequence_num_2].x)
				/ (angle_set[sequence_num_2] - angle_set[sequence_num_1]));
			tem_y = int(((i - angle_set[sequence_num_1]) * contours[min_depth_seq_int][sequence_num_1].y + (-i + angle_set[sequence_num_2]) * contours[min_depth_seq_int][sequence_num_2].y)
				/ (angle_set[sequence_num_2] - angle_set[sequence_num_1]));

		}
		else
		{
			sequence_num_1 = sequence_num_1 - 1;
			angle_iterator_1 = find(angle_set.begin(), angle_set.end(), angle_set_copy[sequence_num_1]);
			angle_iterator_2 = find(angle_set.begin(), angle_set.end(), angle_set_copy[0]);
			sequence_num_2 = angle_iterator_2 - angle_set.begin();
			sequence_num_1 = angle_iterator_1 - angle_set.begin();

			//cout << angle_set[sequence_num_1] << "  " << angle_set[sequence_num_2];
			tem_x = int(((i - angle_set[sequence_num_1]) * contours[min_depth_seq_int][sequence_num_2].x + (360 - i + angle_set[sequence_num_2]) * contours[min_depth_seq_int][sequence_num_1].x)
				/ (360 + angle_set[sequence_num_2] - angle_set[sequence_num_1]));
			tem_y = int(((i - angle_set[sequence_num_1]) * contours[min_depth_seq_int][sequence_num_2].y + (360 - i + angle_set[sequence_num_2]) * contours[min_depth_seq_int][sequence_num_1].y)
				/ (360 + angle_set[sequence_num_2] - angle_set[sequence_num_1]));

		}

		Point tem = Point(tem_x, tem_y);
		double dis = sqrt((tem_x - centroid.x)*(tem_x - centroid.x) + (tem_y - centroid.y)*(tem_y - centroid.y));
		point_contours.push_back(tem);
		distance.push_back(dis);
		//cv::circle(depth2, tem, 2, Scalar(0, 0, 0));

	}
	//将此轮廓与模板相关

	vector<double> template_dis;

	//计算模板轮廓
	double mean_template = template_standard(x, template_dis);
	std::cout << "mean_template: " << mean_template << endl;

	//将distance写进文本文件
	ofstream file1("E:\\研究生课题\\王汝宁课题\\distance.txt", ios_base::out);
	if (!file1.is_open())
	{
		std::cout << "打开distance.txt文件失败" << endl;
	}

	double sum = 0;
	for (int i = 0; i < distance.size(); i++)
	{
		sum += distance[i];
	}

	double mean = sum / distance.size();
	cout << "mean: " << mean << endl;
	double multiple = mean_template / mean;
	std::cout << "模板轮廓相对此轮廓的尺寸伸缩倍数为: " << multiple << endl;
	for (int i = 0; i < distance.size(); i++)
	{
		distance[i] = distance[i] * multiple;
		//std::cout << distance[i] << " ";
	}

	for (int i = 0; i < distance.size(); i++)
	{
		//sum += distance[i];
		file1 << distance[i] << " ";
	}
	file1.close();

	//std::cout << "distance: " << distance.size() << endl;

	double template_dis_sum = 0;
	double distance_sum = 0;
	for (int i = 0; i < template_dis.size(); i++)
	{
		template_dis_sum += (template_dis[i] - mean_template) * (template_dis[i] - mean_template);
		//distance_sum += (distance[i]-mean) * (distance[i]-mean);
		//std::cout << template_dis [i] << " ";
	}
	vector<double> correlation;


	for (int i = -180; i < 180; i++)
	{
		vector<double> relative;
		for (int j = 0; j < distance.size(); j++)
		{
			int temp = j - i;
			if (temp < 0)
			{
				temp = temp + 360;
			}
			else
			{
				temp = temp % 360;
			}
			relative.push_back(distance[temp]);
		}
		sum = 0;
		double sum_relative = 0;
		for (int k = 0; k < relative.size(); k++)
		{
			sum += (template_dis[k] - mean_template) * (relative[k] - mean);
			//sum += relative[k] * relative[k];
			sum_relative += (relative[k] - mean*multiple) * (relative[k] - mean*multiple);

		}
		sum = sum / sqrt(template_dis_sum * sum_relative);
		correlation.push_back(sum);
	}
	double correlation_pra = 0;
	//std::cout << "相关函数值为：" << endl;
	for (int i = 0; i < correlation.size(); i++)
	{
		if (correlation_pra < correlation[i])
			correlation_pra = correlation[i];
		//std::cout << correlation[i] << endl;
	}

	
	std::cout << "最大相关系数：" << correlation_pra << endl;
	vector<double>::iterator k = find(correlation.begin(), correlation.end(), correlation_pra);

	int angle_rolate = k - correlation.begin() - 180;

	std::cout << "此时轮廓旋转角度为：" << angle_rolate << endl; 
	
	
	double *correlation_value = new double[2];
	correlation_value[0] = correlation_pra;
	correlation_value[1] = double(angle_rolate);
	return correlation_value;
}



