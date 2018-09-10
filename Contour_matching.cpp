#include "calculate.h"
using namespace std;
using namespace cv;

double calculate_euclidean_distance(vector<double> &distance, vector<double> &template_distance){
	int size = distance.size(); double sum = 0;
	for (int i = 0; i < size; i++){
		sum += (distance[i] - template_distance[i]) * (distance[i] - template_distance[i]);
		//cout << distance[i] << " " << template_distance[i] << "    ";
	}
	double euclidean_distance = sum / size;
	//cout << euclidean_distance << "  ";
	return euclidean_distance;
}
double calculate_consine_distance(vector<double> &distance, vector<double> &template_distance){
	int size = distance.size(); double sum1 = 0, sum2 = 0, sum3 = 0;
	for (int i = 0; i < size; i++){
		sum1 += (distance[i]) * (template_distance[i]);
		sum2 += (distance[i]) * (distance[i]);
		sum3 += (template_distance[i]) * (template_distance[i]);
	}
	double euclidean_distance = sum1 / sqrt(sum2 * sum3);
	//cout << euclidean_distance << "  ";
	return euclidean_distance;
}

void set_correlation_pra(vector<double> &correlation, double *correlation_pra, int type){
	int k = 0; double m;
	if (type == 1){
		double max = -5;
		for (int i = 0; i < correlation.size(); i++){
			if (max < correlation[i]){
				max = correlation[i]; k = i;
			}
		}
		m = max;
	}
	if (type == 0){
		double min = 5;
		for (int i = 0; i < correlation.size(); i++){
			if (min > correlation[i]){
				min = correlation[i]; k = i;
			}
		}
		m = min;
	}
	correlation_pra[0] = m; correlation_pra[1] = k;
}

double * Contours_matching(const vector<vector<Point>> &contours, Mat &depth_gray, int min_depth_seq_int, template_type x)
{
	Moments target_moment;
	target_moment = moments(contours[min_depth_seq_int], false);
	int cx = int(target_moment.m10 / target_moment.m00);
	int cy = int(target_moment.m01 / target_moment.m00);
	Point centroid = Point(cx, cy);
	
	//画出形心
	cv::circle(depth_gray, centroid, 2, Scalar(0, 0, 0));
	for (int i = 0; i < contours[min_depth_seq_int].size(); i++)
	{
		//circle(depth2, contours[min_depth_seq_int][i], 2, Scalar(0, 0, 0));
	}

	//对轮廓上每一点，求其与形心连线与水平线的夹角
	vector<double> angle_set;
	//std::cout << "size: " << contours[min_depth_seq_int].size() << endl;
	for (int i = 0; i < contours[min_depth_seq_int].size(); i++)
	{
		angle_set.push_back(calculate_angle(centroid, contours[min_depth_seq_int][i]));
		//
	}
	vector<double> angle_set_copy = angle_set;
	sort(angle_set_copy.begin(), angle_set_copy.end());
	//for (int i = 0; i < angle_set_copy.size(); i++)
	//{
	//	std::cout << angle_set_copy[i] << " ";
	//}
	//将轮廓内点全部重新排序，按照角度从0到360度，逆时针旋转,并构建距离的序列
	vector<Point> point_contours;
	vector<double> distance;
	double angle_step = 0.5;
	for (double i = 0; i < 360; i = i + angle_step)
	{
		size_t sequence_num_1; size_t sequence_num_2; int tem_x; int tem_y; bool label = false;
		vector<double>::iterator angle_iterator_1; vector<double>::iterator angle_iterator_2;
		for (sequence_num_1 = 0; sequence_num_1<angle_set_copy.size(); sequence_num_1++){
			if (angle_set_copy[sequence_num_1] > i){
				label = true; break;
			}
		}
		//std::cout << angle_set_copy[0] << std::endl;

		if (label == true)
		{
			//std::cout << sequence_num_1 << endl;
			sequence_num_1 = sequence_num_1 - 1;
			angle_iterator_1 = find(angle_set.begin(), angle_set.end(), angle_set_copy[sequence_num_1]);
			angle_iterator_2 = find(angle_set.begin(), angle_set.end(), angle_set_copy[sequence_num_1 + 1]);
			sequence_num_2 = angle_iterator_2 - angle_set.begin();
			sequence_num_1 = angle_iterator_1 - angle_set.begin();

			//cout << angle_set[sequence_num_1] << "  " << angle_set[sequence_num_2];
			tem_x = int(((i - angle_set[sequence_num_1]) * contours[min_depth_seq_int][sequence_num_2].x + (-i + angle_set[sequence_num_2]) * contours[min_depth_seq_int][sequence_num_1].x)
				/ (angle_set[sequence_num_2] - angle_set[sequence_num_1]));
			tem_y = int(((i - angle_set[sequence_num_1]) * contours[min_depth_seq_int][sequence_num_2].y + (-i + angle_set[sequence_num_2]) * contours[min_depth_seq_int][sequence_num_1].y)
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
	for (int i = 0; i < template_dis.size(); i++){
		template_dis[i] /= mean_template;
	}
	mean_template = 1.0;
	//std::cout << "mean_template: " << mean_template << endl;

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
	//cout << "mean: " << mean << endl;
	double multiple = mean_template / mean;
	std::cout << "模板轮廓相对此轮廓的尺寸伸缩倍数为: " << multiple << endl;
	for (int i = 0; i < distance.size(); i++)
	{
		distance[i] = distance[i] /mean;
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
		//template_dis_sum += (template_dis[i]) * (template_dis[i]);
		//distance_sum += (distance[i]-mean) * (distance[i]-mean);
		//std::cout << template_dis [i] << " ";
	}
	vector<double> correlation;
	vector<double> cosine_correlation;
	vector<double> euclidiean_correlation;
	for (double i = -180; i < 180; i = i + angle_step)
	{
		int k = i / angle_step;
		int count_angle = 360 / angle_step;
		vector<double> relative;
		for (int j = 0; j < distance.size(); j++)
		{
			int temp = j - k;
			if (temp < 0)
			{
				temp = temp + count_angle;
			}
			else
			{
				temp = temp % count_angle;
			}
			relative.push_back(distance[temp]);
		}
		sum = 0;
		double sum_relative = 0;
		for (int k = 0; k < relative.size(); k++)
		{
			sum += (template_dis[k] - mean_template) * (relative[k] - 1);
			//sum += (template_dis[k]) * (relative[k]);
			//sum += relative[k] * relative[k];
			sum_relative += (relative[k] - 1) * (relative[k] - 1);
			//sum_relative += (relative[k]) * (relative[k]);
		}
		sum = sum / sqrt(template_dis_sum * sum_relative);
		correlation.push_back(sum);
		double tem = calculate_euclidean_distance(template_dis, relative);
		euclidiean_correlation.push_back(tem);
		double tem_cosine = calculate_consine_distance(template_dis, relative);
		cosine_correlation.push_back(tem_cosine);
	}
	double correlation_pra[2] = { 0, 0 };
	set_correlation_pra(correlation, correlation_pra,1);
	int angle_rolate = correlation_pra[1]*angle_step - 180;

	double euclidiean_pra[2] = { 0, 0 };
	set_correlation_pra(euclidiean_correlation, euclidiean_pra,0);
	cout << endl;
	std::cout << "欧氏距离：" << euclidiean_pra[0] << " " << euclidiean_pra[1] * angle_step - 180;
	//std::cout << "此时轮廓旋转角度为：" << angle_rolate << endl; 
	double cosine_pra[2] = { 0, 0 };
	set_correlation_pra(cosine_correlation, cosine_pra, 1);
	cout << endl;
	std::cout << "余弦距离：" << cosine_pra[0] << " " << cosine_pra[1] * angle_step - 180;

	double *correlation_value = new double[2];
	correlation_value[0] = correlation_pra[0];
	correlation_value[1] = double(angle_rolate);
	return correlation_value;
}



