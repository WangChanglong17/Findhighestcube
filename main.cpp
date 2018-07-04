#include <iostream>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <stdio.h>
#include "calculate.h"
using namespace std;
using namespace cv;


int main()
{
	fstream file;
	file.open("E:\\�о�������\\����������\\depth-0604.txt");
	Mat depth = Mat(424, 512, CV_16SC1);
	
	for (int i = 0; i < 424; i++)
	{
		for (int j = 0; j < 512; j++)
		{
			file >> depth.at<short>(i,j);
		}
	}
	//163,170,315,374;
	Mat depth3 = Mat(424, 512, CV_16SC1);
	for (int i = 0; i < 424; i++)
	{
		for (int j = 0; j < 512; j++)
		{
			if (i>170 && i<315 && j>170 && j<360)
			depth3.at<short>(i, j) = depth.at<short>(i, j);
			else
				depth3.at<short>(i, j) = depth.at<short>(170,170);
		}
	}

	Mat depth2gray;
	normalize(depth3, depth2gray, 0, 255, CV_MINMAX);

	//cout << depth2gray.cols << depth2gray.rows << endl;

	Mat depth2 = Mat(424, 512, CV_8UC1);
	for (int i = 0; i < 424; i++)
	{
		for (int j = 0; j < 512; j++)
		{
			depth2.at<uchar>(i, j) = uchar(depth2gray.at<short>(i,j));
		}
	}

	

	imwrite("E:\\�о�������\\����������\\depth2gray1.png", depth2);
	//normalize(depth2gray, depth2gray, 0, 1, CV_MINMAX);
	//cout << depth2gray << endl;

	//��depth �� depth2 ����Ŵ���ߺ���ģ��ƥ��ľ��� 
	Mat dst_depth, dst_depth2;
	resize(depth, depth, Size(depth.cols * 1, depth.rows * 1), 0, 0, INTER_LINEAR);
	resize(depth2, depth2, Size(depth2.cols * 1, depth2.rows * 1), 0, 0, INTER_LINEAR);

	Mat aa;
	applyColorMap(depth2, aa, cv::COLORMAP_JET);

	cv::namedWindow("aa", WINDOW_AUTOSIZE);
	cv::imshow("aa", aa);
	Mat imggaussian,imgcanny;
	GaussianBlur(aa, imggaussian, Size(3, 3), 0, 0);
	Canny(imggaussian, imgcanny, 30, 40, 3);
	

	vector<vector<Point>> contours;
	vector<Vec4i> hierarchy;
	//���������
	vector<double> interArea;
	Mat imgDilate;
	Mat element = getStructuringElement(MORPH_DILATE, Size(5, 5));
	dilate(imgcanny, imgDilate, element);
	
	Mat thresholdimg;
	threshold(imgDilate, thresholdimg, 10, 255, THRESH_BINARY_INV);

	findContours(imgDilate, contours, hierarchy, RETR_TREE, CHAIN_APPROX_NONE);
	std::cout <<"��⵽��������Ŀ��"<<contours.size() << endl;

	vector<Mat> depthcopies; vector<double> depth_means; Mat threshold_temp,depth_multiply;
	//����depth2
	
	for (int i = 0; i < contours.size(); i++)
	{
		interArea.push_back(contourArea(contours[i]));
		Mat depth2copy; depth2.copyTo(depth2copy);
		depthcopies.push_back(depth2copy);
		drawContours(depthcopies[i], contours, i, cv::Scalar::all(255), CV_FILLED);
		threshold(depthcopies[i], threshold_temp, 254, 255, THRESH_BINARY);
		multiply(depth, threshold_temp, depth_multiply, 1, CV_32F);
		double depth_top = mean(depth_multiply)[0] / mean(threshold_temp)[0];
		depth_means.push_back(depth_top);
	}
	std::cout << endl;
	std::cout << "���������ڲ������ƽ�����ֵΪ��";
	for (int i = 0; i < depth_means.size(); i++)
	{
		std::cout << depth_means[i] << " ";
	}
	std::cout << endl; std::cout << endl;

	vector<double> depth_means_temp = depth_means;
	sort(depth_means_temp.begin(), depth_means_temp.end());
	vector<double>::iterator min_depth_seq = find(depth_means.begin(), depth_means.end(), depth_means_temp[0]);
	int min_depth_seq_int = min_depth_seq - depth_means.begin();
	std::cout << "��" << min_depth_seq_int +1 << "������ΪҪ��Ѱ�ĺ��Ӷ�������" << endl;
	std::cout << "�����Ϊ��" << interArea[min_depth_seq_int] << endl;



	double *correlation_value1 = Contours_matching(contours, min_depth_seq_int, square_);
	std::cout <<"����õ��볤����ƥ�����ز���ֵΪ��"<< correlation_value1[0] << " " << correlation_value1[1] << endl;
	delete[]correlation_value1;

	double *correlation_value2 = Contours_matching(contours, min_depth_seq_int, rectangle_);
	std::cout << "����õ���������ƥ�����ز���ֵΪ��" << correlation_value2[0] << " " << correlation_value2[1] << endl;
	delete[]correlation_value2;

	double *correlation_value3 = Contours_matching(contours, min_depth_seq_int, triangle_);
	std::cout << "����õ�����������ƥ�����ز���ֵΪ��" << correlation_value3[0] << " " << correlation_value3[1] << endl;
	delete[]correlation_value3;

	//�ں��Ӷ���������ȡ1000���������ά���������
	Mat positions = Mat(400, 3, CV_16SC1);
	int m = 0, n = 0;
	for (int i = 170; i < 315; i++)
	{
		for (int j = 170; j < 360; j++)
		{
			if (pointPolygonTest(contours[min_depth_seq_int], Point(j, i), false) == 1 && pointPolygonTest(contours[min_depth_seq_int], Point(j - 7, i), false) == 1 
				&& pointPolygonTest(contours[min_depth_seq_int], Point(j, i - 5), false) == 1 && pointPolygonTest(contours[min_depth_seq_int], Point(j+12, i), false) == 1 && 
				pointPolygonTest(contours[min_depth_seq_int], Point(j, i+5), false) == 1)
			{
				
				{
					positions.at<short>(m, 0) = j*depth.at<short>(i, j); positions.at<short>(m, 1) = i*depth.at<short>(i, j); positions.at<short>(m, 2) = depth.at<short>(i, j);
					m++;
					if (m == 400) break;
				}
			}					
		}
		if (m == 400) break;
	}
	//����ڲξ���
	Mat inter_parameter = Mat::zeros(3, 3, CV_64FC1);
	inter_parameter.at<double>(0, 0) = 364.2293; inter_parameter.at<double>(0, 2) = 256; 
	inter_parameter.at<double>(1, 1) = 360.8003; inter_parameter.at<double>(1, 2) = 212;
	inter_parameter.at<double>(2, 2) = 1;

	Mat inter_parameter_inversion = inter_parameter.inv();

	std::cout << inter_parameter_inversion << endl;
	//cout << positions << endl;

	Mat positions_t;
	cv::transpose(positions, positions_t);
	positions.convertTo(positions, CV_64F);
	positions_t.convertTo(positions_t, CV_64F);
	//����ڲξ�����棬�õ��������ϵ�µ�X,Y,Z	
	Mat positions_c = inter_parameter_inversion * positions_t;

	double xx[3] = {0,0,0};
	for (int i = 0; i < positions_c.rows; i++)
	{
		for (int j = 0; j < positions_c.cols; j++)
		{
			xx[i] = xx[i] + positions_c.at<double>(i, j);
		}
	}
	double xx_mean[3] = { 0, 0, 0 };
	xx_mean[0] = xx[0] / positions_c.cols; xx_mean[1] = xx[1] / positions_c.cols; xx_mean[2] = xx[2] / positions_c.cols;

	for (int i = 0; i < positions_c.rows; i++)
	{
		for (int j = 0; j < positions_c.cols; j++)
		{
			positions_c.at<double>(i, j) = positions_c.at<double>(i, j) - xx_mean[i];
		}
	}

	Mat positions_c_t;
	cv::transpose(positions_c, positions_c_t);

	//������������ı��ļ�
	ofstream outFile("E:\\�о�������\\����������\\top_positions.txt", ios_base::out);  //���½��򸲸Ƿ�ʽд��  
	if (!outFile.is_open())
	{
		std::cout << "���ļ�ʧ��" << endl;
		return false;
	}
	
	for (int i = 0; i < 400; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			 double t = positions_c_t.at<double>(i, j);
			 outFile << t << "\t";
		}
		outFile << endl;
	}

	
	Mat PTP(3,3,CV_64FC1);
	PTP = positions_c * positions_c_t;
	Mat values, vectors;
	cv::eigen(PTP, values, vectors);

	std::cout << "�Ǹ���������Ϊ��"<< PTP << endl;
	std::cout << "������ֵΪ "<<values << endl;
	std::cout << "����������Ϊ "<<vectors << endl;
	//����С����ֵ�����Ӧ��������������Ϊƽ�淨����
	double f_vector[3];
	if (values.at<double>(0,0) > values.at<double>(1,0))
	{
		if (values.at<double>(1, 0) > values.at<double>(2, 0))
		{
			f_vector[0] = vectors.at<double>(2, 0); f_vector[1] = vectors.at<double>(2, 1); f_vector[2] = vectors.at<double>(2, 2);
		}
		else{
			f_vector[0] = vectors.at<double>(1, 0); f_vector[1] = vectors.at<double>(1, 1); f_vector[2] = vectors.at<double>(1, 2);
		}
		
	}
	else
	{
		if (values.at<double>(0, 0) > values.at<double>(2, 0))
		{
			f_vector[0] = vectors.at<double>(2, 0); f_vector[1] = vectors.at<double>(2, 1); f_vector[2] = vectors.at<double>(2, 2);
		}
		else{
			f_vector[0] = vectors.at<double>(0, 0); f_vector[1] = vectors.at<double>(0, 1); f_vector[2] = vectors.at<double>(0, 2);
		}
		
	}

	std::cout << "ƽ�淨����Ϊ��" << f_vector[0] << " " << f_vector[1] <<" "<< f_vector[2];


	
	cv::namedWindow("imgcanny", WINDOW_AUTOSIZE);
	cv::imshow("imgcanny", imgcanny);
	cv::namedWindow("imgDilate", WINDOW_AUTOSIZE);
	cv::imshow("imgDilate", thresholdimg);
	cv::namedWindow("depth2gray", WINDOW_AUTOSIZE);
	cv::imshow("depth2gray", depth2);
	


	cv::namedWindow("depthcopies", WINDOW_AUTOSIZE);
	cv::imshow("depthcopies", depthcopies[min_depth_seq_int]);

	/*
	namedWindow("threshold_temp", WINDOW_AUTOSIZE);
	imshow("threshold_temp", threshold_temp);
	*/
	cvWaitKey(0);
	system("PAUSE");
	return 0;
}