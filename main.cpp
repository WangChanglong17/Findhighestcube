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
	file.open("E:\\研究生课题\\王汝宁课题\\depth-0604.txt");
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

	

	imwrite("E:\\研究生课题\\王汝宁课题\\depth2gray1.png", depth2);
	//normalize(depth2gray, depth2gray, 0, 1, CV_MINMAX);
	//cout << depth2gray << endl;

	//把depth 和 depth2 矩阵放大，提高后续模板匹配的精度 
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
	//轮廓内面积
	vector<double> interArea;
	Mat imgDilate;
	Mat element = getStructuringElement(MORPH_DILATE, Size(5, 5));
	dilate(imgcanny, imgDilate, element);
	
	Mat thresholdimg;
	threshold(imgDilate, thresholdimg, 10, 255, THRESH_BINARY_INV);

	findContours(imgDilate, contours, hierarchy, RETR_TREE, CHAIN_APPROX_NONE);
	std::cout <<"检测到的轮廓数目："<<contours.size() << endl;

	vector<Mat> depthcopies; vector<double> depth_means; Mat threshold_temp,depth_multiply;
	//复制depth2
	
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
	std::cout << "各个轮廓内部区域的平均深度值为：";
	for (int i = 0; i < depth_means.size(); i++)
	{
		std::cout << depth_means[i] << " ";
	}
	std::cout << endl; std::cout << endl;

	vector<double> depth_means_temp = depth_means;
	sort(depth_means_temp.begin(), depth_means_temp.end());
	vector<double>::iterator min_depth_seq = find(depth_means.begin(), depth_means.end(), depth_means_temp[0]);
	int min_depth_seq_int = min_depth_seq - depth_means.begin();
	std::cout << "第" << min_depth_seq_int +1 << "个轮廓为要找寻的盒子顶部区域" << endl;
	std::cout << "其面积为：" << interArea[min_depth_seq_int] << endl;



	double *correlation_value1 = Contours_matching(contours, min_depth_seq_int, square_);
	std::cout <<"计算得到与长方形匹配的相关参数值为："<< correlation_value1[0] << " " << correlation_value1[1] << endl;
	delete[]correlation_value1;

	double *correlation_value2 = Contours_matching(contours, min_depth_seq_int, rectangle_);
	std::cout << "计算得到与正方形匹配的相关参数值为：" << correlation_value2[0] << " " << correlation_value2[1] << endl;
	delete[]correlation_value2;

	double *correlation_value3 = Contours_matching(contours, min_depth_seq_int, triangle_);
	std::cout << "计算得到与正三角形匹配的相关参数值为：" << correlation_value3[0] << " " << correlation_value3[1] << endl;
	delete[]correlation_value3;

	//在盒子顶部区域中取1000个点放入三维坐标矩阵中
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
	//相机内参矩阵
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
	//左乘内参矩阵的逆，得到相机坐标系下的X,Y,Z	
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

	//将矩阵输出到文本文件
	ofstream outFile("E:\\研究生课题\\王汝宁课题\\top_positions.txt", ios_base::out);  //按新建或覆盖方式写入  
	if (!outFile.is_open())
	{
		std::cout << "打开文件失败" << endl;
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

	std::cout << "非负正定矩阵为："<< PTP << endl;
	std::cout << "其特征值为 "<<values << endl;
	std::cout << "其特征向量为 "<<vectors << endl;
	//求最小特征值及其对应的特征向量，即为平面法向量
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

	std::cout << "平面法向量为：" << f_vector[0] << " " << f_vector[1] <<" "<< f_vector[2];


	
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