#include <iostream>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <stdio.h>
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

	Mat imggaussian,imgcanny;
	GaussianBlur(depth2, imggaussian, Size(3, 3), 0, 0);
	Canny(imggaussian, imgcanny, 5, 15, 3);
	

	vector<vector<Point>> contours;
	vector<Vec4i> hierarchy;
	//轮廓内面积
	vector<double> interArea;
	Mat imgDilate;
	Mat element = getStructuringElement(MORPH_DILATE, Size(5, 5));
	dilate(imgcanny, imgDilate, element);
	
	Mat thresholdimg;
	threshold(imgDilate, thresholdimg, 10, 255, THRESH_BINARY_INV);

	findContours(imgDilate, contours, hierarchy, RETR_TREE, CV_CHAIN_APPROX_SIMPLE);
	cout <<"检测到的轮廓数目："<<contours.size() << endl;

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
	cout << endl;
	cout << "各个轮廓内部区域的平均深度值为：";
	for (int i = 0; i < depth_means.size(); i++)
	{
		cout << depth_means[i] << " ";
	}
	cout << endl; cout << endl;

	vector<double> depth_means_temp = depth_means;
	sort(depth_means_temp.begin(), depth_means_temp.end());
	vector<double>::iterator min_depth_seq = find(depth_means.begin(), depth_means.end(), depth_means_temp[0]);
	int min_depth_seq_int = min_depth_seq - depth_means.begin();
	cout << "第" << min_depth_seq_int +1 << "个轮廓为要找寻的盒子顶部区域" << endl;
	cout << "其面积为：" << interArea[min_depth_seq_int] << endl;

	//在盒子顶部区域中取500个点放入三维坐标矩阵中
	Mat positions = Mat(300, 3, CV_16SC1);
	int m = 0;
	for (int i = 170; i < 315; i=i+2)
	{
		for (int j = 170; j < 360; j = j+2)
		{
			if (pointPolygonTest(contours[min_depth_seq_int], Point(j, i), false) == 1)
			{
				positions.at<short>(m, 0) = i; positions.at<short>(m, 1) = j; positions.at<short>(m, 2) = depth.at<short>(i, j);
				m++;
				if (m == 300) break;
			}		
			
		}
		if (m == 300) break;
	}
	//cout << positions << endl;

	Mat positions_t;
	transpose(positions, positions_t);
	positions.convertTo(positions, CV_64F);
	positions_t.convertTo(positions_t, CV_64F);
	Mat positons_t_positions(3,3,CV_64FC1);
	positons_t_positions = positions_t * positions;
	Mat values, vectors;
	eigen(positons_t_positions, values, vectors);

	cout << values << endl;
	cout << vectors << endl;
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

	cout << "平面法向量为：" << f_vector[0] << " " << f_vector[1] <<" "<< f_vector[2];


	/*
	namedWindow("imgcanny", WINDOW_AUTOSIZE);
	imshow("imgcanny", imgcanny);
	namedWindow("imgDilate", WINDOW_AUTOSIZE);
	imshow("imgDilate", thresholdimg);
	namedWindow("depth2gray", WINDOW_AUTOSIZE);
	imshow("depth2gray", depth2);
	*/


	namedWindow("depthcopies", WINDOW_AUTOSIZE);
	imshow("depthcopies", depthcopies[min_depth_seq_int]);

	/*
	namedWindow("threshold_temp", WINDOW_AUTOSIZE);
	imshow("threshold_temp", threshold_temp);
	*/
	cvWaitKey(0);
	system("PAUSE");
	return 0;
}