#include <iostream>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <stdio.h>
#include <array>
#include "calculate.h"
using namespace std;
using namespace cv;

//从txt文件中获取深度矩阵
Mat get_depth(string path,int h, int w)
{
	fstream file;	
	file.open(path);
	Mat depth(h, w, CV_16SC1);
	for (int i = 0; i < h; i++)
	{
		short *depth_ptr = depth.ptr<short>(i);
		for (int j = 0; j < w; j++)
		{
			file >> depth_ptr[j];
		}
	}
	return depth;
}
//输出txt
void out_points(Mat &positions_c_t, string path, int num)
{
	ofstream outFile(path, ios_base::out);  //按新建或覆盖方式写入  
	if (!outFile.is_open())
	{
		std::cout << "打开文件失败" << endl;
		return;
	}
	for (int i = 0; i < num; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			double t = positions_c_t.at<double>(i, j);
			outFile << t << "\t";
		}
		outFile << endl;
	}
	outFile.close();
}

//只获取感兴趣区域的深度值，其他置为统一值
Mat get_target_area(const Mat &depth,int min_row, int max_row, int min_col, int max_col)
{
	Mat depth3 = Mat(depth.rows, depth.cols, CV_16SC1);
	for (int i = 0; i < depth.rows; i++)
	{
		for (int j = 0; j < depth.cols; j++)
		{
			if (i>min_row && i<max_row && j>min_col && j<max_col)
				depth3.at<short>(i, j) = depth.at<short>(i, j);
			else
				depth3.at<short>(i, j) = 0;
		}
	}
	return depth3;
}

Mat get_gray_depth(Mat &depth_roi_area)
{
	Mat depth_normalize;
	normalize(depth_roi_area, depth_normalize, 0, 255, CV_MINMAX);

	Mat depth_gray = Mat(depth_roi_area.rows, depth_roi_area.cols, CV_8UC1);
	for (int i = 0; i < depth_gray.rows; i++)
	{
		uchar *depth_gray_ptr = depth_gray.ptr<uchar>(i);
		ushort *depth_normalize_ptr = depth_normalize.ptr<ushort>(i);
		for (int j = 0; j < depth_gray.cols; j++)
		{
			depth_gray_ptr[j] = uchar(depth_normalize_ptr[j]);
		}
	}
	imwrite("E:\\研究生课题\\王汝宁课题\\depth2gray1.png", depth_gray);
	return depth_gray;
}
//检测图像中所有轮廓
vector<vector<Point>> get_depth_contours(const Mat &depth_gray)
{
	Mat depth_color;
	applyColorMap(depth_gray, depth_color, cv::COLORMAP_JET);

	cv::namedWindow("depth_color", WINDOW_AUTOSIZE);
	cv::imshow("depth_color", depth_color);
	Mat imggaussian, imgcanny;
	GaussianBlur(depth_color, imggaussian, Size(3, 3), 0, 0);
	Canny(imggaussian, imgcanny, 30, 40, 3);

	vector<vector<Point>> contours;
	vector<Vec4i> hierarchy;
	
	Mat imgDilate;
	Mat element = getStructuringElement(MORPH_DILATE, Size(5, 5));
	dilate(imgcanny, imgDilate, element);

	Mat thresholdimg;
	threshold(imgDilate, thresholdimg, 10, 255, THRESH_BINARY_INV);

	findContours(imgDilate, contours, hierarchy, RETR_TREE, CHAIN_APPROX_NONE);
	std::cout << "检测到的轮廓数目：" << contours.size() << endl;
	cv::namedWindow("imgcanny", WINDOW_AUTOSIZE);
	cv::imshow("imgcanny", imgcanny);
	cv::namedWindow("imgDilate", WINDOW_AUTOSIZE);
	cv::imshow("imgDilate", thresholdimg);
	
	return contours;
}

//得到轮廓区域深度最小的轮廓序号
int get_min_depth_seq(vector<vector<Point>> &contours, Mat &depth,Mat depth_gray)
{
	vector<Mat> depthcopies; vector<double> depth_means; Mat threshold_temp, depth_multiply;
	for (int i = 0; i < contours.size(); i++)
	{
		Mat depth2copy; depth_gray.copyTo(depth2copy);
		depthcopies.push_back(depth2copy);
		drawContours(depthcopies[i], contours, i, cv::Scalar::all(255), CV_FILLED);
		threshold(depthcopies[i], threshold_temp, 254, 255, THRESH_BINARY);
		multiply(depth, threshold_temp, depth_multiply, 1, CV_64F);
		double depth_top = mean(depth_multiply)[0] / mean(threshold_temp)[0];
		depth_means.push_back(depth_top);
	/*	cv::namedWindow("depth_gray", WINDOW_AUTOSIZE);
		cv::imshow("depth_gray", depthcopies[i]);
		cv::namedWindow("depth_thre", WINDOW_AUTOSIZE);
		cv::imshow("depth_thre", threshold_temp);
		cvWaitKey(0);*/
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
	std::cout << "第" << min_depth_seq_int + 1 << "个轮廓为要找寻的盒子顶部区域" << endl;
	drawContours(depth_gray, contours, min_depth_seq_int, cv::Scalar::all(255), CV_FILLED);
	//double area = contourArea(contours[min_depth_seq_int]);
	//cout << "area: " << area;
	int num_count = 0;
	ofstream outFile("E:\\研究生课题\\王汝宁课题\\rect.txt", ios_base::out);  //按新建或覆盖方式写入  
	if (!outFile.is_open())
	{
		std::cout << "打开文件失败" << endl;
		return 0;
	}
	for (int i = 0; i < 424; i++){
		for (int j = 0; j < 512; j++){
			if (pointPolygonTest(contours[min_depth_seq_int], Point(j, i), false) == 1){
				{
					num_count++;
					outFile << j << " " << i << " " << depth.at<short>(i, j) << endl;;
				}
			}
		}
	}
	//cout << "数量： " << num_count;
	return min_depth_seq_int;
}

Mat get_positions_c(vector<vector<Point>> &contours, Mat &depth, Mat &inter_parameter,int min_depth_seq_int, int points_num, int min_row, int max_row, int min_col, int max_col,int &m)
{
	//在盒子顶部区域中取一定数目点放入三维坐标矩阵中
	
	vector<std::array<double,3>> positons_vec;
	int n = 0;
	for (int i = min_row; i < max_row; i++){
		for (int j = min_col; j < max_col; j++){
			//if (pointPolygonTest(contours[min_depth_seq_int], Point(j, i), false) == 1 && pointPolygonTest(contours[min_depth_seq_int], Point(j - 7, i), false) == 1
			//	&& pointPolygonTest(contours[min_depth_seq_int], Point(j, i - 5), false) == 1 && pointPolygonTest(contours[min_depth_seq_int], Point(j + 12, i), false) == 1 &&
			//	pointPolygonTest(contours[min_depth_seq_int], Point(j, i + 5), false) == 1)
			if (pointPolygonTest(contours[min_depth_seq_int], Point(j, i), false) == 1){
				{
					array<double,3> point_temp={ j*depth.at<short>(i, j), i*depth.at<short>(i, j), depth.at<short>(i, j) };
					//positions.at<short>(m, 0) = j*depth.at<short>(i, j); positions.at<short>(m, 1) = i*depth.at<short>(i, j); positions.at<short>(m, 2) = depth.at<short>(i, j);
					positons_vec.push_back(point_temp);
					m++;
					//if (m == points_num) break;
				}
			}
		}
		//if (m == points_num) break;
	}
	//cout << "m: " << m << endl;
	Mat positions = Mat(m, 3, CV_64FC1);
	for (int i = 0; i < m; i++){
		for (int j = 0; j < 3; j++){
			positions.at<double>(i, j) = positons_vec[i][j];
		}
	}
	
	out_points(inter_parameter, "E:\\研究生课题\\王汝宁课题\\inter_parameter.txt", 3);
	Mat inter_parameter_inversion = inter_parameter.inv();
	//cout << positions << endl;
	std::cout << "................................................................................." << endl;
	Mat positions_t;
	cv::transpose(positions, positions_t);
	positions.convertTo(positions, CV_64F);
	positions_t.convertTo(positions_t, CV_64F);
	out_points(positions, "E:\\研究生课题\\王汝宁课题\\positions_depth.txt", m);
	//左乘内参矩阵的逆，得到相机坐标系下的X,Y,Z	
	Mat positions_c = inter_parameter_inversion * positions_t;
	return positions_c;
}

//获取形心坐标
double*  get_centroid(Mat &positions_c)
{
	if (positions_c.rows == 3)
	{
		double xx[3] = { 0, 0, 0 };
		for (int i = 0; i < positions_c.rows; i++)
		{
			for (int j = 0; j < positions_c.cols; j++)
			{
				xx[i] = xx[i] + positions_c.at<double>(i, j);
			}
		}
		static double xx_mean[3] = { 0, 0, 0 };
		xx_mean[0] = xx[0] / positions_c.cols; xx_mean[1] = xx[1] / positions_c.cols; xx_mean[2] = xx[2] / positions_c.cols;
		return xx_mean;
	}
	else
	{
		return NULL;
	}
}



double *get_normal_vector(Mat &positions_c)
{
	Mat positions_c_t;
	cv::transpose(positions_c, positions_c_t);
	Mat PTP(3, 3, CV_64FC1);
	PTP = positions_c * positions_c_t;
	Mat values, vectors;
	cv::eigen(PTP, values, vectors);
	/*std::cout << "非负正定矩阵为：" << PTP << endl;
	std::cout << "其特征值为 " << values << endl;
	std::cout << "其特征向量为 " << vectors << endl;*/

	//求最小特征值及其对应的特征向量，即为平面法向量
	static double f_vector[3];
	f_vector[0] = vectors.at<double>(2, 0); f_vector[1] = vectors.at<double>(2, 1); f_vector[2] = vectors.at<double>(2, 2);
	return f_vector;
}
//由法向量得到一组正交单位阵
Mat get_orthogonal_array(double *f_vector)
{
	Mat orthogonal_array(3, 3, CV_64FC1);
	double x_temp[3] = { -1, 1, 0 };
	x_temp[2] = (0 - (x_temp[0] * f_vector[0] + x_temp[1] * f_vector[1])) / f_vector[2];
	double sum_x = sqrt(x_temp[0] * x_temp[0] + x_temp[1] * x_temp[1] + x_temp[2] * x_temp[2]);
	for (int k = 0; k < 3; k++){
		x_temp[k] = x_temp[k] / sum_x;
	}
	Mat temp_x_z(2, 3, CV_64FC1);
	for (int j = 0; j < 3; j++){
		temp_x_z.at<double>(0, j) = f_vector[j];
		temp_x_z.at<double>(1, j) = x_temp[j];
	}
	Mat temp_x_z_t;
	cv::transpose(temp_x_z, temp_x_z_t);
	Mat temp(3, 3, CV_64FC1);
	temp = temp_x_z_t * temp_x_z;
	Mat values, vectors;
	cv::eigen(temp, values, vectors);
	//cout << "y: " << vectors.at<double>(2, 0) <<" "<< vectors.at<double>(2, 1) <<" "<< vectors.at<double>(2, 2) << endl;
	for (int i = 0; i < 3; i++)
	{
		orthogonal_array.at<double>(i, 0) = x_temp[i];
		orthogonal_array.at<double>(i, 1) = vectors.at<double>(2, i);
		orthogonal_array.at<double>(i, 2) = f_vector[i];
	}
	return orthogonal_array;
}
//由旋转矩阵反算矫正后的图像坐标
Mat get_rotate_pic(Mat &rotate, Mat &positions_c, Mat & inter_parameter,double *centroid,int &m){
	Mat rotate_positions_c = rotate * positions_c;
	Mat rotate_t;
	cv::transpose(rotate_positions_c, rotate_t);
	out_points(rotate_t, "E:\\研究生课题\\王汝宁课题\\rotate_positions.txt", m);

	for (int i = 0; i < rotate_positions_c.rows; i++){
		for (int j = 0; j < rotate_positions_c.cols; j++){
			rotate_positions_c.at<double>(i, j) += centroid[i];
		}
	}
	Mat rotate_points = inter_parameter * rotate_positions_c;
	int *point_pic_x = new int[rotate_points.cols]; int min_x = INT_MAX;
	int *point_pic_y = new int[rotate_points.cols]; int min_y = INT_MAX;
	Mat rotate_pic = Mat::zeros(424, 512, CV_8UC1);
	for (int j = 0; j < rotate_points.cols; j++){
		if (rotate_points.at<double>(2, j) != 0){
			point_pic_x[j] = int(rotate_points.at<double>(0, j) / rotate_points.at<double>(2, j));
			if (point_pic_x[j] < min_x) min_x = point_pic_x[j];
			point_pic_y[j] = int(rotate_points.at<double>(1, j) / rotate_points.at<double>(2, j));
			if (point_pic_y[j] < min_y) min_y = point_pic_y[j];
		}
	}
	for (int i = 0; i < rotate_points.cols; i++){
		rotate_pic.at<uchar>(point_pic_y[i] - min_y + 100, point_pic_x[i] - min_x + 100) = 255;
	}

	return rotate_pic;
}

double* get_n_vectors(Mat &positions_c){
	//获取形心坐标
	double *centroid = get_centroid(positions_c);
	cout << endl <<"形心坐标： "<< centroid[0] << " " << centroid[1] << " " << centroid[2] << endl;
	//所有点统一减去形心坐标
	for (int i = 0; i < positions_c.rows; i++){
		for (int j = 0; j < positions_c.cols; j++){
			positions_c.at<double>(i, j) = positions_c.at<double>(i, j) - centroid[i];
		}
	}

	Mat positions_c_t;
	cv::transpose(positions_c, positions_c_t);

	//获取三维平面的法向量
	double *f_vector = get_normal_vector(positions_c);
	Mat a(1, 3, CV_64FC1);
	for (int i = 0; i < 3; i++){
		a.at<double>(0, i) = f_vector[i];
	}
	
	return get_normal_vector(positions_c);
}

Mat get_rotate(double *f_vector){
	//由平面法向量计算出两个正交单位向量，组成成交单位矩阵（以列存储）
	Mat orthogonal_array = get_orthogonal_array(f_vector);

	//计算由正交矩阵到标准正交矩阵的旋转矩阵
	Mat standard_orthogonal_array(3, 3, CV_64FC1, Scalar(0));
	for (int i = 0; i < 3; i++){
		standard_orthogonal_array.at<double>(i, i) = 1;
	}
	Mat rotate = orthogonal_array.inv() * standard_orthogonal_array;
	return rotate;
}
int main(){
	int m = 0;
	//深度相机内参矩阵
	Mat inter_parameter = Mat::zeros(3, 3, CV_64FC1);
	inter_parameter.at<double>(0, 0) = 364.2293; inter_parameter.at<double>(0, 2) = 256;
	inter_parameter.at<double>(1, 1) = 360.8003; inter_parameter.at<double>(1, 2) = 212;
	inter_parameter.at<double>(2, 2) = 1;
	//cout <<"深度相机内参矩阵为："<<endl<<endl<< inter_parameter << endl;
	Mat depth = get_depth("E:\\研究生课题\\王汝宁课题\\depth-80.txt", 424, 512);
	//Mat depth = kalman("E:\\研究生课题\\王汝宁课题\\堆叠数据20180910\\4-circle\\depth-");
	Mat depth_f;
	depth.convertTo(depth_f, CV_32FC1);
	Mat depth_f_copy;
	//双边滤波对深度数据进行平滑，同时保证图像边缘易于识别
	//bilateralFilter(depth_f, depth_f_copy, 5, 5, 30);
	//depth_f_copy.convertTo(depth, CV_16SC1);
	//170,315,250,360;
	int min_row = 80; int max_row = 300; int min_col = 180; int max_col = 420;
	Mat depth_roi_area = get_target_area(depth, min_row, max_row, min_col, max_col);

	Mat depth_gray = get_gray_depth(depth_roi_area);

	//把depth 和 depth2 矩阵放大，提高后续模板匹配的精度 
	resize(depth, depth, Size(depth.cols * 1, depth.rows * 1), 0, 0, INTER_LINEAR);
	resize(depth_gray, depth_gray, Size(depth_gray.cols * 1, depth_gray.rows * 1), 0, 0, INTER_LINEAR);

	//检测深度图（只包含感兴趣区域）中所有轮廓
	vector<vector<Point>> contours = get_depth_contours(depth_gray);
	
	//找到最高的盒子顶部表面
	int min_depth_seq_int = get_min_depth_seq(contours, depth, depth_gray);

	double *correlation_value1 = Contours_matching(contours, depth_gray, min_depth_seq_int, square_);
	std::cout << "计算得到与正方形匹配的相关参数值为：" << correlation_value1[0] << " " << correlation_value1[1] << endl << endl;;
	delete[]correlation_value1;

	double *correlation_value2 = Contours_matching(contours, depth_gray, min_depth_seq_int, rectangle_);
	std::cout << "计算得到与长方形匹配的相关参数值为：" << correlation_value2[0] << " " << correlation_value2[1] << endl << endl;;
	delete[]correlation_value2;

	double *correlation_value3 = Contours_matching(contours, depth_gray, min_depth_seq_int, triangle_);
	std::cout << "计算得到与正三角形匹配的相关参数值为：" << correlation_value3[0] << " " << correlation_value3[1] << endl << endl;
	delete[]correlation_value3;

	double *correlation_value4 = Contours_matching(contours, depth_gray, min_depth_seq_int, circle_);
	std::cout << "计算得到与圆匹配的相关参数值为：" << correlation_value4[0] << " " << correlation_value4[1] << endl << endl;;
	delete[]correlation_value4;

	//获取盒子顶部表面区域的点的相机坐标系下的三维坐标，采集num个点
	int num = 1000;
	Mat positions_c = get_positions_c(contours, depth, inter_parameter,min_depth_seq_int, num, min_row, max_row, min_col, max_col,m);

	ransac xxx = ransac(positions_c,6,10000,0.2);//16以下效果较好
	Mat positions_c_temp = xxx.get_ransac_points();
	cout << endl << "Ransac之前平面点数目： " << positions_c.cols << endl;

	Mat positions_c_temp_t;
	cv::transpose(positions_c_temp, positions_c_temp_t);
	out_points(positions_c_temp_t, "E:\\研究生课题\\王汝宁课题\\top_positions_ransac.txt", positions_c_temp_t.rows);
	cout <<endl<<"ransac后平面点数量："<< positions_c_temp.cols << endl;
	double *f_vector0 = get_n_vectors(positions_c_temp);
	std::cout << endl << "ransac后平面单位法向量为：" << f_vector0[0] << " " << f_vector0[1] << " " << f_vector0[2] <<endl;
	double *centroid = get_centroid(positions_c);//获取形心坐标
	double *f_vector = get_n_vectors(positions_c);
	std::cout<< endl << "ransac前平面单位法向量为：" << f_vector[0] << " " << f_vector[1] << " " << f_vector[2] << endl;

	Mat rotate = get_rotate(f_vector);
	cout <<endl <<"旋转矩阵："<<endl<< rotate << endl;

	//由旋转矩阵反算矫正后的图像坐标
	Mat rotate_pic = get_rotate_pic(rotate, positions_c, inter_parameter, centroid,m);
	//........................................
	
	Mat element = getStructuringElement(MORPH_DILATE, Size(5, 5));
	Mat rotate_temp;
	dilate(rotate_pic, rotate_temp, element);
	//Mat rect_rotate = rotate_temp(Rect(0, 0,  400, 400));
	//resize(rect_rotate, rect_rotate, Size(800, 800), 0, 0, INTER_LINEAR);
	//检测深度图（只包含感兴趣区域）中所有轮廓
	vector<vector<Point>> contours2 = get_depth_contours(rotate_temp);
	drawContours(rotate_temp, contours2, 0, cv::Scalar::all(255), CV_FILLED);
	Mat dstImage;
	medianBlur(rotate_temp, dstImage, 3);//中值滤波，对轮廓边缘进行平滑处理，减少噪声
	vector<vector<Point>> contours3 = get_depth_contours(dstImage);
	double *correlation_value23 = Contours_matching(contours3, rotate_temp, 0, square_);
	std::cout << "计算得到与正方形匹配的相关参数值为：" << correlation_value23[0] << " " << correlation_value23[1] << endl << endl;;
	delete[]correlation_value23;

	double *correlation_value22 = Contours_matching(contours3, rotate_temp, 0, circle_);
	std::cout << "计算得到与圆匹配的相关参数值为：" << correlation_value22[0] << " " << correlation_value22[1] << endl << endl;;

	cvNamedWindow("ddd",WINDOW_AUTOSIZE);
	imshow("ddd", dstImage);
	//............................................
	cvNamedWindow("gray", WINDOW_AUTOSIZE);
	imshow("gray", depth_gray);

	cvWaitKey(0);
	system("PAUSE");
	return 0;
}