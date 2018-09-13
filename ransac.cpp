#include "calculate.h"

//不重复地从n中随机取m个数
vector<int> genknuth(int m, int n)
{
	srand((unsigned)time(NULL));
	vector<int> k;
	for (int i = 0; i<n; i++)
		if (rand() % (n - i) < m) {
			k.push_back(i);
			m--;
		}
	return k;
}
Mat get_f_vector(Mat &pp,double *mean){
	Mat ppp(pp.rows, pp.cols, CV_64FC1);
	for (int i = 0; i < pp.rows; i++){
		for (int j = 0; j < pp.cols; j++){
			ppp.at<double>(i, j) = pp.at<double>(i, j)-mean[i];
		}
	}
	Mat pp_t;
	transpose(ppp, pp_t);
	Mat ptp = ppp * pp_t;
	Mat values, vectors;
	cv::eigen(ptp, values, vectors);
	Mat temp(1, 3, CV_64FC1);
	for (int i = 0; i < 3; i++){
		temp.at<double>(0, i) = vectors.at<double>(2, i);
	}
	return temp;
}

Mat points2mat(vector<Point3d> pp){
	Mat p(3, pp.size(), CV_64FC1);
	for (int i = 0; i < pp.size(); i++){
		p.at<double>(0, i) = (pp[i].x);
		p.at<double>(1, i) = (pp[i].y);
		p.at<double>(2, i) = (pp[i].z);
	}
	return p;
}

ransac::ransac(Mat &positions1, double threshold1, int iterations1 , double rate1 , double error1 )
{
	positions1.copyTo(positions);
	iterations = iterations1;
	threshold = threshold1;
	error = error1;
	rate = rate1;
	num = positions.cols; 
	run_ransac();
}

void ransac::run_ransac(){
	for (int i = 0; i < iterations; i++){
		int mm = 10;
		vector<int> rand_num = genknuth(mm, positions.cols);
		Mat pp(3, mm, CV_64FC1);
		double mean[3] = { 0, 0, 0 };
		for (int k = 0; k < 3; k++){
			for (int m = 0; m < mm; m++){
				pp.at<double>(k, m) = (positions.at<double>(k, rand_num[m]));
				mean[k] += pp.at<double>(k, m) / mm;
			}
		}
		Mat mean_m(3, positions.cols, CV_64FC1);
		for (int g = 0; g < 3; g++){
			for (int h = 0; h < mean_m.cols; h++){
				mean_m.at<double>(g, h) = mean[g];
			}
		}
		//获取法向量
		Mat f_vector = get_f_vector(pp, mean);
		Mat diff;
		//得到误差矩阵
		subtract(positions, mean_m, diff);
		Mat error_mat_temp = f_vector * diff;
		Mat error_mat = error_mat_temp.mul(error_mat_temp);
		vector<Point3d> points_temp;
		for (int i = 0; i < error_mat.cols; i++){
			double e = error_mat.at<double>(0, i);
			if (e < threshold){
				Point3d p_temp = Point3d(positions.at<double>(0, i), positions.at<double>(1, i), positions.at<double>(2, i));
				points_temp.push_back(p_temp);

			}
		}
		if (points_temp.size() > rate * num && points_temp.size() == points.size()){

			Mat positions_temp = points2mat(points_temp);
			double mean1[3] = { 0, 0, 0 };
			for (int k = 0; k < 3; k++){
				for (int m = 0; m < positions_temp.cols; m++){
					mean1[k] += positions_temp.at<double>(k, m) / positions_temp.cols;
				}
			}
			Mat mean_1(3, positions.cols, CV_64FC1);
			for (int g = 0; g < 3; g++){
				for (int h = 0; h < mean_1.cols; h++){
					mean_1.at<double>(g, h) = mean1[g];
				}
			}
			Mat normal_v = get_f_vector(positions_temp, mean1);
			Mat diff0;
			//得到误差矩阵
			subtract(positions, mean_1, diff0);
			Mat a = normal_v * diff0;
			Mat b = a.mul(a);
			double error_temp = 0;
			vector<Point3d> points_temp1;
			double *bptr = b.ptr<double>(0);
			for (int i = 0; i < b.cols; i++){
				error_temp += bptr[i] / b.cols;
				if (bptr[i] < threshold){
					Point3d p_temp = Point3d(positions.at<double>(0, i), positions.at<double>(1, i), positions.at<double>(2, i));
					points_temp1.push_back(p_temp);
				}
			}
			if (error > error_temp){
				error = error_temp;
				points.clear();
				vector<Point3d>::iterator iter;
				for (iter = points_temp1.begin(); iter != points_temp1.end(); iter++){
					points.push_back(*iter);
				}
			}
		}
		else if (points_temp.size() > rate * num && points_temp.size() > points.size()){

			Mat positions_temp = points2mat(points_temp);
			double mean1[3] = { 0, 0, 0 };
			for (int k = 0; k < 3; k++){
				for (int m = 0; m < positions_temp.cols; m++){
					mean1[k] += positions_temp.at<double>(k, m) / positions_temp.cols;
				}
			}
			Mat mean_1(3, positions.cols, CV_64FC1);
			for (int g = 0; g < 3; g++){
				for (int h = 0; h < mean_1.cols; h++){
					mean_1.at<double>(g, h) = mean1[g];
				}
			}
			Mat normal_v = get_f_vector(positions_temp, mean1);
			Mat diff0;
			//得到误差矩阵
			subtract(positions, mean_1, diff0);
			Mat a = normal_v * diff0;
			Mat b = a.mul(a);
			double error_temp = 0;
			vector<Point3d> points_temp1;
			double *bptr = b.ptr<double>(0);
			for (int i = 0; i < b.cols; i++){
				error_temp += bptr[i] / b.cols;
				if (bptr[i] < threshold){
					Point3d p_temp = Point3d(positions.at<double>(0, i), positions.at<double>(1, i), positions.at<double>(2, i));
					points_temp1.push_back(p_temp);
				}
			}
			error = error_temp;
			points.clear();
			vector<Point3d>::iterator iter;
			for (iter = points_temp1.begin(); iter != points_temp1.end(); iter++){
				points.push_back(*iter);
			}
		}
	}
}


Mat ransac::get_ransac_points(){
	Mat p(3, points.size(), CV_64FC1);
	for (int i = 0; i < points.size(); i++){
		p.at<double>(0, i) = (points[i].x);
		p.at<double>(1, i) = (points[i].y);
		p.at<double>(2, i) = (points[i].z);
	}
	return p;
}

