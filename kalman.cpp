#include "calculate.h"

using namespace std;
using namespace cv;

//Mat variance = Mat::zeros(424, 512, CV_64FC1);
int h = 424; int w = 512; int min_row = 80; int max_row = 300; int min_col = 180; int max_col = 420;
vector<double> depth_sum;
vector<double> depth_kal;
vector<Mat> depth_rects;
Mat p(max_row - min_row, max_col - min_col, CV_64FC1);
Mat k(max_row - min_row, max_col - min_col, CV_64FC1);

double get_kalman_variance(string path, int h, int w, int min_row, int max_row, int min_col, int max_col){
	for (int i = 1; i <= 100; i++){
		fstream f;
		f.open(path + to_string(i) + ".txt");
		Mat depth(h, w, CV_16SC1);
		for (int i = 0; i < h; i++){
			short *depth_ptr = depth.ptr<short>(i);
			for (int j = 0; j < w; j++){
				f >> depth_ptr[j];
			}
		}
		Mat depth_temp;
		Rect r(min_col, min_row, max_col-min_col, max_row-min_row);
		depth_temp = depth(r);
		depth_rects.push_back(depth_temp);
	}
	double mean = 0;
	for (int i = 0; i < 100; i++){
		mean += depth_sum[i]/100;
	}
	double variance = 0;
	for (int i = 0; i < 100; i++){
		variance += (depth_sum[i] - mean) * (depth_sum[i] - mean) / 100;
	}
	p = variance;
	cout << "·½²î£º" << variance << endl;
	return variance;

}


Mat kalman(string path){

	for (int i = 0; i < max_row - min_row; i++){
		double *ptr1 = p.ptr<double>(i);
		double *ptr2 = k.ptr<double>(i);
		for (int j = 0; j < max_col - min_col; j++){
			ptr1[j] = 2;
			ptr2[j] = 0;
		}
	}
	for (int i = 1; i <= 100; i++){
		fstream f;
		f.open(path + to_string(i) + ".txt");
		Mat depth(h, w, CV_16SC1);
		for (int i = 0; i < h; i++){
			short *depth_ptr = depth.ptr<short>(i);
			for (int j = 0; j < w; j++){
				f >> depth_ptr[j];
			}
		}
		Mat depth_temp;
		Rect r(min_col, min_row, max_col - min_col, max_row - min_row);
		depth_temp = depth(r);
		depth_rects.push_back(depth_temp);
	}
	double variance = 2;
	//vector<double> depth_t(depth_sum);
	for (int i = 1; i < 100; i++){
		for (int m = 0; m < max_row - min_row; m++){
			short *ptr = depth_rects[i].ptr<short>(m);
			short *ptr3 = depth_rects[i-1].ptr<short>(m);
			double *ptr1 = p.ptr<double>(m);
			double *ptr2 = k.ptr<double>(m);
			for (int n = 0; n < max_col - min_col; n++){
				double x = ptr3[n];
				ptr1[n] = ptr1[n] + 0.01;
				ptr2[n] = ptr1[n] / (ptr1[n] + variance);
				x = x + ptr2[n] * (ptr[n] - x);
				ptr1[n] = ptr1[n] - ptr2[n] * ptr1[n];
				ptr[n] = x;
			}
		}
	}
	return depth_rects[99];

}