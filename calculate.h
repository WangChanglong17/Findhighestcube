
#ifndef CAL_
#define CAL_
#include<vector> 
#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <time.h>
#include <fstream>

using namespace cv;
using namespace std;

enum template_type{ square_ = 1, rectangle_, triangle_, circle_ };
double template_standard(template_type x, vector <double> &template_dis);
double calculate_angle(Point P1, Point P2);
double * Contours_matching(const vector<vector<Point>> &contours, Mat &depth_gray, int min_depth_seq_int, template_type x);
double get_kalman_variance(string path, int h, int w, int min_row, int max_row, int min_col, int max_col);
Mat kalman(string path);
Mat points2mat(vector<Point3d> pp);
Mat get_f_vector(Mat &pp, double *mean);
vector<int> genknuth(int m, int n);

class ransac
{
private:
	Mat positions;
	double error;
	vector<Point3d> points;
	double f_vector[3];
	double threshold;
	int iterations;
	int num;
	double rate;
public:
	ransac(Mat &positions1, double threshold1 = 5, int iterations1 = 100, double rate1 = 0.01, double error1 = 10000);
	void run_ransac();
	~ransac(){}
	Mat get_ransac_points();
};

#endif