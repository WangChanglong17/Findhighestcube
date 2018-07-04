
#ifndef CAL_
#define CAL_
#include<vector> 
#include <opencv2/opencv.hpp>
#include <stdio.h>


using namespace cv;
using namespace std;

enum template_type{ square_ = 1, rectangle_, triangle_, circle_ };
double template_standard(template_type x, vector <double> &template_dis);
double calculate_angle(Point P1, Point P2);
double * Contours_matching(vector<vector<Point>> contours, int min_depth_seq_int, template_type x);
#endif