#ifndef VISION_H
#define VISION_H
#include <opencv2/opencv.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "highgui.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <iostream>
#include <string>
#include <sstream>
#include <cv.h>

using namespace std;
using namespace cv;

typedef struct targets_data{
	double vgate_angle, vgate_radius, vgate_area;
	double buoy_angle, buoy_radius, buoy_area;
	double wire_angle, wire_radius, wire_area;
} targets_data;


class Vision{
	public:
		Vision();
		targets_data findTargets();
		Mat src;

        void getImage();

	private:
        VideoCapture cap_;
        int index_;

		bool done_vgate;
		bool done_buoy;
		bool done_wire;

		//all helpers

		//buoy helpers
		vector<double> detect_buoy(Mat src);
		void outputBuoyAngle(Mat &src, vector<Point2f> &candidates, vector<double> &rvec);

		//gate helpers


		//wire helpers
		Mat color(Mat src);
		vector<double> outputWireAngle(Mat &src_colored);

};

#endif
