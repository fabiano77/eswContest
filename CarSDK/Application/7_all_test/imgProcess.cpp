
#include <iostream>
#include <stdio.h>
#include <string.h>
//#include <sys/time.h>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/gpu/device/utility.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/objdetect/objdetect.hpp>

#define PI 3.1415926

using namespace std;
using namespace cv;

extern "C" {


	/**********************
	* 영상처리 알고리즘 작성
	***********************/

	void OpenCV_calibration(float* map1, float* map2, int w, int h) {
		Size videoSize = Size(w, h);
		Mat Mat_map1(h, w, CV_32FC1, map1);
		Mat Mat_map2(h, w, CV_32FC1, map2);



		Mat disCoeffs;
		Mat cameraMatrix = Mat(3, 3, CV_32FC1);
		int numBoards = 4;

		int numCornerHor = 7;
		int numCornerVer = 7;
		int numSquare = numCornerHor * numCornerVer;

		Size board_sz = Size(numCornerHor, numCornerVer);

		vector <vector <Point3f> > object_point;
		vector <vector <Point2f> > image_point;

		vector <Point2f> corners;
		int successes = 0;

		Mat image;
		Mat gray_image;

		vector <Point3f> obj;
		for (int i = 0; i < numSquare; i++) {
			obj.push_back(Point3f(i / numCornerHor, i % numCornerHor, 0.0f));
		}

		ostringstream osstream;

		while (successes < numBoards) {
			.

				osstream.str("");
			osstream << "./Calib_img/frame_" << 1280 << "_" << successes << ".png";

			image = imread(osstream.str(), IMREAD_COLOR);

			cout << osstream.str() << ", size = " << image.size() << endl;
			resize(image, image, Size(w, h), 0, 0, CV_INTER_LINEAR);

			if (image.empty()) {
				cout << "IMAGE IS EMPTY!" << endl;
				return;
			}

			cvtColor(image, gray_image, COLOR_BGR2GRAY);

			bool found = findChessboardCorners(image, board_sz, corners, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_FILTER_QUADS);

			if (found) {
				drawChessboardCorners(image, board_sz, corners, found);
			}

			image_point.push_back(corners);
			object_point.push_back(obj);

			cout << successes + 1 << "th snap stored!" << endl; 


			successes++;

			osstream.clear();

			if (successes >= numBoards)
				break;


		}

		vector <Mat> rvecs;
		vector <Mat> tvecs;

		cameraMatrix.ptr<float>(0)[0] = 1;
		cameraMatrix.ptr<float>(0)[0] = 1;

		calibrateCamera(object_point, image_point, image.size(), cameraMatrix, disCoeffs, rvecs, tvecs);
		initUndistortRectifyMap(cameraMatrix, disCoeffs, Mat(), cameraMatrix, videoSize, CV_32FC1, Mat_map1, Mat_map2);
	}

	void OpenCV_remap(unsigned char* inBuf, int w, int h, unsigned char* outBuf, float* map1, float* map2)
	{
		Mat srcRGB(h, w, CV_8UC3, inBuf);
		Mat dstRGB(h, w, CV_8UC3, outBuf);

		Mat map1_Mat(h, w, CV_32FC1, map1);
		Mat map2_Mat(h, w, CV_32FC1, map2);

		remap(srcRGB, dstRGB, map1_Mat, map2_Mat, INTER_LINEAR);
	}

	void OpenCV_topview_transform(unsigned char* inBuf, int w, int h, unsigned char* outBuf, int mode)
	{
		Mat srcRGB(h, w, CV_8UC3, inBuf);
		Mat dstRGB(h, w, CV_8UC3, outBuf);
		Mat Hmatrix;

		if (mode == 1)
		{
			Point2f Hp[4] = {	//변환전 좌표
				Point2f(160 * (w / 640.0), 180 * (h / 360.0)),
				Point2f(480 * (w / 640.0), 180 * (h / 360.0)),
				Point2f(620 * (w / 640.0), 270 * (h / 360.0)),
				Point2f(20 * (w / 640.0), 270 * (h / 360.0)) };

			Point2f p[4] = {	//변환후 좌표
				Point2f(100 * (w / 640.0), -100 * (h / 360.0)),
				Point2f(540 * (w / 640.0), -100 * (h / 360.0)),
				Point2f(550 * (w / 640.0), 270 * (h / 360.0)),
				Point2f(90 * (w / 640.0), 270 * (h / 360.0)) };
			Hmatrix = getPerspectiveTransform(Hp, p);
		}
		else if (mode == 2)
		{
			Point2f Hp[4] = {	//변환전 좌표
				Point2f(80 * (w / 640.0), 200 * (h / 360.0)),
				Point2f(560 * (w / 640.0), 200 * (h / 360.0)),
				Point2f(640 * (w / 640.0), 360 * (h / 360.0)),
				Point2f(0 * (w / 640.0), 360 * (h / 360.0)) };

			Point2f p[4] = {	//변환후 좌표
				Point2f(0 * (w / 640.0), 50 * (h / 360.0)),
				Point2f(640 * (w / 640.0), 50 * (h / 360.0)),
				Point2f(640 * (w / 640.0), 360 * (h / 360.0)),
				Point2f(0 * (w / 640.0), 360 * (h / 360.0)) };
			Hmatrix = getPerspectiveTransform(Hp, p);
		}

		Size topviewSize(w, h);	//변환후 사이즈
		warpPerspective(srcRGB, dstRGB, Hmatrix, topviewSize);

	}



}