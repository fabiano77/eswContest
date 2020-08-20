﻿#include "imgProcess.h"
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include <iostream>
#include <stdio.h>
#include <string>
#include <sstream>
#include <opencv2/opencv.hpp>

#define PI 3.1415926

using namespace std;
using namespace cv;

void settingStatic(int w, int h);

int calculSteer(Mat& src, int w, int h, bool whiteMode);
//PreCondition	: frame은 자동차의 topview2영상을 입력으로 받는다.
//PostCondition	: 차선을 인식하여 인식된 결과를 src에 표시한다..
//Return : 조향해야될 결과값을 반환한다(-500 ~ 500)

void lineFiltering(Mat& src, Mat& dst, int mode);

void cannyEdge(Mat& src, Mat& dst);

Vec8i hough_ransacLine(Mat& src, Mat& dst, int w, int h, int T, bool printMode, int& detectedLineType);

Vec4i ransac_algorithm(vector<Vec4i> lines, vector<Point2i> P, int w, int h, int T, double& inlierPercent, Rect weightingRect);

int slopeSign(Vec4i line);

double slope(Vec4i line);

double distance_between_line_and_point(Vec4i& line, Point2i point, int w, int h);

int getPointX_at_Y(Vec4i line, const int Y);

int getPointY_at_X(Vec4i line, const int X);

int lineDeviation(Mat& dst, Vec4i line1, Vec4i line2);

string toString(int A);

string toString(double A);

Point centerPoint(Vec4i line);


/// <summary>
/// 관심영역만 마스킹해서 나머지는 올 블랙으로 없애버림
/// </summary>
/// <param name="src"></param> //Input img
/// <param name="dst"></param> //after masking image
/// <param name="points"></param> //4points is needed
void regionOfInterest(Mat& src, Mat& dst, Point* points);
/// <summary>
/// Count gray rate in roi
/// </summary>
/// <param name="src"></param> input img
/// <param name="down"></param> down point
/// <param name="up"></param> up point
/// <param name="dydx"></param> slope degree of line
/// <returns></returns> gray rate of img
float countGray(Mat& src, Point down, Point up, const float dydx);

// TUNNEL
int isDark(Mat& frame, const double percent);
int Tunnel_isStart(Mat& frame, const double percent);


extern "C" {

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
				Point2f(30 * (w / 640.0), 100 * (h / 360.0)),
				Point2f(610 * (w / 640.0), 100 * (h / 360.0)),
				Point2f(640 * (w / 640.0), 360 * (h / 360.0)),
				Point2f(0 * (w / 640.0), 360 * (h / 360.0)) };
			Hmatrix = getPerspectiveTransform(Hp, p);
		}
		else if (mode == 3)
		{
			dstRGB = srcRGB.rowRange(150, 360).clone();
			resize(dstRGB, dstRGB, Size(640, 360));

			return;
		}
		Size topviewSize(w, h);	//변환후 사이즈
		warpPerspective(srcRGB, dstRGB, Hmatrix, topviewSize);

	}

	int autoSteering(unsigned char* inBuf, int w, int h, unsigned char* outBuf)
	{
		Mat srcRGB(h, w, CV_8UC3, inBuf);
		Mat dstRGB(h, w, CV_8UC3, outBuf);

		int steer = calculSteer(srcRGB, w, h, 0);
		dstRGB = srcRGB;

		return steer;
	}

	bool checkObstacle(unsigned char* inBuf, int w, int h, unsigned char* outBuf) {
		/*Capture from inBuf*/
		Mat srcRGB(h, w, CV_8UC3, inBuf);
		Mat dstRGB(h, w, CV_8UC3, outBuf);
		/*Declare usable variables*/
		int height_up = 80;
		int height_down = 150;
		int width = w;
		int height = h;
		/*filtering value setting*/
		Scalar upper_gray(255, 100, 160);
		Scalar lower_gray(0, 0, 0);
		/***************************************/
		/*Convert Color*/
		int color_convert = 1;
		Mat img_white;
		Point roi_points[4] = { Point(0,100),Point(0,360),Point(640,360),Point(640,100) };
		Mat img_roi;
		regionOfInterest(srcRGB, img_roi, roi_points);
		lineFiltering(img_roi, img_white, color_convert);
		/*Canny Image*/
		Mat img_canny;
		cannyEdge(img_white, img_canny);
		imshow("img_white", img_white);
		/*Hough Ransac*/
		Mat img_ransac;
		int line_type;

		Vec8i ransac_points = hough_ransacLine(img_canny, srcRGB, 640, 360, 17, 1, line_type);

		/*Line Splitting*/ //point로 바꿔야됨
		Vec4i line_left(ransac_points[0], ransac_points[1], ransac_points[2], ransac_points[3]);
		Vec4i line_right(ransac_points[4], ransac_points[5], ransac_points[6], ransac_points[7]);

		/*Check Line Splitting*/
		line(srcRGB, Point(line_left[0], line_left[1]), Point(line_left[2], line_left[3]), Scalar(0, 0, 255), 2);
		line(srcRGB, Point(line_right[0], line_right[1]), Point(line_right[2], line_right[3]), Scalar(0, 0, 255), 2);
		/*Check Line Upper Bound*/
		line(srcRGB, Point(0, height_up), Point(width, height_up), Scalar(255, 0, 0), 2);
		/*Calculate Gradient*/
		double grad_right = slope(line_right);
		double grad_left = slope(line_left);

		/*Point that meets upper and lower bound*/
		int rightup_x = getPointX_at_Y(line_right, height_up);
		int rightdown_x = getPointX_at_Y(line_right, height_down);
		int leftup_x = getPointX_at_Y(line_left, height_up);
		int leftdown_x = getPointX_at_Y(line_left, height_down);
		/*Calculate up or down point*/
		Point point_rightup, point_rightdown, point_leftup, point_leftdown;
		point_rightup = Point(rightup_x, height_up);
		point_rightdown = Point(rightdown_x, height_down);
		point_leftup = Point(leftup_x, height_up);
		point_leftdown = Point(leftdown_x, height_down);


		/*Count Gray*/
		float grayrate_left = 0;
		float grayrate_right = 0;
		Mat img_filtered, img_hsv;
		cvtColor(img_roi, img_hsv, COLOR_BGR2HSV);
		inRange(img_hsv, lower_gray, upper_gray, img_filtered);
		line(img_filtered, Point(line_left[0], line_left[1]), Point(line_left[2], line_left[3]), Scalar(0, 0, 255), 2);
		line(img_filtered, Point(line_right[0], line_right[1]), Point(line_right[2], line_right[3]), Scalar(0, 0, 255), 2);
		grayrate_right = countGray(img_filtered, point_rightdown, point_rightup, grad_right);
		grayrate_left = countGray(img_filtered, point_leftdown, point_leftup, grad_left);
		/*Calculate Rate*/
		/**************************************/

		/*Show image*/
		Point location(width / 2, height * 6 / 8);// Location that is Detected Sign 
		Point location2(width / 2, height / 8);// Location that is time of calculation
		Point location_left(width / 4, height * 7 / 8); // Location that is rate of gray detected(left)
		Point location_right(width * 3 / 4, height * 7 / 8); //Location that is rate of gray detected(right)

		int font = FONT_ITALIC; // italic font
		double fontScale = 1;

		string s_left = toString((double)grayrate_left) + "%";
		string s_right = toString((double)grayrate_right) + "%";
		putText(srcRGB, s_left, location_left, font, fontScale, Scalar(255, 0, 0), 2);
		putText(srcRGB, s_right, location_right, font, fontScale, Scalar(255, 0, 0), 2);
		/*Choose Left or Right*/
		//선이 없는 경우 배제하는 것도 필요
		if (grayrate_left > grayrate_right)
		{
			putText(srcRGB, "Left to go", location, font, fontScale, Scalar(0, 0, 255), 2);
			return true;
		}
		else
		{
			putText(srcRGB, "Right to go", location, font, fontScale, Scalar(0, 0, 255), 2);
			return false;
		}

	}

	int Tunnel(unsigned char* inBuf, int w, int h, const double percent)
	{
		Mat srcRGB(h, w, CV_8UC3, inBuf);

		// return Tunnel_isStart(srcRGB, percent);

		if (Tunnel_isStart(srcRGB, percent)) {
			printf("IN TUNNEL\n");
			return 1;
		}
		return 0;
	}
}	// extern "C"


Scalar color[7];
Scalar pink(255, 50, 255);
Scalar mint(255, 153, 0);
Scalar red(0, 0, 255);
Scalar green(0, 255, 0);
Scalar blue(255, 0, 0);
Scalar purple(255, 102, 165);
Scalar orange(0, 169, 237);
Vec4i leftGuide;
Vec4i rightGuide;
Vec4i centerGuide[5];
//Mat temp;
int guideCnt = 5;
bool first = 0;
int HLP_threshold = 30;	//105
int HLP_minLineLength = 90;//115
int HLP_maxLineGap = 500;	//260

static void on_trackbar(int, void*)
{
}

void settingStatic(int w, int h)
{
	color[0] = Scalar(255, 255, 0);
	color[1] = blue;
	color[2] = green;
	color[3] = orange;
	color[4] = red;
	color[5] = Scalar(255, 0, 255);
	color[6] = Scalar(192, 192, 192);

	//centerGuide[0] = Vec4i(320, 100, 320, 150);
	//centerGuide[1] = Vec4i(320, 150, 320, 200);
	//centerGuide[2] = Vec4i(320, 200, 320, 260);
	//centerGuide[3] = Vec4i(320, 260, 320, 320);
	//centerGuide[4] = Vec4i(320, 320, 320, 360);

	centerGuide[0] = Vec4i(320, 160, 320, 200);
	centerGuide[1] = Vec4i(320, 200, 320, 240);
	centerGuide[2] = Vec4i(320, 240, 320, 280);
	centerGuide[3] = Vec4i(320, 280, 320, 320);
	centerGuide[4] = Vec4i(320, 320, 320, 360);

	for (int i = 0; i < guideCnt; i++)
	{
		centerGuide[i][0] *= (w / 640.0);
		centerGuide[i][1] *= (h / 360.0);
		centerGuide[i][2] *= (w / 640.0);
		centerGuide[i][3] *= (h / 360.0);
	}

	leftGuide = Vec4i(0, 180, 150, 0) * (w / 640.0);
	rightGuide = Vec4i(490, 0, 640, 180) * (w / 640.0);

	cout << "settingStatic" << endl;
}

int calculSteer(Mat& src, int w, int h, bool whiteMode)
{
	if (!first++) settingStatic(w, h);
	int retval(0);
	Mat src_yel;
	Mat src_can;
	Point printPosition(230, 50);

	lineFiltering(src, src_yel, whiteMode);
	cannyEdge(src_yel, src_can);
	int lineType;	// 0 == 라인이 없다, 1 == 라인이 한개, 2 == 라인이 두개.
	Vec8i l = hough_ransacLine(src_can, src, w, h, 15, 1, lineType);
	Vec4i firstLine(l[0], l[1], l[2], l[3]);
	Vec4i secondLine(l[4], l[5], l[6], l[7]);

	if (lineType == 0)
	{
		putText(src, "no lines", printPosition, FONT_HERSHEY_COMPLEX, 0.8, Scalar(255, 255, 255), 2);
		return 9999;	//의미없다는 결과를 알려주는 수(조향하지말라는 뜻)
	}
	else if (lineType == 1 && (getPointY_at_X(firstLine, w / 2) > 0 || abs(slope(firstLine)) < 0.95))
	{
		//직선이 1개이고, 
		//기울기절댓값이 0.95미만이거나 직선의 x = 180에서의 좌표가 화면내에 존재할때
		/************************************************************************************
		*										곡선 구간
		************************************************************************************/
		int proximity(0);	//외곽 차선이 차체와 얼마나 근접하였는지 값 (0~500)
		int bias = slopeSign(firstLine) * 90 * (640.0 / w);	//좌, 우회전에 따른 가이드라인 바이어스.
		int linePointY_atCenter = getPointY_at_X(firstLine, (w / 2) + bias);
		for (int i = 0; i < guideCnt; i++)
		{
			//가이드 직선 표시 & 직선의 근접도 판단.
			if (linePointY_atCenter > centerGuide[i][1])
			{
				rectangle(src, Point(centerGuide[i][0] - 5 + bias, centerGuide[i][1]), Point(centerGuide[i][2] + 5 + bias, centerGuide[i][3]), Scalar(70, 70, 70), -1);
				//5단계로 나누어 조향.
				proximity += 500.0 / guideCnt;
			}
			else
				rectangle(src, Point(centerGuide[i][0] - 5 + bias, centerGuide[i][1]), Point(centerGuide[i][2] + 5 + bias, centerGuide[i][3]), color[i], -1);
		}
		//if (linePointY_atCenter > centerGuide[0][1])	// 선형적으로 조향.
		//	proximity = 500 * (linePointY_atCenter - centerGuide[0][1]) / (h - centerGuide[0][1]);

		if (proximity == 0)
			retval = 9999; //(조향하지말라는 뜻)
		else if (slopeSign(firstLine) < 0)
			retval = proximity;
		else
			retval = -proximity;
	}
	else //if (lineType == 2 || (lineType == 1 && abs(slope(firstLine)) > 0.95) || (lineType == 1 && (getPointY_at_X(fisrtLine, w / 2) < 0)) )
	{
		// 직선이 두개이거나
		// 직선이 하나인데 기울기의 절댓값이 0.95미만이거나
		// 직선이 하나인데 직선의 x = (화면중심)에서의 y좌표가 화면 상단을 뚫고나가있을경우.
		/************************************************************************************
		*										직진 구간
		************************************************************************************/
		//가이드 표시.
		line(src, Point(leftGuide[0], leftGuide[1]), Point(leftGuide[2], leftGuide[3]), mint, 4);
		line(src, Point(rightGuide[0], rightGuide[1]), Point(rightGuide[2], rightGuide[3]), mint, 4);

		int Deviation = 0;	//기준 직선에 얼마나 치우쳐있는지 값.
		if (lineType == 2)
		{
			Deviation += lineDeviation(src, firstLine, leftGuide);
			Deviation += lineDeviation(src, secondLine, rightGuide);

			Deviation *= 1;
			//직선 2개 보이는 직진구간 보정.
		}
		else //lineType == 1
		{
			if (slope(firstLine) < 0)
				Deviation += lineDeviation(src, firstLine, leftGuide);
			else
				Deviation += lineDeviation(src, firstLine, rightGuide);

			Deviation *= 1.5;
			//직선 1개만 보이는 직진구간 보정.
		}
		retval = Deviation;
	}

	line(src, Point(firstLine[0], firstLine[1]), Point(firstLine[2], firstLine[3]), pink, 5);
	if (lineType == 2)
		line(src, Point(secondLine[0], secondLine[1]), Point(secondLine[2], secondLine[3]), pink, 5);

	putText(src, "Steering = " + ((retval == 9999) ? "0" : toString(retval)), printPosition, 0, 0.8, Scalar(255, 255, 255), 2);
	putText(src, (abs(retval) < 20) ? "[ ^ ]" : (retval < 0) ? "[<<<]" : "[>>>]", printPosition + Point(55, 30), 0, 0.8, Scalar(255, 255, 255), 2);

	if (retval != 9999)
	{
		//유의미한 값이 도출됐을 경우.
		if (retval < -500) retval = -500;
		else if (retval > 500) retval = 500;
	}

	return retval;
}

void lineFiltering(Mat& src, Mat& dst, int mode)
{
	dst = Mat();
	int h1(14), s(60), v(100); // 예선영상 14, 0, 240
	int h2(46);
	Scalar lower_yellow(h1, s, v);
	Scalar upper_yellow(h2, 255, 255);
	Mat hsv;
	Mat binMat;

	cvtColor(src, hsv, COLOR_BGR2HSV);					//HSV색영역
	inRange(hsv, lower_yellow, upper_yellow, binMat);	//2진 Mat객체 binMat생성

	if (mode == 1)//흰색차선 인식 모드
	{
		Scalar lower_white(220, 220, 220); // bgr white
		Scalar upper_white(255, 255, 255);
		Mat whiteBinMat;

		inRange(src, lower_white, upper_white, whiteBinMat);
		//addWeighted(binMat, 1.0, whiteBinMat, 1.0, 0.0, binMat);	//추출한 노란색과 흰색 객체를 합친 binMat생성
		binMat = binMat + whiteBinMat;
	}

	dst = binMat;
	//bitwise_and(src, src, dst, binMat);	//binMat객체로 원본 frame 필터링.(노란색남음)
}

void cannyEdge(Mat& src, Mat& dst)
{
	int threshold_1 = 118;		//215 //340
	int threshold_2 = 242;		//330 //500

	Canny(src, dst, threshold_1, threshold_2);	//노란색만 남은 frame의 윤곽을 1채널 Mat객체로 추출
}

Vec8i hough_ransacLine(Mat& src, Mat& dst, int w, int h, int T, bool printMode, int& detectedLineType)
{
	Point printPoint(210 * (w / 640.0), 140 * (h / 360.0));
	vector<Point2i> P;
	Vec4i firstLine;
	Vec4i secondLine;

	for (int x = 0; x < w; x++)
	{
		for (int y = 0; y < h; y++)
		{
			if (src.at<uchar>(y, x))
			{
				//모든 점 찾기
				P.push_back(Point2i(x, y));
			}
		}
	}

	createTrackbar("H_thresh", "trackbar", &HLP_threshold, 120, on_trackbar);
	createTrackbar("H_minLen", "trackbar", &HLP_minLineLength, 200, on_trackbar);
	createTrackbar("H_maxGap", "trackbar", &HLP_maxLineGap, 500, on_trackbar);
	namedWindow("trackbar", WINDOW_NORMAL);
	moveWindow("trackbar", 320 * 5, 180 * 5);

	vector<Vec4i> lines;		//검출될 직선이 저장될 객체
	HoughLinesP(src, lines, 1, CV_PI / 180, HLP_threshold, HLP_minLineLength, HLP_maxLineGap);

	int lowLinePosition(0);
	for (unsigned int i = 0; i < lines.size(); i++)
	{
		//수직 예외직선 삭제 및 ROI설정을 위한 직선 위치판단
		if (abs(slope(lines[i])) > 30 || abs(slope(lines[i])) < 0.1)
		{
			line(dst, Point(lines[i][0], lines[i][1]), Point(lines[i][2], lines[i][3]), Scalar(), 2);
			lines.erase(lines.begin() + i);
			i--;
			continue;
		}
		if (lines[i][1] > h * (1 / 2.0) && lines[i][3] > h * (1 / 2.0))
		{
			//화면 하단 1/2에 라인이 있을 경우.
			if (lowLinePosition < 1)lowLinePosition = 1;
		}
		if (lines[i][1] > h * (2 / 3.0) && lines[i][3] > h * (2 / 3.0))
		{
			//화면 하단 1/3에 라인이 있을 경우.
			if (lowLinePosition < 2)lowLinePosition = 2;
		}
	}

	for (unsigned int i = 0; lowLinePosition && i < lines.size(); i++)
	{
		if (lowLinePosition == 1)
		{
			if (i == 0)
			{
				putText(dst, "Region Of Interest", Point(15, h * (1 / 3.0) - 10), 0, 0.65, Scalar(0, 0, 255), 2);
				line(dst, Point(0, h * (1 / 3.0)), Point(w, h * (1 / 3.0)), Scalar(0, 0, 255), 2);
			}
			if (centerPoint(lines[i]).y < h * (1 / 3.0))
			{
				//화면 하단 1/2에 라인이 있을 경우.
				//상단1/3구간 예외직선 무시 구문
				line(dst, Point(lines[i][0], lines[i][1]), Point(lines[i][2], lines[i][3]), Scalar(), 2);
				lines.erase(lines.begin() + i);
				i--;
			}
		}
		else // lowLinePosition == 2
		{
			if (i == 0)
			{
				putText(dst, "Region Of Interest", Point(15, h * (1 / 2.0) - 10), 0, 0.65, Scalar(0, 0, 255), 2);
				line(dst, Point(0, h * (1 / 2.0)), Point(w, h * (1 / 2.0)), Scalar(0, 0, 255), 2);
			}
			if (centerPoint(lines[i]).y < h * (1 / 2.0))
			{
				//화면 하단 1/3에 라인이 있을 경우.
				//상단 1/4구간 예외직선 무시구문
				line(dst, Point(lines[i][0], lines[i][1]), Point(lines[i][2], lines[i][3]), Scalar(), 2);
				lines.erase(lines.begin() + i);
				i--;
			}
		}
	}

	for (unsigned int j = 0; printMode && j < lines.size(); j++)
	{
		line(dst, Point(lines[j][0], lines[j][1]), Point(lines[j][2], lines[j][3]), Scalar(255, 255, 255), 2);
		//circle(dst, Point2i(lines[j][0], lines[j][1]), 4, color[j], -1, LINE_AA);
		//circle(dst, Point2i(lines[j][2], lines[j][3]), 4, color[j], -1, LINE_AA);
	}

	if (lines.size() < 1)
	{
		detectedLineType = 0;
		return Vec8i();
	}

	Vec4i rightLine(0, -1, 0, 0);		//최우측 직선
	Vec4i leftLine(w, -1, w, 0);		//최좌측 직선
	for (unsigned int i = 0; i < lines.size(); i++)
	{
		if (leftLine[0] > lines[i][0])
		{
			leftLine = lines[i];
		}
		if (rightLine[2] < lines[i][2])
		{
			rightLine = lines[i];
		}
	}
	//for문이 끝나고 나면 각종 최좌측, 최우측 직선 각각 저장.
	//temp = dst;
	if (slopeSign(leftLine) < 0 && slopeSign(rightLine) > 0)	//좌우 직선의 기울기부호가 다르다 == 직진구간.
	{
		detectedLineType = 2;
		double left_inlierPercent;
		double right_inlierPercent;
		vector<Vec4i> leftLines;
		vector<Vec4i> rightLines;
		for (unsigned int i = 0; i < lines.size(); i++)
		{
			if (slope(lines[i]) < 0)
				leftLines.push_back(lines[i]);
			else
				rightLines.push_back(lines[i]);
		}
		firstLine = ransac_algorithm(leftLines, P, w, h, T, left_inlierPercent, Rect());
		secondLine = ransac_algorithm(rightLines, P, w, h, T, right_inlierPercent, Rect());
		if (printMode)
		{
			putText(dst, "inlier(%)  = " + toString((int)left_inlierPercent) + "%", printPoint + Point(-160, 0), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255, 255, 255), 2);
			putText(dst, "inlier(%)  = " + toString((int)right_inlierPercent) + "%", printPoint + Point(+160, 0), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255, 255, 255), 2);
			putText(dst, "slope = " + toString(slope(firstLine)), printPoint + Point(-160, 30), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255, 255, 255), 2);
			putText(dst, "slope = " + toString(slope(secondLine)), printPoint + Point(+160, 30), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255, 255, 255), 2);
		}
		//dst = temp;
		return Vec8i(firstLine[0], firstLine[1], firstLine[2], firstLine[3],
			secondLine[0], secondLine[1], secondLine[2], secondLine[3]);
	}
	else //if (slopeSign(leftLine) == slopeSign(rightLine))	//좌우 직선의 기울기부호가 같으면
	{
		Point highest(-1, h);		//최상단 점
		Point lowest(-1, 0);		//최하단 점
		for (unsigned int i = 0; i < lines.size(); i++)
		{
			if (highest.y > lines[i][1])
			{
				highest = Point(lines[i][0], lines[i][1]);
			}
			if (highest.y > lines[i][3])
			{
				highest = Point(lines[i][2], lines[i][3]);
			}
			if (lowest.y < lines[i][1])
			{
				lowest = Point(lines[i][0], lines[i][1]);
			}
			if (lowest.y < lines[i][3])
			{
				lowest = Point(lines[i][2], lines[i][3]);
			}
		}
		Rect lineRatio(highest, lowest);
		if (((lineRatio.x < w / 2.0) && (lineRatio.x + lineRatio.width < w / 2.0)) ||
			((lineRatio.x > w / 2.0) && (lineRatio.x + lineRatio.width > w / 2.0)))
		{
			detectedLineType = 0;
			rectangle(dst, lineRatio, purple, 4);
			putText(dst, "rect ratio = " + toString((double)lineRatio.height / lineRatio.width), printPoint + Point(0, 60), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255, 255, 255), 2);
			return Vec8i();
		}

		detectedLineType = 1;
		//가중치 영역 설정.
		Rect weightingRect(w / 3, h / 2, w / 3, h / 2);
		//rectangle(dst, weightingRect, purple, 2);

		double inlierPercent;
		firstLine = ransac_algorithm(lines, P, w, h, T, inlierPercent, weightingRect);

		if (printMode)
		{
			putText(dst, "inlier(%)  = " + toString((int)inlierPercent) + "%", printPoint + Point(0, 0), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255, 255, 255), 2);
			putText(dst, "slope = " + toString(slope(firstLine)), printPoint + Point(0, 30), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255, 255, 255), 2);
			putText(dst, "rect ratio = " + toString((double)lineRatio.height / lineRatio.width), printPoint + Point(0, 60), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255, 255, 255), 2);
		}
		//dst = temp;
		return Vec8i(firstLine[0], firstLine[1], firstLine[2], firstLine[3],
			-1, -1, -1, -1);
	}
	return 0;
}

Vec4i ransac_algorithm(vector<Vec4i> lines, vector<Point2i> P, int w, int h, int T, double& inlierPercent, Rect weightingRect)
{
	Vec4i resultLine;
	int cntMax(-1);
	int checkCnt(0);
	for (unsigned int i = 0; i < lines.size(); i++)
	{
		int cntInlier = 0;
		//int cnt = 0;

		Vec4i checkLine = lines[i];
		for (unsigned int j = 0; j < P.size(); j++)
		{
			Point2i calculPoint = P[j];
			//cnt++;
			if (distance_between_line_and_point(checkLine, calculPoint, w, h) < T)
			{
				if (P[j].x > weightingRect.x
					&& P[j].x < weightingRect.x + weightingRect.width
					&& P[j].y > weightingRect.y
					&& P[j].y < weightingRect.y + weightingRect.height)
				{
					//가중치영역인 weightingRect에서 지지를 받는 경우.
					if (P[j].y > weightingRect.y + (weightingRect.height / 2))
						cntInlier += 7;
					else
						cntInlier += 3;
				}
				else if (P[j].y > weightingRect.y && P[j].y < weightingRect.y + weightingRect.height)
					cntInlier += 2;
				else
					cntInlier++;
			}
		}

		if (cntInlier > cntMax)
		{
			resultLine = checkLine;
			cntMax = cntInlier;
			//checkCnt = cnt;
		}
		//그외직선 지지도 디버깅.
		//inlierPercent = ((double)cntInlier / P.size()) * 100.0;
		//circle(temp, centerPoint(lines[i]), 3, blue, -1);
		//putText(temp, toString((int)inlierPercent) + "%", centerPoint(lines[i]) + Point(10, 0), FONT_HERSHEY_SIMPLEX, 0.65, Scalar(), 2);
	}
	//inlierPercent = ((double)cntMax / checkCnt) * 100.0;
	inlierPercent = ((double)cntMax / P.size()) * 100.0;

	return resultLine;
}

int slopeSign(Vec4i line)
{
	if (line[1] == -1)	//직선이 없다는것.
		return 0;
	else if (slope(line) > 0)
		return 1;
	else
		return -1;
}

double slope(Vec4i line)
{
	return ((double)line[3] - line[1]) / ((double)line[2] - line[0]);
}

double distance_between_line_and_point(Vec4i& line, Point2i point, int w, int h)
{
	double m, b;
	m = slope(line);
	b = line[1] - m * line[0];
	//y = mx + b 형태의 직선의 방정식
	//d = | mx + b - y | / root( m^2 + 1 )

	if (line[0] != 0)
	{
		//직선을 길게 확장해준다. 좌측y절편, 우측y절편
		//double y1, y2;
		//y1 = b;
		//y2 = (m * w + b);
		line = Vec4i(0, cvRound(b), w, cvRound(m * w + b));
	}
	return (abs(m * point.x + b - point.y) / sqrt(m * m + 1));
}

int getPointX_at_Y(Vec4i line, const int Y)
{
	double m, b;
	m = slope(line);
	b = line[1] - m * line[0];

	return (int)((Y - b) / m);
}

int getPointY_at_X(Vec4i line, const int X)
{
	double m, b;
	m = slope(line);
	b = line[1] - m * line[0];

	return (int)(m * X + b);
}

int lineDeviation(Mat& dst, Vec4i line1, Vec4i line2)
{
	//y = 10에서 직선끼리의 거리 비교.
	return getPointX_at_Y(line1, 60) - getPointX_at_Y(line2, 60);
}

string toString(int A)
{
	stringstream ss;
	string text;
	ss << A;
	ss >> text;
	return text;
}

string toString(double A)
{
	stringstream ss;
	string text;
	ss << A;
	ss >> text;
	return text;
}

Point centerPoint(Vec4i line)
{
	return Point((line[0] + line[2]) / 2.0, (line[1] + line[3]) / 2.0);
}

void regionOfInterest(Mat& src, Mat& dst, Point* points)
{ // points의 포인터인 이유-> 여러개의 꼭짓점 경우

	Mat maskImg = Mat::zeros(src.size(), CV_8UC3);

	Scalar ignore_mask_color = Scalar(255, 255, 255);
	const Point* ppt[1] = { points }; //개의 꼭짓점 :n vertices
	int npt[] = { 4 };

	fillPoly(maskImg, ppt, npt, 1, Scalar(255, 255, 255), 8);
	Mat maskedImg;
	bitwise_and(src, maskImg, maskedImg);
	dst = maskedImg;
}

float countGray(Mat& src, Point down, Point up, const float dydx)
{
	CV_Assert(src.type() == CV_8UC1);
	int count_left = 0;
	int count_right = 0;
	int count_total = 0;
	float rate = 0;
	/*check left side or right side*/
	int width = src.cols;
	if (down.x < width / 2) {//left
		for (int y = up.y; y < down.y; y++)//up.y<down.y
		{ //y
			int lower_x;//lower bound for calculate rectangular form
			if (dydx >= 1000) {
				lower_x = 0;
			}
			else { lower_x = (y - up.y) / dydx + up.x; }

			for (int x = 0; x < lower_x; x++) // left Gray detection
			{ //x
				uchar color_value = src.at<uchar>(y, x);
				//img_filtered를 사용해야 회색의 범위를 찾음
				if (color_value > 128)
				{
					count_left++;
				}
				count_total++;
			}
		}
		rate = (float)count_left / count_total * 100.0;
	}
	else {//right
		for (int y = up.y; y < down.y; y++)//up.y<down.y
		{ //y
			int upper_x;//upper bound for calculate rectangular form
			if (dydx >= 1000) { upper_x = width; }
			else { upper_x = (y - up.y) / dydx + up.x; }
			for (int x = upper_x; x < width; x++) // left Gray detection
			{ //x
				uchar color_value = src.at<uchar>(y, x);
				//img_filtered를 사용해야 회색의 범위를 찾음
				if (color_value > 128)
				{
					count_right++;
				}
				count_total++;
			}
		}
		rate = (float)count_right / count_total * 100.0;
	}

	return rate;
}


/*/////////////////////////////
		TUNNEL START
*//////////////////////////////
int flag_tunnel;
int first_tunnel = 0;
int MAXTHR_tunnel = 20;
int MINTHR_tunnel = 10;

int isDark(Mat& frame, const double percent) {

	Mat grayFrame;

	cvtColor(frame, grayFrame, COLOR_RGB2GRAY);

	int pixelCnt(0);
	int pixelValue(0);
	for (int i = 0; i < grayFrame.cols; i += 10)
	{
		for (int j = 0; j < grayFrame.rows; j += 10)
		{
			pixelValue += grayFrame.at<uchar>(j, i);
			pixelCnt++;
		}
	}
	int totalValue = pixelCnt * 255;
	double brightRate = ((double)pixelValue / totalValue) * 100.0;

	if (brightRate < (100 - percent))
	{
		return 1;
	}
	return 0;
}
int Tunnel_isStart(Mat& frame, const double percent) {
	if (!first_tunnel++) flag_tunnel = -1;

	if (isDark(frame, percent)) {
		if (flag_tunnel < MAXTHR_tunnel)
			flag_tunnel++;
	}
	else {
		if (flag_tunnel > 0)
			flag_tunnel--;
	}

	if (flag_tunnel >= MINTHR_tunnel)
		return 1;
	return 0;
}
/*/////////////////////////////
		TUNNEL END
*//////////////////////////////


