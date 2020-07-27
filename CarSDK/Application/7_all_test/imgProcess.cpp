
#include "imgProcess.h"
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include <iostream>
#include <stdio.h>
#include <string>
#include <sstream>

//#include <sys/time.h>

#include <opencv2/opencv.hpp>


#define PI 3.1415926

using namespace std;
using namespace cv;

int calculSteer(Mat& src);
//PreCondition	: frame�� �ڵ����� topview2������ �Է����� �޴´�.
//PostCondition	: ������ �ν��Ͽ� �νĵ� ����� src�� ǥ���Ѵ�..
//Return : �����ؾߵ� ������� ��ȯ�Ѵ�(-500 ~ 500)

void lineFiltering(Mat& src, Mat& dst, int mode);
//PreCondition	: ����
//PostCondition	: �����(or ���)������ �����ϴ� src���󿡼� ���������� ���� ���͸��Ͽ� dst�� ��ȯ�Ѵ�.
//Return : ����

void houghLineDetection(Mat& src, Mat& dst, bool printMode);//, Vec4i& firstLine, Vec4i& secondLine);
//PreCondition	: ����.
//PostCondition	: houghLineP �Լ� ����� dst�� ��Ÿ���ش�.
//Return : ����

void cannyEdge(Mat& src, Mat& dst);
//PreCondition	: ����
//PostCondition	: canny�Լ� ����� dst�� ����Ѵ�.
//Return : ����

Vec4i ransacLine(Mat& src, Mat& dst, int w, int h, double T, int n, bool printMode);
//PreCondition	: src������ cannyó���� ��ģ ��������Ѵ�.
//PostCondition	: dst�� ����� ������ ��ũ������ ǥ���Ѵ�.
//Return : ����� ������ ��ȯ(Vec4i)

Vec8i split_ransacLine(Mat& src, Mat& dst, int w, int h, double T, int n, bool printMode);
//PreCondition	: src������ cannyó���� ��ģ ��������Ѵ�.
//PostCondition	: src������ �¿�� �����Ͽ� �����Ѵ�. �׿� ����.
//Return : ����� ���� �ΰ��� ��ȯ(Vec8i)
//			return Vec8i(leftLine[0], leftLine[1], leftLine[2], leftLine[3],
//						rightLine[0], rightLine[1], rightLine[2], rightLine[3]);

double distance_between_line_and_point(Vec4i& line, Point2i point, int w, int h);

int slopeSign(Vec4i line);

double slope(Vec4i line);

int lineDeviation(Vec4i line1, Vec4i line2);

int getPointX_at_Y(Vec4i line);

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

	int autoSteering(unsigned char* inBuf, int w, int h, unsigned char* outBuf)
	{
		Mat srcRGB(h, w, CV_8UC3, inBuf);
		Mat dstRGB(h, w, CV_8UC3, outBuf);

		int steer = calculSteer(srcRGB);
		dstRGB = srcRGB;

		return steer;
	}
}


Scalar color[7];
Scalar pink(255, 50, 255);
Scalar mint(255, 153, 0);
Scalar red(0, 0, 255);
Scalar green(0, 255, 0);
Scalar blue(255, 0, 0);

Vec4i leftGuide(0, 118, 320, -330);//leftGuide(0, 138, 320, -278);
Vec4i rightGuide(320, -420, 640, 155);//rightGuide(320, -390, 640, 178);

Vec4i centerGuide[4];

bool first = 0;
int HLP_threshold = 60;	//105
int HLP_minLineLength = 70;//115
int HLP_maxLineGap = 500;	//260

void settingStatic()
{
	color[0] = Scalar(255, 255, 0);
	color[1] = Scalar(255, 0, 0);
	color[2] = Scalar(0, 255, 0);
	color[3] = Scalar(0, 0, 255);
	color[4] = Scalar(255, 0, 255);
	color[5] = Scalar(0, 255, 255);
	color[6] = Scalar(192, 192, 192);

	centerGuide[0] = Vec4i(320, 200, 320, 240);
	centerGuide[1] = Vec4i(320, 240, 320, 280);
	centerGuide[2] = Vec4i(320, 280, 320, 320);
	centerGuide[3] = Vec4i(320, 320, 320, 360);
	cout << "settingStatic" << endl;
}

int calculSteer(Mat& src)
{
	if (!first++) settingStatic();
	int retval;
	stringstream ss;
	string text1, text2;
	Mat src_yel;
	Mat src_can;

	lineFiltering(src, src_yel, 0);
	cannyEdge(src_yel, src_can);
	Vec8i l = split_ransacLine(src_can, src, 640, 360, 13, 20, 1);
	Vec4i leftLine(l[0], l[1], l[2], l[3]);
	Vec4i rightLine(l[4], l[5], l[6], l[7]);

	if (slopeSign(leftLine) == 0 && slopeSign(rightLine) == 0)
	{
		//양쪽 직선 모두 없다.
		putText(src, "NONE", Point(260, 100), FONT_HERSHEY_COMPLEX, 0.8, Scalar(255, 255, 255), 2);
	}
	else if (slopeSign(leftLine) == slopeSign(rightLine) || abs(slope(leftLine)) < 1.0 || abs(slope(rightLine)) < 1.0)
	{
		//기울기 부호가 같다 == 곡선.
		//기울기 부호가 1.2미만 == 곡선
		int lineCenterY = (leftLine[3] > rightLine[1]) ? leftLine[3] : rightLine[1];
		int proximity = ((lineCenterY - 200.0) / (160.0)) * 500;
		if (proximity < 0) proximity = 0;

		for (int i = 0; i < 4; i++)
		{
			//가이드 직선 표시.
			if (lineCenterY > centerGuide[i][1])
				rectangle(src, Point(centerGuide[i][0] - 5, centerGuide[i][1]), Point(centerGuide[i][2] + 5, centerGuide[i][3]), Scalar(150, 150, 150), -1);
			else
				rectangle(src, Point(centerGuide[i][0] - 10, centerGuide[i][1]), Point(centerGuide[i][2] + 10, centerGuide[i][3]), color[i], -1);
		}

		cout << "(곡선)proximity =" << proximity << endl;
		text1.append("proximity = ");
		ss << proximity;
		ss >> text2;
		putText(src, text1 + text2, Point(210, 100), FONT_HERSHEY_COMPLEX, 0.8, Scalar(255, 255, 255), 2);
		if (slopeSign(leftLine) + slopeSign(rightLine) < 0)
		{
			//기울기 음수 = (/)
			putText(src, "(->)", Point(290, 130), FONT_HERSHEY_COMPLEX, 0.8, Scalar(255, 255, 255), 2);
			retval = proximity;
		}
		else
		{
			putText(src, "(<-)", Point(290, 130), FONT_HERSHEY_COMPLEX, 0.8, Scalar(255, 255, 255), 2);
			retval = -proximity;
		}
	}
	else //기울기 부호가 다르다 == 직선
	{
		//가이드 직선 표시.
		line(src, Point(leftGuide[0], leftGuide[1]), Point(leftGuide[2], leftGuide[3]), mint, 8);
		line(src, Point(rightGuide[0], rightGuide[1]), Point(rightGuide[2], rightGuide[3]), mint, 8);

		//기울기 절대값이 1.0미만이면 곡선
		int Deviation = 0;
		if (leftLine[1] != -1)
			Deviation += lineDeviation(leftLine, leftGuide);
		if (rightLine[1] != -1)
			Deviation += lineDeviation(rightLine, rightGuide);
		cout << "(직선)Deviation =" << Deviation << endl;
		text1.append("Deviation = ");
		ss << Deviation;
		ss >> text2;
		putText(src, text1 + text2, Point(210, 100), FONT_HERSHEY_COMPLEX, 0.8, Scalar(255, 255, 255), 2);
		putText(src, (Deviation < 0) ? "(<-)" : "(->)", Point(290, 130), FONT_HERSHEY_COMPLEX, 0.8, Scalar(255, 255, 255), 2);
		retval = Deviation;
	}
	line(src, Point(leftLine[0], leftLine[1]), Point(leftLine[2], leftLine[3]), pink, 5);
	line(src, Point(rightLine[0], rightLine[1]), Point(rightLine[2], rightLine[3]), pink, 5);

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

void houghLineDetection(Mat& src, Mat& dst, bool printMode)//, Vec4i& firstLine, Vec4i& secondLine)
{
	vector<Vec4i> lines;		//검출될 직선이 저장될 객체

	//createTrackbar("H_thresh", "trackbar", &HLP_threshold, 120, on_trackbar);
	//createTrackbar("H_minLen", "trackbar", &HLP_minLineLength, 200, on_trackbar);
	//createTrackbar("H_maxGap", "trackbar", &HLP_maxLineGap, 500, on_trackbar);
	//namedWindow("trackbar", WINDOW_NORMAL);
	//moveWindow("trackbar", 320 * 5, 40);

	HoughLinesP(src, lines, 1, CV_PI / 180, HLP_threshold, HLP_minLineLength, HLP_maxLineGap);

	for (unsigned int j = 0; j < lines.size(); j++)
	{
		line(dst, Point(lines[j][0], lines[j][1]), Point(lines[j][2], lines[j][3]), color[j], 2);
		circle(dst, Point2i(lines[j].operator[](0), lines[j].operator[](1)), 5, color[j], -1, 8);
		circle(dst, Point2i(lines[j].operator[](2), lines[j].operator[](3)), 5, color[j], -1, 8);
	}
}

void cannyEdge(Mat& src, Mat& dst)
{
	int threshold_1 = 118;		//215 //340
	int threshold_2 = 242;		//330 //500

	Canny(src, dst, threshold_1, threshold_2);	//노란색만 남은 frame의 윤곽을 1채널 Mat객체로 추출
}

Vec4i ransacLine(Mat& src, Mat& dst, int w, int h, double T, int n, bool printMode)
{
	stringstream ss;
	string text1, text2;
	Point printPoint(20, 200);
	vector<Point2i> P;
	Vec4i resultLine;
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
	if (P.size() < 100.0 * (360 / w))
	{
		//src에 존재하는 픽셀이 100 미만이면 패스
		putText(dst, "No edge point", printPoint, FONT_HERSHEY_COMPLEX, 0.8, Scalar(255, 255, 255), 2);
		return Vec4i(-1, -1, -1, -1);
	}

	int cntMax(-1);
	int checkCnt(0);
	double inlierPercent;
	srand(time(NULL));
	for (int i = 0; i < n; i++)
	{
		//임의의 두점을 정하고 그것들을 이어서 직선 만들기
		int cntInlier = 0;
		int cnt = 0;
		Point2i P1, P2;
		P1 = P[rand() % P.size()];
		P2 = P[rand() % P.size()];
		if (P1 == P2) { i--; continue; }

		Vec4i checkLine(P1.x, P1.y, P2.x, P2.y);
		for (unsigned int j = 0; j < P.size(); j += 20)
		{
			//임의의 직선과 나머지 직선의 거리를 비교하여 임계치 조사.
			Point2i calculPoint = P[j];
			cnt++;
			if (distance_between_line_and_point(checkLine, calculPoint, w, h) < T) cntInlier++;
		}

		if (cntInlier > cntMax)
		{
			//inlier가 가장많은 직선을 남긴다.
			resultLine = checkLine;
			cntMax = cntInlier;
			checkCnt = cnt;
		}
	}
	inlierPercent = ((double)cntMax / checkCnt) * 100.0;
	if (inlierPercent < 47)
	{
		//inlier가 일정수치 미만이면 패스.
		putText(dst, "No line", printPoint, FONT_HERSHEY_COMPLEX, 0.8, Scalar(255, 255, 255), 2);
		text1.assign("inlier(%) = ");
		ss << (int)inlierPercent;
		ss >> text2;
		putText(dst, text1 + text2 + "%", printPoint + Point(0, 30), FONT_HERSHEY_COMPLEX, 0.8, Scalar(255, 255, 255), 2);
		return Vec4i(-1, -1, -1, -1);
	}
	if (printMode == 1)
	{
		text1.assign("P size = ");
		ss << P.size();
		ss >> text2;
		putText(dst, text1 + text2, printPoint, FONT_HERSHEY_COMPLEX, 0.8, Scalar(255, 255, 255), 2);
		text1.assign("cntMax = ");
		ss << cntMax;
		ss >> text2;
		putText(dst, text1 + text2, printPoint + Point(0, 30), FONT_HERSHEY_COMPLEX, 0.8, Scalar(255, 255, 255), 2);
		text1.assign("inlier(%) = ");
		ss << (int)inlierPercent;
		ss >> text2;
		putText(dst, text1 + text2 + "%", printPoint + Point(0, 60), FONT_HERSHEY_COMPLEX, 0.8, Scalar(255, 255, 255), 2);
	}

	text1.assign("slope = ");
	ss << slope(resultLine);
	ss >> text2;
	putText(dst, text1 + text2 , printPoint + Point(0, 90), FONT_HERSHEY_COMPLEX, 0.8, Scalar(255, 255, 255), 2);
	return resultLine;

}

Vec8i split_ransacLine(Mat& src, Mat& dst, int w, int h, double T, int n, bool printMode)
{
	//화면을 두개로 쪼개어 ransacline을 찾는다.
	Mat src1 = src(Range(0, h), Range(0, w / 2));
	Mat src2 = src(Range(0, h), Range(w / 2, w));
	Mat dst_right = dst(Range(0, h), Range(w / 2, w));

	Vec4i leftLine = ransacLine(src1, dst, w / 2, h, T, n, printMode);
	Vec4i rightLine = ransacLine(src2, dst_right, w / 2, h, T, n, printMode);
	if ((leftLine[0] != leftLine[2]) && (rightLine[0] != rightLine[2]))
	{
		//왼쪽 직선과 오른쪽 직선이 모두 있다면
		if (slopeSign(leftLine) == slopeSign(rightLine))
		{
			//왼쪽 직선과 오른쪽 직선의 부호가 같다면.
			//좌우 직선의 가운데점을 중점으로맞춘다.
			int center = (leftLine[3] + rightLine[1]) / 2;
			leftLine[3] = center;
			rightLine[1] = center;
		}
	}
	line(dst, Point(leftLine[0], leftLine[1]), Point(leftLine[2], leftLine[3]), pink, 5);
	line(dst_right, Point(rightLine[0], rightLine[1]), Point(rightLine[2], rightLine[3]), pink, 5);

	return Vec8i(leftLine[0], leftLine[1], leftLine[2], leftLine[3],
		rightLine[0] + (w / 2), rightLine[1], rightLine[2] + (w / 2), rightLine[3]);
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

int lineDeviation(Vec4i line1, Vec4i line2)
{
	//y = 50에서 직선끼리의 거리 비교.
	return getPointX_at_Y(line1) - getPointX_at_Y(line2);
}

int getPointX_at_Y(Vec4i line)
{
	double m, b;
	m = slope(line);
	b = line[1] - m * line[0];

	return ((100 - b) / m);
}