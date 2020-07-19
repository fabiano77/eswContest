
#include <iostream>
#include <stdio.h>
#include <string.h>
#include <sstream>
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

	/**
	  * @brief  To load image file to the buffer.
	  * @param  file: pointer for load image file in local path
				 outBuf: destination buffer pointer to load
				 nw : width value of the destination buffer
				 nh : height value of the destination buffer
	  * @retval none
	  */
	void OpenCV_load_file(char* file, unsigned char* outBuf, int nw, int nh)
	{
		Mat srcRGB;
		Mat dstRGB(nh, nw, CV_8UC3, outBuf);

		srcRGB = imread(file, CV_LOAD_IMAGE_COLOR); // rgb
		//cvtColor(srcRGB, srcRGB, CV_RGB2BGR);

		cv::resize(srcRGB, dstRGB, cv::Size(nw, nh), 0, 0, CV_INTER_LINEAR);
	}



	void OpenCV_calibration(char* map1, char* map2) {
		// DoCalib와 initUndistortRectifyMap 함수를 이용해서 
		Size videoSize = Size(320, 180);
		Mat Mat_map1(320, 180, CV_8S, map1);
		Mat Mat_map2(320, 180, CV_8S, map2);
		Mat disCoeffs;
		Mat cameraMatrix = Mat(3, 3, CV_32FC1);
		int numBoards = 5;

		//이하 DoCalib() 함수 전문.

		int numCornerHor = 7; // 수평 점의 개수
		int numCornerVer = 7; // 수직 점의 개수
		int numSquare = numCornerHor * numCornerVer; // 사각형의 개수

		Size board_sz = Size(numCornerHor, numCornerVer);

		vector<vector<Point3f>> object_point; // 실제 3D 차원을 의미하는 객채
		vector<vector<Point2f>> image_point; // 체스 판을 XY평면에 위치시킨 상태를 의미하는 객채

		vector<Point2f> corners;
		int successes = 0; // 하나의 이미지에 대해 Calib을 수행할 때 사용되는 카운터
						   // 체스 판의 이미지 수 만큼 수행되어야 한다.

		Mat image; // 체스 판의 이미지를 저장할 Mat 객채
		Mat gray_image;

		vector<Point3f> obj;
		for (int i = 0; i < numSquare; i++) {
			obj.push_back(Point3f(i / numCornerHor, i % numCornerHor, 0.0f));
			//체스 판에 대해 XY축에 고정된 상태로 가정한다.
			//좌표에 대한 랜덤한 시스템을 위하여 obj 벡터에 체스 판의 가능한 좌표를 모두 저장한다.
		   // (0~numCornerVer, 0~numCornerHor, 0.0f) 범위
		}

		ostringstream osstream; // imread를 위한 sstream

		while (successes < numBoards) { // 모든 체스 판의 사진을 처리 할 때 까지 loop를 실행한다.

			osstream.str("");
			osstream << "CarSDK/Calib_img/frame_" << successes << ".png"; // 사진의 제목 처리

			image = imread(osstream.str(), IMREAD_COLOR); // 첫 번째 사진부터 image객채에 read시킨다.

			if (image.empty()) { // image 객채에 대한 오류 검출
				cout << "IMAGE IS EMPTY!" << endl;
				return;
			}

			cvtColor(image, gray_image, COLOR_BGR2GRAY);

			bool found = findChessboardCorners(image, board_sz, corners, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_FILTER_QUADS);
			//첫 번째 인자는 3-채널 color 이미지인 InputOutputArray가 들어와야한다.
			//두 번째 인자는 체스 판의 모서리의 Size이다. 칸의 수가 아닌 모서리의 수로 센다.
			//세 번째 인자는 검출된 모서리의 좌표가 입력된다.

			if (found) {
				//cornerSubPix(gray_image, corners, Size(11, 11), Size(-1, -1), TermCriteria(TermCriteria::EPS | TermCriteria::MAX_ITER), 30, 0.1);
				drawChessboardCorners(image, board_sz, corners, found);
			}
			//imshow("COLOR", image);
			//imshow("GRAY", gray_image);
			//waitKey(10);

			//if (key == 27) // esc 입력 시 종료
			//	return 0;

			//if (key == ' ' && found != 0) { // space bar 입력 시 좌표 값이 객채로 저장됨

			image_point.push_back(corners);
			object_point.push_back(obj);

			cout << successes + 1 << "th snap stored!" << endl; // Console창에 출력

			successes++; // 다음 사진에 대해 loop 실행

			osstream.clear(); // 제목을 담을 osstream 초기화

			if (successes >= numBoards) // 모든 사진에 대해 계산이 완료되면 loop 탈출
				break;
			/*}*/

		}

		vector<Mat> rvecs;
		vector<Mat> tvecs;

		intrinsic.ptr<float>(0)[0] = 1;
		intrinsic.ptr<float>(0)[0] = 1;

		calibrateCamera(object_point, image_point, image.size(), intrinsic, distCoeffs, rvecs, tvecs);
		//여기까지 DoCalib()함수.

		// disCoeffs와 cameraMatrix에 정보가 담긴다.
		initUndistortRectifyMat(cameraMatrix, distCoeffs, Mat(), cameraMatrix, videoSize, CV_32FC1, Mat_map1, Mat_map2);
		// Mat_map2의 정보를 map2에 복사한다.
	}

	/**
	  * @brief  To convert format from BGR to RGB.
	  * @param  inBuf: buffer pointer of BGR image
				 w: width value of the buffers
				 h : height value of the buffers
				 outBuf : buffer pointer of RGB image
	  * @retval none
	  */
	void OpenCV_Bgr2RgbConvert(unsigned char* inBuf, int w, int h, unsigned char* outBuf)
	{
		Mat srcRGB(h, w, CV_8UC3, inBuf);
		Mat dstRGB(h, w, CV_8UC3, outBuf);

		cvtColor(srcRGB, dstRGB, CV_BGR2RGB);
	}

	/**
	  * @brief  Detect faces on loaded image and draw circles on the faces of the loaded image.
	  * @param  file: pointer for load image file in local path
				 outBuf: buffer pointer to draw circles on the detected faces
				 nw : width value of the destination buffer
				 nh : height value of the destination buffer
	  * @retval none
	  */
	void OpenCV_face_detection(char* file, unsigned char* outBuf, int nw, int nh)
	{
		Mat srcRGB = imread(file, CV_LOAD_IMAGE_COLOR);
		Mat dstRGB(nh, nw, CV_8UC3, outBuf);

		// Load Face cascade (.xml file)
		CascadeClassifier face_cascade;
		face_cascade.load("haarcascade_frontalface_alt.xml");

		// Detect faces
		std::vector<Rect> faces;
		face_cascade.detectMultiScale(srcRGB, faces, 1.1, 2, 0 | CV_HAAR_SCALE_IMAGE, Size(30, 30));

		// Draw circles on the detected faces
		for (int i = 0; i < faces.size(); i++)
		{
			Point center(faces[i].x + faces[i].width * 0.5, faces[i].y + faces[i].height * 0.5);
			ellipse(srcRGB, center, Size(faces[i].width * 0.5, faces[i].height * 0.5), 0, 0, 360, Scalar(255, 0, 255), 4, 8, 0);
		}

		cv::resize(srcRGB, dstRGB, cv::Size(nw, nh), 0, 0, CV_INTER_LINEAR);
	}

	/**
	  * @brief  To bind two images on destination buffer.
	  * @param  file1: file path of first image to bind
				 file2: file path of second image to bind
				 outBuf : destination buffer pointer to bind
				 nw : width value of the destination buffer
				 nh : height value of the destination buffer
	  * @retval none
	  */
	void OpenCV_binding_image(char* file1, char* file2, unsigned char* outBuf, int nw, int nh)
	{
		Mat srcRGB = imread(file1, CV_LOAD_IMAGE_COLOR);
		Mat srcRGB2 = imread(file2, CV_LOAD_IMAGE_COLOR);
		Mat dstRGB(nh, nw, CV_8UC3, outBuf);

		cv::resize(srcRGB2, srcRGB2, cv::Size(srcRGB2.cols / 1.5, srcRGB2.rows / 1.5));
		cv::Point location = cv::Point(280, 220);
		for (int y = std::max(location.y, 0); y < srcRGB.rows; ++y)
		{
			int fY = y - location.y;
			if (fY >= srcRGB2.rows)
				break;

			for (int x = std::max(location.x, 0); x < srcRGB.cols; ++x)
			{
				int fX = x - location.x;
				if (fX >= srcRGB2.cols)
					break;

				double opacity = ((double)srcRGB2.data[fY * srcRGB2.step + fX * srcRGB2.channels() + 3]) / 255.;
				for (int c = 0; opacity > 0 && c < srcRGB.channels(); ++c)
				{
					unsigned char overlayPx = srcRGB2.data[fY * srcRGB2.step + fX * srcRGB2.channels() + c];
					unsigned char srcPx = srcRGB.data[y * srcRGB.step + x * srcRGB.channels() + c];
					srcRGB.data[y * srcRGB.step + srcRGB.channels() * x + c] = srcPx * (1. - opacity) + overlayPx * opacity;
				}
			}
		}

		cv::resize(srcRGB, dstRGB, cv::Size(nw, nh), 0, 0, CV_INTER_LINEAR);
	}

	/**
	  * @brief  Apply canny edge algorithm and draw it on destination buffer.
	  * @param  file: pointer for load image file in local path
				 outBuf: destination buffer pointer to apply canny edge
				 nw : width value of destination buffer
				 nh : height value of destination buffer
	  * @retval none
	  */
	void OpenCV_canny_edge_image(char* file, unsigned char* outBuf, int nw, int nh)
	{
		Mat srcRGB = imread(file, CV_LOAD_IMAGE_COLOR);
		Mat srcGRAY;
		Mat dstRGB(nh, nw, CV_8UC3, outBuf);

		cvtColor(srcRGB, srcGRAY, CV_BGR2GRAY);
		// 케니 알고리즘 적용
		cv::Mat contours;
		cv::Canny(srcGRAY, // 그레이레벨 영상
			contours, // 결과 외곽선
			125,  // 낮은 경계값
			350);  // 높은 경계값

		// 넌제로 화소로 외곽선을 표현하므로 흑백 값을 반전
		//cv::Mat contoursInv; // 반전 영상
		//cv::threshold(contours, contoursInv, 128, 255, cv::THRESH_BINARY_INV);
		// 밝기 값이 128보다 작으면 255가 되도록 설정

		cvtColor(contours, contours, CV_GRAY2BGR);

		cv::resize(contours, dstRGB, cv::Size(nw, nh), 0, 0, CV_INTER_LINEAR);
	}

	/**
	  * @brief  Detect the hough and draw hough on destination buffer.
	  * @param  srcBuf: source pointer to hough transform
				 iw: width value of source buffer
				 ih : height value of source buffer
				 outBuf : destination pointer to hough transform
				 nw : width value of destination buffer
				 nh : height value of destination buffer
	  * @retval none
	  */
	void OpenCV_hough_transform(unsigned char* srcBuf, int iw, int ih, unsigned char* outBuf, int nw, int nh)
	{
		Scalar lineColor = cv::Scalar(255, 255, 255);

		Mat dstRGB(nh, nw, CV_8UC3, outBuf);

		Mat srcRGB(ih, iw, CV_8UC3, srcBuf);
		Mat resRGB(ih, iw, CV_8UC3);
		//cvtColor(srcRGB, srcRGB, CV_BGR2BGRA);

		// 캐니 알고리즘 적용
		cv::Mat contours;
		cv::Canny(srcRGB, contours, 125, 350);

		// 선 감지 위한 허프 변환
		std::vector<cv::Vec2f> lines;
		cv::HoughLines(contours, lines, 1, PI / 180, // 단계별 크기 (1과 π/180에서 단계별로 가능한 모든 각도로 반지름의 선을 찾음)
			80);  // 투표(vote) 최대 개수

		// 선 그리기
		cv::Mat result(contours.rows, contours.cols, CV_8UC3, lineColor);
		//printf("Lines detected: %d\n", lines.size());

		// 선 벡터를 반복해 선 그리기
		std::vector<cv::Vec2f>::const_iterator it = lines.begin();
		while (it != lines.end())
		{
			float rho = (*it)[0];   // 첫 번째 요소는 rho 거리
			float theta = (*it)[1]; // 두 번째 요소는 델타 각도

			if (theta < PI / 4. || theta > 3. * PI / 4.) // 수직 행
			{
				cv::Point pt1(rho / cos(theta), 0); // 첫 행에서 해당 선의 교차점   
				cv::Point pt2((rho - result.rows * sin(theta)) / cos(theta), result.rows);
				// 마지막 행에서 해당 선의 교차점
				cv::line(srcRGB, pt1, pt2, lineColor, 1); // 하얀 선으로 그리기

			}
			else // 수평 행
			{
				cv::Point pt1(0, rho / sin(theta)); // 첫 번째 열에서 해당 선의 교차점  
				cv::Point pt2(result.cols, (rho - result.cols * cos(theta)) / sin(theta));
				// 마지막 열에서 해당 선의 교차점
				cv::line(srcRGB, pt1, pt2, lineColor, 1); // 하얀 선으로 그리기
			}
			//printf("line: rho=%f, theta=%f\n", rho, theta);
			++it;
		}

		cv::resize(srcRGB, dstRGB, cv::Size(nw, nh), 0, 0, CV_INTER_LINEAR);
	}

	/**
	  * @brief  Merge two source images of the same size into the output buffer.
	  * @param  src1: pointer to parameter of rgb32 image buffer
				 src2: pointer to parameter of bgr32 image buffer
				 dst : pointer to parameter of rgb32 output buffer
				 w : width of src and dst buffer
				 h : height of src and dst buffer
	  * @retval none
	  */
	void OpenCV_merge_image(unsigned char* src1, unsigned char* src2, unsigned char* dst, int w, int h)
	{
		Mat src1AR32(h, w, CV_8UC4, src1);
		Mat src2AR32(h, w, CV_8UC4, src2);
		Mat dstAR32(h, w, CV_8UC4, dst);

		cvtColor(src2AR32, src2AR32, CV_BGRA2RGBA);

		for (int y = 0; y < h; ++y) {
			for (int x = 0; x < w; ++x) {
				double opacity = ((double)(src2AR32.data[y * src2AR32.step + x * src2AR32.channels() + 3])) / 255.;
				for (int c = 0; opacity > 0 && c < src1AR32.channels(); ++c) {
					unsigned char overlayPx = src2AR32.data[y * src2AR32.step + x * src2AR32.channels() + c];
					unsigned char srcPx = src1AR32.data[y * src1AR32.step + x * src1AR32.channels() + c];
					src1AR32.data[y * src1AR32.step + src1AR32.channels() * x + c] = srcPx * (1. - opacity) + overlayPx * opacity;
				}
			}
		}

		memcpy(dst, src1AR32.data, w * h * 4);
	}

	void OpenCV_remap(unsigned char* inBuf, int w, int h, unsigned char* outBuf)
	{
		Mat srcRGB(h, w, CV_8UC3, inBuf);
		Mat dstRGB(h, w, CV_8UC3, outBuf);

		remap(srcRGB, dstRGB, map1, map2, INTER_LINEAR);
	}

}

