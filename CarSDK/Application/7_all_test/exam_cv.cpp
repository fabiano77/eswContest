
#include <iostream>
#include <stdio.h>
#include <string.h>
#include <sstream>
//#include <sys/time.h>

#include <opencv2/calib3d/calib3d.hpp>
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



	void OpenCV_calibration(float* map1, float* map2, int w, int h) {
		// DoCalib�� initUndistortRectifyMap �Լ��� �̿��ؼ� 
		Size videoSize = Size(w, h);
		Mat Mat_map1(h, w, CV_32FC1, map1);
		Mat Mat_map2(h, w, CV_32FC1, map2);



		Mat disCoeffs;
		Mat cameraMatrix = Mat(3, 3, CV_32FC1);
		int numBoards = 4;
		//���� DoCalib() �Լ� ����.

		int numCornerHor = 7; // ���� ���� ����
		int numCornerVer = 7; // ���� ���� ����
		int numSquare = numCornerHor * numCornerVer; // �簢���� ����

		Size board_sz = Size(numCornerHor, numCornerVer);

		vector <vector <Point3f> > object_point; // ���� 3D ������ �ǹ��ϴ� ��ä
		vector <vector <Point2f> > image_point; // ü�� ���� XY��鿡 ��ġ��Ų ���¸� �ǹ��ϴ� ��ä

		vector <Point2f> corners;
		int successes = 0; // �ϳ��� �̹����� ���� Calib�� ������ �� ���Ǵ� ī����
						   // ü�� ���� �̹��� �� ��ŭ ����Ǿ�� �Ѵ�.

		Mat image; // ü�� ���� �̹����� ������ Mat ��ä
		Mat gray_image;

		vector <Point3f> obj;
		for (int i = 0; i < numSquare; i++) {
			obj.push_back(Point3f(i / numCornerHor, i % numCornerHor, 0.0f));
			//ü�� �ǿ� ���� XY�࿡ ������ ���·� �����Ѵ�.
			//��ǥ�� ���� ������ �ý����� ���Ͽ� obj ���Ϳ� ü�� ���� ������ ��ǥ�� ��� �����Ѵ�.
		   // (0~numCornerVer, 0~numCornerHor, 0.0f) ����
		}

		ostringstream osstream; // imread�� ���� sstream

		while (successes < numBoards) { // ��� ü�� ���� ������ ó�� �� �� ���� loop�� �����Ѵ�.

			osstream.str("");
			osstream << "./Calib_img/frame_" << 1280 << "_" << successes << ".png"; // ������ ���� ó��

			image = imread(osstream.str(), IMREAD_COLOR); // ù ��° �������� image��ä�� read��Ų��.

			cout << osstream.str() << ", size = " << image.size() << endl;
			resize(image, image, Size(w, h), 0, 0, CV_INTER_LINEAR); // 사이즈변환.

			if (image.empty()) { // image ��ä�� ���� ���� ����
				cout << "IMAGE IS EMPTY!" << endl;
				return;
			}

			cvtColor(image, gray_image, COLOR_BGR2GRAY);

			bool found = findChessboardCorners(image, board_sz, corners, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_FILTER_QUADS);
			//ù ��° ���ڴ� 3-ä�� color �̹����� InputOutputArray�� ���;��Ѵ�.
			//�� ��° ���ڴ� ü�� ���� �𼭸��� Size�̴�. ĭ�� ���� �ƴ� �𼭸��� ���� ����.
			//�� ��° ���ڴ� ����� �𼭸��� ��ǥ�� �Էµȴ�.

			if (found) {
				//cornerSubPix(gray_image, corners, Size(11, 11), Size(-1, -1), TermCriteria(TermCriteria::EPS | TermCriteria::MAX_ITER), 30, 0.1);
				drawChessboardCorners(image, board_sz, corners, found);
			}
			//imshow("COLOR", image);
			//imshow("GRAY", gray_image);
			//waitKey(10);

			//if (key == 27) // esc �Է� �� ����
			//	return 0;

			//if (key == ' ' && found != 0) { // space bar �Է� �� ��ǥ ���� ��ä�� �����

			image_point.push_back(corners);
			object_point.push_back(obj);

			cout << successes + 1 << "th snap stored!" << endl; // Consoleâ�� ���


			successes++; // ���� ������ ���� loop ����

			osstream.clear(); // ������ ���� osstream �ʱ�ȭ

			if (successes >= numBoards) // ��� ������ ���� ����� �Ϸ�Ǹ� loop Ż��
				break;
			/*}*/

		}

		vector <Mat> rvecs;
		vector <Mat> tvecs;

		cameraMatrix.ptr<float>(0)[0] = 1;
		cameraMatrix.ptr<float>(0)[0] = 1;

		calibrateCamera(object_point, image_point, image.size(), cameraMatrix, disCoeffs, rvecs, tvecs);
		//������� DoCalib()�Լ�.

		// disCoeffs�� cameraMatrix�� ������ ����.
		initUndistortRectifyMap(cameraMatrix, disCoeffs, Mat(), cameraMatrix, videoSize, CV_32FC1, Mat_map1, Mat_map2);

		// Mat_map2�� ������ map2�� �����Ѵ�.
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
		// �ɴ� �˰����� ����
		cv::Mat contours;
		cv::Canny(srcGRAY, // �׷��̷��� ����
			contours, // ��� �ܰ���
			125,  // ���� ��谪
			350);  // ���� ��谪

		// ������ ȭ�ҷ� �ܰ����� ǥ���ϹǷ� ��� ���� ����
		//cv::Mat contoursInv; // ���� ����
		//cv::threshold(contours, contoursInv, 128, 255, cv::THRESH_BINARY_INV);
		// ��� ���� 128���� ������ 255�� �ǵ��� ����

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

		// ĳ�� �˰����� ����
		cv::Mat contours;
		cv::Canny(srcRGB, contours, 125, 350);

		// �� ���� ���� ���� ��ȯ
		std::vector<cv::Vec2f> lines;
		cv::HoughLines(contours, lines, 1, PI / 180, // �ܰ躰 ũ�� (1�� ��/180���� �ܰ躰�� ������ ��� ������ �������� ���� ã��)
			80);  // ��ǥ(vote) �ִ� ����

		// �� �׸���
		cv::Mat result(contours.rows, contours.cols, CV_8UC3, lineColor);
		//printf("Lines detected: %d\n", lines.size());

		// �� ���͸� �ݺ��� �� �׸���
		std::vector<cv::Vec2f>::const_iterator it = lines.begin();
		while (it != lines.end())
		{
			float rho = (*it)[0];   // ù ��° ��Ҵ� rho �Ÿ�
			float theta = (*it)[1]; // �� ��° ��Ҵ� ��Ÿ ����

			if (theta < PI / 4. || theta > 3. * PI / 4.) // ���� ��
			{
				cv::Point pt1(rho / cos(theta), 0); // ù �࿡�� �ش� ���� ������   
				cv::Point pt2((rho - result.rows * sin(theta)) / cos(theta), result.rows);
				// ������ �࿡�� �ش� ���� ������
				cv::line(srcRGB, pt1, pt2, lineColor, 1); // �Ͼ� ������ �׸���

			}
			else // ���� ��
			{
				cv::Point pt1(0, rho / sin(theta)); // ù ��° ������ �ش� ���� ������  
				cv::Point pt2(result.cols, (rho - result.cols * cos(theta)) / sin(theta));
				// ������ ������ �ش� ���� ������
				cv::line(srcRGB, pt1, pt2, lineColor, 1); // �Ͼ� ������ �׸���
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
		
		if(mode == 1)
		{
			Point2f Hp[4] = {	//변환전 좌표
				Point2f(160*(w/640.0), 180*(h/360.0)),
				Point2f(480*(w/640.0), 180*(h/360.0)),
				Point2f(620*(w/640.0), 270*(h/360.0)),
				Point2f(20*(w/640.0), 270*(h/360.0)) };

			Point2f p[4] = {	//변환후 좌표
				Point2f(100*(w/640.0), -100*(h/360.0)),
				Point2f(540*(w/640.0), -100*(h/360.0)),
				Point2f(550*(w/640.0), 270*(h/360.0)),
				Point2f(90*(w/640.0), 270*(h/360.0)) };
				Hmatrix = getPerspectiveTransform(Hp, p);
		}
		else if(mode == 2)
		{
			Point2f Hp[4] = {	//변환전 좌표
				Point2f(80*(w/640.0), 200*(h/360.0)),
				Point2f(560*(w/640.0), 200*(h/360.0)),
				Point2f(640*(w/640.0), 360*(h/360.0)),
				Point2f(0*(w/640.0), 360*(h/360.0)) };

			Point2f p[4] = {	//변환후 좌표
				Point2f(0*(w/640.0), 50*(h/360.0)),
				Point2f(640*(w/640.0), 50*(h/360.0)),
				Point2f(640*(w/640.0), 360*(h/360.0)),
				Point2f(0*(w/640.0), 360*(h/360.0)) };
				Hmatrix = getPerspectiveTransform(Hp, p);
		}

		Size topviewSize(w, h);	//변환후 사이즈
		warpPerspective(srcRGB, dstRGB, Hmatrix, topviewSize);

	}

}

