#ifndef IMGPROCESS_H_
#define IMGPROCESS_H_ 

#ifdef __cplusplus
extern "C" {
#endif

	void cSettingStatic(int w, int h);

	int calculSteer(Mat& src, int w, int h, bool whiteMode);
	//PreCondition	: frame은 자동차의 topview2영상을 입력으로 받는다.
	//PostCondition	: 차선을 인식하여 인식된 결과를 src에 표시한다..
	//Return : 조향해야될 결과값을 반환한다(-500 ~ 500)

	void lineFiltering(Mat& src, Mat& dst, int mode);

	void cannyEdge(Mat& src, Mat& dst);

	Vec8i hough_ransacLine(Mat& src, Mat& dst, int w, int h, int T, bool printMode, int& detectedLineType, const double lowThresAngle);

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

	int isDark(Mat& frame, const double percent, int debug);

	int Tunnel_isStart(Mat& frame, const double percent);

	bool priorityStop(Mat& src, Mat& dst, double percent, bool debug);

	int checkRedSignal(Mat& src, Mat& dst, double percent, bool debug);

	int checkYellowSignal(Mat& src, Mat& dst, double percent, bool debug);

	int checkGreenSignal(Mat& src, Mat& dst, double percent, bool debug);

	int countPixel(Mat& src, Rect ROI);

	void outputMission(Mat& dst, int ms0, int ms1, int ms2, int ms3, int ms4, int ms5, int ms6, int ms7, int ms8);

	void outputSensor(Mat& dst, int w, int h, int c1, int c2, int c3, int c4, int c5, int c6, int stopline);

	void fileOutVideo(Mat& src);

	void fileOutimage(Mat& src, string str);

	void closeVideoWrite();


#ifdef __cplusplus
}
#endif


#endif //IMGPROCESS_H_

