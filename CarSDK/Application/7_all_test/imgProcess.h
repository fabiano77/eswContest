#ifndef IMGPROCESS_H_
#define IMGPROCESS_H_ 

#ifdef __cplusplus
extern "C" {
#endif

/****************************
* ����ó�� �Լ� �ۼ�
*****************************/


	void OpenCV_calibration(float* map1, float* map2, int w, int h);
	void OpenCV_remap(unsigned char* inBuf, int w, int h, unsigned char* outBuf, float* map1, float* map2);
	void OpenCV_topview_transform(unsigned char* inBuf, int w, int h, unsigned char* outBuf, int mode);


#ifdef __cplusplus
}
#endif


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


#endif //IMGPROCESS_H_

