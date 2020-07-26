#ifndef IMGPROCESS_H_
#define IMGPROCESS_H_ 

#ifdef __cplusplus
extern "C" {
#endif

/****************************
* 영상처리 함수 작성
*****************************/


	void OpenCV_calibration(float* map1, float* map2, int w, int h);
	void OpenCV_remap(unsigned char* inBuf, int w, int h, unsigned char* outBuf, float* map1, float* map2);
	void OpenCV_topview_transform(unsigned char* inBuf, int w, int h, unsigned char* outBuf, int mode);


#ifdef __cplusplus
}
#endif


int calculSteer(Mat& src);
//PreCondition	: frame은 자동차의 topview2영상을 입력으로 받는다.
//PostCondition	: 차선을 인식하여 인식된 결과를 src에 표시한다..
//Return : 조향해야될 결과값을 반환한다(-500 ~ 500)

void lineFiltering(Mat& src, Mat& dst, int mode);
//PreCondition	: 없음
//PostCondition	: 노란색(or 흰색)차선이 존재하는 src영상에서 차선영역만 남게 필터링하여 dst로 반환한다.
//Return : 없음

void houghLineDetection(Mat& src, Mat& dst, bool printMode);//, Vec4i& firstLine, Vec4i& secondLine);
//PreCondition	: 없음.
//PostCondition	: houghLineP 함수 결과를 dst에 나타내준다.
//Return : 없음

void cannyEdge(Mat& src, Mat& dst);
//PreCondition	: 없음
//PostCondition	: canny함수 결과를 dst로 출력한다.
//Return : 없음

Vec4i ransacLine(Mat& src, Mat& dst, int w, int h, double T, int n, bool printMode);
//PreCondition	: src영상은 canny처리를 거친 결과여야한다.
//PostCondition	: dst에 검출된 직선을 핑크색으로 표시한다.
//Return : 검출된 라인이 반환(Vec4i)

Vec8i split_ransacLine(Mat& src, Mat& dst, int w, int h, double T, int n, bool printMode);
//PreCondition	: src영상은 canny처리를 거친 결과여야한다.
//PostCondition	: src영상을 좌우로 분할하여 검출한다. 그외 동일.
//Return : 검출된 라인 두개를 반환(Vec8i)
//			return Vec8i(leftLine[0], leftLine[1], leftLine[2], leftLine[3],
//						rightLine[0], rightLine[1], rightLine[2], rightLine[3]);

double distance_between_line_and_point(Vec4i& line, Point2i point, int w, int h);

int slopeSign(Vec4i line);

double slope(Vec4i line);

int lineDeviation(Vec4i line1, Vec4i line2);

int getPointX_at_Y(Vec4i line);


#endif //IMGPROCESS_H_

