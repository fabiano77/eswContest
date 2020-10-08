#ifndef IMGPROCESS_H_
#define IMGPROCESS_H_ 

#ifdef __cplusplus
extern "C" {
#endif
	// cSettingStatic
	// 해상도에 맞추어 기본 변수들을 초기화해주는 함수
	// PreCondition 	: none
	// PostCondition	: 입력받은 해상도 w * h 에 맞추어 기본 변수들을 초기화 시킨다.
	// Return 			: none
	void cSettingStatic(int w, int h);
	
	// calibration
	// 카메라 행렬을 계산하여 저장하는 함수
	// PreCondition 	: none 
	// PostCondition	: /../../Calib_img 에 있는 카메라 영상으로 calibration을 하기위한
	//						camera Matrix를 map1, map2에 저장한다.
	// Return 			: none
	void calibration(float* map1, float* map2, int w, int h);

	// OpenCV_remap
	// 카메라 행렬을 이용하여 현재 영상의 왜곡을 풀어주는 함수
	// PreCondition 	: calibration()함수를 실행하여 map1, map2을 인자로 넘겨주어야한다.
	// PostCondition	: inBuf를 캘리브레이션하여 outBuf로 출력한다.
	// Return 			: none
	void OpenCV_remap(unsigned char* inBuf, int w, int h, unsigned char* outBuf, float* map1, float* map2);

	// topview_transform
	// 영상을 투영 변환하여 다른 시점으로 변화시키는 함수
	// PreCondition 	: none
	// PostCondition	: 입력영상 inBuf를 mode에 따라 시점변환하여 outBuf로 반환한다.
	//						mode 1: 수직 위에서 본 시점
	//						mode 2: 대각선 위에서 본 시점
	//						mode 3: 대각선 위에서 본 시점2
	// Return 			: none
	void topview_transform(unsigned char* inBuf, int w, int h, unsigned char* outBuf, int mode);

	// displayPrintStr
	// 영상에 원하는 (str)문자열을 표시해주는 함수
	// PreCondition 	: none
	// PostCondition	: outBuf로 받은 영상에 (string)name을 상단에 초록색상으로 출력한다.
	// Return 			: none
	void displayPrintStr(unsigned char* outBuf, int w, int h, char* name);

	// displayPrintMission
	// 영상에 현재 미션들의 진행상태를 간단하게 나타내주는 함수
	// PreCondition 	: none
	// PostCondition	: outBuf로 받은 영상에 ms1, ms2, ..., ms8 (각 미션 상태)를 출력한다.
	// Return 			: none
	void displayPrintMission(unsigned char* outBuf, int w, int h, 
		int ms0, int ms1, int ms2, int ms3, int ms4, int ms5, int ms6, int ms7, int ms8);

	// displayPrintSensor
	// 영상에 현재 각 거리센서값을 나타내주는 함수
	// PreCondition 	: none
	// PostCondition	: outBuf로 받은 영상에 거리센서 값과 하단 적외선 센서 결과를 출력한다.
	// Return 			: none
	void displayPrintSensor(unsigned char* outBuf, int w, int h,
		int c1, int c2, int c3, int c4, int c5, int c6, int stopline);

	// displayPrintStopLine
	// 영상에 하단 적외선센서 값을 출력해주는 함수
	// PreCondition 	: none
	// PostCondition	: outBuf로 받은 영상에 하단 적외선 센서 결과를 출력한다.
	// Return 			: none
	void displayPrintStopLine(unsigned char* outBuf, int w, int h);

	// overlayPrintAngle
	// 영상에 현재 앞바퀴의 각도를 그래픽으로 표시해주는 함수
	// PreCondition 	: none
	// PostCondition	: inBuf로 받은 영상에 현재 각(angle)을 표시하고,
	//						./../../overlay_pictures/~ 에 저장되어있는 바퀴 이미지를 
	//						angle만큼 회전변환하여 화면에 보여준다.
	// Return 			: none
	void overlayPrintAngle(unsigned char* inBuf, int w, int h, unsigned char* outBuf, int angle);

	// checkRed
	// 빨간색 신호등을 감지하는 함수
	// PreCondition 	: none
	// PostCondition	: 화면에 빨간색 신호등 감지값을 출력한다.
	// Return 			: 신호등 감지시 1, 아닐시 0
	int checkRed(unsigned char* inBuf, int w, int h, unsigned char* outBuf);

	// checkYellow
	// 노란색 신호등을 감지하는 함수
	// PreCondition 	: none
	// PostCondition	: 화면에 노란색 신호등 감지값을 출력한다.
	// Return 			: 신호등 감지시 1, 아닐시 0
	int checkYellow(unsigned char* inBuf, int w, int h, unsigned char* outBuf);

	// checkGreen
	// 초록색 신호등을 감지하는 함수
	// PreCondition 	: none
	// PostCondition	: 화면에 초록색 신호등 감지값을 출력한다.
	// Return 			: 신호등 우회전 판단시 1, 좌회전 판단시 -1, 아닐시 0
	int checkGreen(unsigned char* inBuf, int w, int h, unsigned char* outBuf);

	// isPriorityStop
	// 영상에서 우선정지 표지판을 감지하는 함수
	// PreCondition 	: none
	// PostCondition	: 화면에 우선정지 표지판 감지값을 출력한다.
	// Return 			: 감지시 true, 아닐시 false
	bool isPriorityStop(unsigned char* inBuf, int w, int h, unsigned char* outBuf);

	// calculDistance_FinishLine
	// 영상에서 전방 가로일직선과 차량의 거리를 측정해주는 함수
	// PreCondition 	: none
	// PostCondition	: none
	// Return 			: 전방 가로선과 차량의 거리를 (cm)단위로 반환한다.
	int calculDistance_FinishLine(unsigned char* inBuf, int w, int h, unsigned char* outBuf);

	// debugFiltering
	// 각종 영상처리 함수들을 포함하여 mode만 바꾸어 출력을 확인해볼수있는 디버깅 함수
	// PreCondition 	: none
	// PostCondition	: mode에 따라 원하는 영상처리의 
	// Return 			: none
	void debugFiltering(unsigned char* inBuf, int w, int h, unsigned char* outBuf, int mode);

	// autoSteering
	// 차선인식후 조향해야할 값을 반환하는 함수
	// PreCondition 	: topview_transform()함수를 진행한 영상을 inBuf로 넘겨주어야한다.
	// PostCondition	: 화면에 차선인식 및 조향값 도출 결과를 표시한다.
	// Return 			: 조향해야할 값을 (-500 ~ 0 ~ +500)범위에서 출력한다.
	int autoSteering(unsigned char* inBuf, int w, int h, unsigned char* outBuf, int whiteMode);

	// Tunnel
	// 현재 영상의 밝기를 판단하여 터널인지 인식하는 함수
	// PreCondition 	: none
	// PostCondition	: none
	// Return 			: 현재 영상의 밝기값을 판단하여 percent이하의 밝기값이 나온다면 true를 반환한다.
	int Tunnel(unsigned char* inBuf, int w, int h, const double percent);

	/// <summary>
	/// check where is the end line in front of the car
	/// need get img that is already topview transformed
	/// </summary>
	/// <param name="inBuf"></param>
	/// <param name="w"></param>
	/// <param name="h"></param>
	/// <returns></returns>
	int checkFront(unsigned char* inBuf, int w, int h, unsigned char* outBuf);


	/// <summary>
	/// check where is obstacle
	/// </summary>
	/// <param name="src"></param> //Input img buf(������ �̹����� ������ ��)
	/// <returns></returns> true (left), false(right)
	bool checkObstacle(unsigned char* inBuf, int w, int h, unsigned char* outBuf);

	// PreCondition 	: cSettingStatic()함수가 정상 실행 되어야한다.
	// PostCondition	: inBuf의 영상을 (.jpg)파일로 저장한다.
	// Return 			: none
	void opencv_imwrite(unsigned char* inBuf);

	// PreCondition 	: none
	// PostCondition	: inBuf의 영상을 한 프레임씩 저장한다
	//						frame per second = 10
	// Return 			: none
	void opencv_videowrite(unsigned char* inBuf);

	// PreCondition 	: opencv_videowrite()함수로 영상을 저장시켜야한다.
	// PostCondition	: opencv_videowrite()로 저장시킨 영상들을 (.mp4)파일로 최종저장한다.
	// Return 			: none
	void opencv_videoclose(void);

	// PreCondition 	: none
	// PostCondition	: none
	// Return 			: 전방에 흰색 정지선이 있는지의 유무만 판단하여 정지선이 있다면 true를 반환한다.
	bool checkWhiteLine(unsigned char* inBuf, int w, int h);

	// PreCondition 	: checkWhiteLine 함수가 true를 반환한 이후여야한다.
	// PostCondition	: none
	// Return 			: 전방 흰색 정지선과의 거리를 측정하여 반환한다(cm)
	int stopLine_distance(unsigned char* inBuf, int w, int h, float* map1, float* map2);

#ifdef __cplusplus
}
#endif


#endif //IMGPROCESS_H_

