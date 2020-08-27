#ifndef IMGPROCESS_H_
#define IMGPROCESS_H_ 

#ifdef __cplusplus
extern "C" {
#endif

	void cSettingStatic(int w, int h);

	void calibration(float* map1, float* map2, int w, int h);

	void OpenCV_remap(unsigned char* inBuf, int w, int h, unsigned char* outBuf, float* map1, float* map2);

	void topview_transform(unsigned char* inBuf, int w, int h, unsigned char* outBuf, int mode);

	void displayPrintStr(unsigned char* outBuf, int w, int h, char* name);

	void displayPrintMission(unsigned char* outBuf, int w, int h, 
		int ms0, int ms1, int ms2, int ms3, int ms4, int ms5, int ms6, int ms7, int ms8);

	void displayPrintSensor(unsigned char* outBuf, int w, int h,
		int c1, int c2, int c3, int c4, int c5, int c6, int stopline);

	int checkRed(unsigned char* inBuf, int w, int h, unsigned char* outBuf);

	int checkYellow(unsigned char* inBuf, int w, int h, unsigned char* outBuf);

	int checkGreen(unsigned char* inBuf, int w, int h, unsigned char* outBuf);

	bool isPriorityStop(unsigned char* inBuf, int w, int h, unsigned char* outBuf);

	void debugFiltering(unsigned char* inBuf, int w, int h, unsigned char* outBuf, int mode);

	int autoSteering(unsigned char* inBuf, int w, int h, unsigned char* outBuf, int whiteMode);

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

	void opencv_imwrite(unsigned char* inBuf);

	void opencv_videowrite(unsigned char* inBuf);

	void opencv_videoclose(void);


#ifdef __cplusplus
}
#endif


#endif //IMGPROCESS_H_

