#ifndef IMGPROCESS_H_
#define IMGPROCESS_H_ 

#ifdef __cplusplus
extern "C" {
#endif

	void OpenCV_calibration(float* map1, float* map2, int w, int h);

	void OpenCV_remap(unsigned char* inBuf, int w, int h, unsigned char* outBuf, float* map1, float* map2);

	void OpenCV_topview_transform(unsigned char* inBuf, int w, int h, unsigned char* outBuf, int mode);

	void displayPrint(unsigned char* inBuf, int w, int h, unsigned char* outBuf, char* name);

	void debugFiltering(unsigned char* inBuf, int w, int h, unsigned char* outBuf);

	int autoSteering(unsigned char* inBuf, int w, int h, unsigned char* outBuf, int whiteMode);

	int Tunnel(unsigned char* inBuf, int w, int h, const double percent);

	/// <summary>
    /// check where is obstacle
	/// </summary>
	/// <param name="src"></param> //Input img buf(������ �̹����� ������ ��)
	/// <returns></returns> true (left), false(right)
	bool checkObstacle(unsigned char* inBuf,int w, int h, unsigned char* outBuf);
#ifdef __cplusplus
}
#endif


#endif //IMGPROCESS_H_

