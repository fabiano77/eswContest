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

#endif //IMGPROCESS_H_

