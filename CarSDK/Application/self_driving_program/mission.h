#include <pthread.h>
#include "control_mission.h"
#ifndef VPEIMG
#define VPEIMG
#define VPE_OUTPUT_W 640                                      //image output width
#define VPE_OUTPUT_H 360                                      //image output height
#define VPE_OUTPUT_IMG_SIZE (VPE_OUTPUT_W * VPE_OUTPUT_H * 3) //image size*depth
#endif

#define BASIC_SPEED 75      // driving default speed
#define BUZZER_PULSE 100000 // buzzer pulse period
struct thr_data
{
    struct display *disp;
    struct v4l2 *v4l2;
    struct vpe *vpe;
    struct buffer **input_bufs;
    //double anti_overflow[1];
    struct ControlData controlData;
    struct ImgProcessData imgData;
    struct MissionData missionData;

    int msgq_id;

    bool bfull_screen;
    bool bstream_start;
    pthread_t threads[3];
};

// startFunc
// 시작 여부를 판단하는 함수
// PreCondition 	: image available
// PostCondition	: Start Mission is done and flyoverFunc will be activated.
//                    thr_data->missionData.ms[0]=DONE
// Return 			: none
void startFunc(struct thr_data *arg);

// flyoverFunc
// 고가도로 미션을 진행하고 탈출여부를 판단하는 함수
// PreCondition 	: image available
// PostCondition	: Flyover Mission is done. thr_data->missionData.ms[1]=DONE
//                    thr_data->missionData.ms[i]=READY (2<=i<8)
// Return 			: none
void flyoverFunc(struct thr_data *arg);

// priorityFunc
// 우선정지 미션을 진행하고 탈출여부를 판단하는 함수
// PreCondition 	: image available, flyover mission is done.
// PostCondition	: Priority Mission is done. thr_data->missionData.ms[2]=DONE
// Return 			: If priority mission is done, return true. Else return false.
bool priorityFunc(struct thr_data *arg);

// parkingFunc
// 주차미션을 진행하고 탈출여부를 판단하는 함수
// PreCondition 	: image available
// PostCondition	: If first parking is done thr_data->missionData.ms[3]=REMAIN
//               	  If second parking is done thr_data->missionData.ms[3]=DONE
// Return 			: If second parking is done, return true. Else return false.
bool parkingFunc(struct thr_data *arg);

// roundAboutFunc
// 회전교차로 미션을 진행하고 탈출여부를 판단하는 함수
// PreCondition 	: image available
// PostCondition	: Round About Mission is done.
//                    thr_data->missionData.ms[4]=DONE
// Return 			: If roundAbout mission is done, return true. Else return false.
bool roundaboutFunc(struct thr_data *arg);

// tunnelFunc
// 터널 미션을 진행하고 탈출여부를 판단하는 함수
// PreCondition 	: image available
// PostCondition	: Tunnel Mission is done.
//                    thr_data->missionData.ms[5]=DONE
// Return 			: If tunnel mission is done, return true. Else return false.
bool tunnelFunc(struct thr_data *arg);

// overtakeFunc
// 추월 미션 여부 판단 및 진행 후, 탈출여부를 판단하는 함수
// PreCondition 	: image available
// PostCondition	: Overtake Mission is done.
//                    thr_data->missionData.ms[6]=DONE
// Return 			: If tunnel mission is done, return true. Else return false.
bool overtakeFunc(struct thr_data *arg);

// signalLightFunc
// 신호등을 감별하여 미션을 진행하고, 탈출여부를 판단하는 함수
// PreCondition 	: image available
// PostCondition	: Signal Light Mission is done.
//                    thr_data->missionData.ms[7]=DONE
//                    thr_data->missionData.ms[8]=READY (finishFunc is activated.)
// Return 			: If tunnel mission is done, return true. Else return false.
bool signalLightFunc(struct thr_data *arg);

// finishFunc
// 최종 End Line에서 차량이 멈추게하는 함수
// PreCondition 	: image available
// PostCondition	: Finish Mission is done. Autodriving will be done in a sec.
//                    thr_data->missionData.ms[8]=DONE
// Return 			: none
void finishFunc(struct thr_data *arg);

// DesireDistance
// 원하는 속도, 주행각도로 주어진 각도로 운전
// PreCondition 	: SettingSpeed!=0, 1000 < Steering < 2000
// PostCondition	: Car will stop when driving distance is more far than SettingDistance.
// Return 			: none
void DesireDistance(int SettingSpeed, int SettingDistance, int SettingSteering);

// SteeringServo_Write
// 원하는 주행각도로 운전하게 만드는 함수
// PreCondition 	: 1000<angle<2000
// PostCondition	: driving angle is changed that you want to handle.
// Return 			: none
void SteeringServo_Write(signed short angle);

// SteeringServo_Write
// parkingFunc반복하는 함수
// PreCondition 	: debug
// PostCondition	: driving angle is changed that you want to handle.
// Return 			: none
void repeatParking(struct thr_data *arg);