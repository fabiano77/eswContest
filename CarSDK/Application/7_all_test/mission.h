#include <pthread.h>
#include "control_mission.h"
#ifndef VPEIMG
#define VPEIMG
#define VPE_OUTPUT_W 640
#define VPE_OUTPUT_H 360
#define VPE_OUTPUT_IMG_SIZE (VPE_OUTPUT_W * VPE_OUTPUT_H * 3)
#endif

#define BASIC_SPEED 55      // ????ес???? ?? ??????????, autoSteer??? ??? ??????.
#define BUZZER_PULSE 100000 // ?? ????? ????
struct thr_data
{
    struct display *disp;
    struct v4l2 *v4l2;
    struct vpe *vpe;
    struct buffer **input_bufs;
    struct ControlData controlData;
    struct MissionData missionData;
    struct ImgProcessData imgData;
    unsigned char img_data_buf[VPE_OUTPUT_IMG_SIZE];

    int msgq_id;

    bool bfull_screen;
    bool bstream_start;
    pthread_t threads[4];
};

void startFunc(struct thr_data *arg);

void flyoverFunc(struct thr_data *arg);

void priorityFunc(struct thr_data *arg);

void parkingFunc(struct thr_data *arg);

void roundaboutFunc(struct thr_data *arg);

void tunnelFunc(struct thr_data *arg);

void overtakeFunc(struct thr_data *arg);

void signalLightFunc(struct thr_data *arg);

void finishFunc(struct thr_data *arg);

void DesireDistance(int SettingSpeed, int SettingDistance, int SettingSteering);

void SteeringServo_Write(signed short angle);