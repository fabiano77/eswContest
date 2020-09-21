#include <pthread.h>
#include "control_mission.h"
#ifndef VPEIMG
#define VPEIMG
#define VPE_OUTPUT_W 640
#define VPE_OUTPUT_H 360
#define VPE_OUTPUT_IMG_SIZE (VPE_OUTPUT_W * VPE_OUTPUT_H * 3)
#endif

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

void startFunc(void *arg);

void flyoverFunc(void *arg);

void priorityFunc(void *arg);

void parkingFunc(void *arg);

void roundaboutFunc(void *arg);

void tunnelFunc(void *arg);

void overtakeFunc(void *arg);

void signalLightFunc(void *arg);

void finishFunc(void *arg);
