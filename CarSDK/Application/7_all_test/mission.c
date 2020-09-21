#include "mission.h"

void start(void *arg)
{
    struct thr_data *data = (struct thr_data *)arg;
    DesireSpeed_Write(0);
    data->missionData.ms[0] = start;
    data->imgData.bmission = true;
    data->imgData.bprintString = true;
    sprintf(data->imgData.missionString, "start - Wait");

    enum StartState state = WAIT_S;

    while (state)
    {
        switch (state)
        {
        case WAIT_S:
            if (DistanceSensor_cm(1) < 8)
            {
                sprintf(data->imgData.missionString, "start - front_close");
                state = FRONT_CLOSE_S;
            }
            break;

        case FRONT_CLOSE_S:
            if (DistanceSensor_cm(1) > 8)
            {
                sprintf(data->imgData.missionString, "start - front_open");
                state = FRONT_OPEN_S;
            }
            break;

        case FRONT_OPEN_S:
            state = START_S;
            buzzer(1, 0, BUZZER_PULSE);
            break;

        case START_S:
            break;
        }
        usleep(10000);
    }

    start = DONE;
    flyover = READY;
    data->missionData.ms[0] = start;
    //data->missionData.ms[1] = flyover;
    data->imgData.bmission = false;
    data->imgData.bprintString = false;
    data->imgData.bauto = true;
    data->imgData.bspeedControl = true;
}

void flyoverFunc(void *arg)
{
    struct thr_data *data = (struct thr_data *)arg;
    //오른쪽 거리센서가 안잡히면 탈출하는 것으로 예상.
    //data->imgData.bmission = true;
    int escapeCnt = 4;
    data->imgData.bprintString = true;
    sprintf(data->imgData.missionString, "flyover");

    while (escapeCnt)
    {
        if (DistanceSensor_cm(2) > 25 && DistanceSensor_cm(6) > 25)
            escapeCnt--;
        usleep(50000);
    }
    buzzer(1, 0, BUZZER_PULSE);
    flyover = DONE;
    //data->imgData.bmission = false;
    data->imgData.bprintString = false;
}

void priorityFunc(void *arg)
{
    struct thr_data *data = (struct thr_data *)arg;
}

void parkingFunc(void *arg)
{

    struct thr_data *data = (struct thr_data *)arg;
}

void roundaboutFunc(void *arg)
{
    struct thr_data *data = (struct thr_data *)arg;
}

void tunnelFunc(void *arg)
{
    struct thr_data *data = (struct thr_data *)arg;
}

void overtakeFunc(void *arg)
{
    struct thr_data *data = (struct thr_data *)arg;
}

void signalLightFunc(void *arg)
{
    struct thr_data *data = (struct thr_data *)arg;
}

void finishFunc(void *arg)
{
    struct thr_data *data = (struct thr_data *)arg;
}