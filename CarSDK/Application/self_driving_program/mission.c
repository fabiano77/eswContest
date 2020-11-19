#include <stdio.h>
#include <signal.h>
#include <pthread.h>
#include <sys/ipc.h>
#include <sys/msg.h>
#include <sys/time.h>
#include <termios.h>
#include <errno.h>
#include <syslog.h>
#include <math.h>
#include "util.h"
#include "v4l2.h"
#include "display-kms.h"
#include "vpe-common.h"
#include "input_cmd.h"
#include "drawing.h"
#include "exam_cv.h"
#include "car_lib.h"
#include "mission.h"
#include "imgProcess.h"
#include "control_mission.h"

struct thr_data *ptr_data;
// ���� structure??? control_mission.h ??? ?????????. 9/8(???)
void startFunc(struct thr_data *arg)
{
    struct thr_data *data = (struct thr_data *)arg;
    usleep(100000);
    DesireSpeed_Write_uart(0);
    usleep(100000);
    data->imgData.bmission = true;
    data->imgData.bprintString = true;

    enum StartState state = WAIT_S;
    sprintf(data->imgData.missionString, "start - Wait");


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
    data->imgData.bmission = false;
    data->imgData.bprintString = false;
    data->imgData.bauto = true;
    data->imgData.bspeedControl = true;
    DesireSpeed_Write_uart(BASIC_SPEED);
}

void flyoverFunc(struct thr_data *arg)
{
    struct thr_data *data = (struct thr_data *)arg;
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
    //data->imgData.bmission = false;
    data->imgData.bprintString = false;
}

bool priorityFunc(struct thr_data *arg)
{
    struct thr_data *data = (struct thr_data *)arg;
    data->imgData.bcheckPriority = true;

    if (data->missionData.frame_priority >= 4)
    {
        data->imgData.bskip = true;
        data->imgData.bauto = false;
        DesireSpeed_Write_uart(0);
        Winker_Write(ALL_ON);
        while (data->imgData.bcheckPriority)
        {
            if (data->missionData.frame_priority == 0)
            {
                data->imgData.bcheckPriority = false;
            }
            usleep(100000);
        }
        usleep(1000000);
        Winker_Write(ALL_OFF);
        DesireSpeed_Write_uart(BASIC_SPEED);
        data->imgData.bauto = true;
        data->imgData.bskip = false;
        return true;
    }
    else
        return false;
}

bool parkingFunc(struct thr_data *arg)
{
    struct thr_data *data = (struct thr_data *)arg;

    if (1) //(data->controlData.steerVal <= 1600 && data->controlData.steerVal >= 1400) || parking == REMAIN)
    {      // 바퀴가 전진방향을 유지하고 있는 경우에만 주차 분기로 진입하도록 하는 장치 (결선 주행용)
        if (DistanceSensor_cm(2) <= 28)
        {
            struct timeval st_p, et_p;
            gettimeofday(&st_p, NULL);

            data->imgData.bprintString = true;
            sprintf(data->imgData.missionString, "Parking");
            int parking_width = 0;
            bool wrong_detection = 1;
            int encoderVal = 0;

            enum ParkingState state = FIRST_WALL;
            enum HorizontalStep step_h = FIRST_BACKWARD;
            enum VerticalStep step_v = FIRST_BACKWARD_V;

            while (state && wrong_detection)
            {
                data->missionData.parkingData.frontRight = (DistanceSensor_cm(2) <= 28) ? true : false;
                data->missionData.parkingData.rearRight = (DistanceSensor_cm(3) <= 28) ? true : false;

                switch (state)
                {
                case FIRST_WALL:
                    sprintf(data->imgData.missionString, "First Wall");
                    if (data->missionData.parkingData.frontRight == false)
                    {
                        state = DISTANCE_CHECK;
                        EncoderCounter_Write(0);
                    }
                    break;

                case DISTANCE_CHECK:
                    if (data->missionData.frame_priority)
                    {
                        wrong_detection = 0;
                        break;
                    }
                    encoderVal = Encoder_Read();
                    if (encoderVal != 65278)
                    {
                        parking_width = encoderVal;
                        sprintf(data->imgData.missionString, "parking_width : %d", parking_width);
                        if (parking_width >= 2000)
                        {
                            wrong_detection = 0;
                            break;
                        }
                    }
                    if (data->missionData.parkingData.frontRight == true)
                    {
                        printf("Result Width : %-3d\n", parking_width);
                        parking_width <= 850 ? data->missionData.parkingData.verticalFlag = true : data->missionData.parkingData.horizontalFlag = true;
                        state = SECOND_WALL;
                    }
                    break;

                case SECOND_WALL:
                    sprintf(data->imgData.missionString, "Second Wall");
                    if (data->missionData.parkingData.rearRight == true)
                    {
                        state = PARKING_START;

                        data->imgData.bmission = true;
                    }
                    break;

                case PARKING_START:
                    sprintf(data->imgData.missionString, "Parking Start");
                    if (data->missionData.parkingData.verticalFlag && data->missionData.parkingData.horizontalFlag == false)
                    {
                        DesireDistance(60, 200, 1500);
                        while (data->missionData.parkingData.verticalFlag)
                        {
                            switch (step_v)
                            {
                            case FIRST_BACKWARD_V:
                                sprintf(data->imgData.missionString, "FIRST_BACKWARD_V");
                                SteeringServo_Write(1050);
                                usleep(80000);
                                DesireSpeed_Write_uart(-60);
                                while (1)
                                {
                                    if (DistanceSensor_cm(3) <= 13 && DistanceSensor_cm(5) <= 13)
                                    {
                                        DesireSpeed_Write_uart(0);
                                        step_v = SECOND_BACKWARD_V;
                                        break;
                                    }
                                    usleep(5000);
                                }
                                break;

                            case SECOND_BACKWARD_V:
                                sprintf(data->imgData.missionString, "SECOND_BACKWARD_V");
                                SteeringServo_Write(1500);
                                usleep(70000);
                                DesireSpeed_Write_uart(-35);
                                if (DistanceSensor_cm(2) <= 11)
                                {
                                    DesireSpeed_Write_uart(0);
                                    step_v = RIGHT_FRONT_V;
                                    break;
                                }
                                else if (DistanceSensor_cm(6) <= 11)
                                {
                                    DesireSpeed_Write_uart(0);
                                    step_v = LEFT_FRONT_V;
                                    break;
                                }
                                if (DistanceSensor_cm(5) <= 4)
                                {
                                    printf("under steer\n");
                                    DesireSpeed_Write_uart(0);
                                    step_v = UNDER_STEER_V;
                                    break;
                                }
                                else if (DistanceSensor_cm(3) <= 4)
                                {
                                    printf("over steer\n");
                                    DesireSpeed_Write_uart(0);
                                    step_v = OVER_STEER_V;
                                    break;
                                }
                                break;

                            case UNDER_STEER_V:
                                sprintf(data->imgData.missionString, "UNDER_STEER");
                                DesireDistance(60, 100, 1300);
                                DesireDistance(60, 100, 1500);
                                DesireDistance(60, 200, 1750);
                                step_v = SECOND_BACKWARD_V;
                                break;

                            case OVER_STEER_V:
                                sprintf(data->imgData.missionString, "OVER_STEER");
                                DesireDistance(60, 100, 1700);
                                DesireDistance(60, 100, 1500);
                                DesireDistance(60, 200, 1250);
                                step_v = SECOND_BACKWARD_V;
                                break;

                            case RIGHT_FRONT_V:
                                sprintf(data->imgData.missionString, "RIGHT_FRONT_V");
                                int right_difference;
                                right_difference = DistanceSensor_cm(2) - DistanceSensor_cm(3);
                                if (abs(right_difference) < 3)
                                {
                                    DesireSpeed_Write_uart(0);
                                    step_v = FIRST_FORWARD_V;
                                    break;
                                }
                                DesireDistance(60, 100, 1500 - (right_difference * 80));
                                DesireDistance(-30, 400, 1500);
                                break;

                            case LEFT_FRONT_V:
                                sprintf(data->imgData.missionString, "RIGHT_FRONT_V");
                                int left_difference;
                                left_difference = DistanceSensor_cm(6) - DistanceSensor_cm(5);
                                if (abs(left_difference) < 3)
                                {
                                    DesireSpeed_Write_uart(0);
                                    step_v = FIRST_FORWARD_V;
                                    break;
                                }
                                DesireDistance(60, 100, 1500 + (left_difference * 80));
                                DesireDistance(-30, 400, 1500);
                                break;

                            case FIRST_FORWARD_V:
                                sprintf(data->imgData.missionString, "FIRST_FORWARD_V");
                                DesireDistance(-30, 400, 1500);
                                step_v = SECOND_FORWARD_V;
                                Winker_Write(ALL_ON);
                                buzzer(2, 500000, 500000);
                                
                                Winker_Write(ALL_OFF);
                                break;

                            case SECOND_FORWARD_V:
                                sprintf(data->imgData.missionString, "SECOND_FORWARD_V");
                                DesireSpeed_Write_uart(60);
                                if (DistanceSensor_cm(2) >= 20 && DistanceSensor_cm(6) >= 20)
                                {
                                    DesireDistance(60, 200, 1500);

                                    step_v = FINISH_V;
                                }
                                break;

                            case FINISH_V:
                                sprintf(data->imgData.missionString, "FINISH_V");
                                DesireDistance(60, 1000, 1050);
                                data->missionData.parkingData.verticalFlag = 0;
                                break;

                            default:
                                break;
                            }

                            usleep(5000);
                        }
                    }
                    else if (data->missionData.parkingData.verticalFlag == false && data->missionData.parkingData.horizontalFlag)
                    {
                        //DesireDistance(60, 230, 1500);
                        DesireDistance(60, 270, 1500);
                        while (data->missionData.parkingData.horizontalFlag)
                        {
                            switch (step_h)
                            {
                            case FIRST_BACKWARD:
                                sprintf(data->imgData.missionString, "FIRST_BACKWARD");
                                DesireDistance(-60, 800, 1050);
                                DesireDistance(-60, 400, 1500);
                                SteeringServo_Write(2000);
                                usleep(50000);
                                DesireSpeed_Write_uart(-26);

                                while (1)
                                {
                                    if (DistanceSensor_cm(4) <= 6)
                                    {
                                        DesireSpeed_Write_uart(0);
                                        break;
                                    }
                                    usleep(5000);
                                }
                                SteeringServo_Write(1250);
                                usleep(80000);
                                DesireSpeed_Write_uart(40);
                                while (1)
                                {
                                    if ((abs(DistanceSensor_cm(2) - DistanceSensor_cm(3)) <= 3) || DistanceSensor_cm(1) <= 4)
                                    {
                                        DesireSpeed_Write_uart(0);
                                        step_h = SECOND_BACKWARD;
                                        break;
                                    }
                                    usleep(5000);
                                }

                                break;

                            case SECOND_BACKWARD:
                                sprintf(data->imgData.missionString, "SECOND_BACKWARD");
                                int difference;
                                difference = DistanceSensor_cm(2) - DistanceSensor_cm(3);
                                if (difference < -2)
                                {
                                    DesireDistance(-30, 400, 1300);
                                    if (abs(DistanceSensor_cm(2) - DistanceSensor_cm(3)) <= 2)
                                    {
                                        DesireSpeed_Write_uart(0);
                                        usleep(20000);
                                        break;
                                    }
                                    DesireDistance(40, 400, 1700);
                                }
                                else if (difference > 2)
                                {
                                    DesireDistance(-30, 400, 1700);
                                    if (abs(DistanceSensor_cm(2) - DistanceSensor_cm(3)) <= 2)
                                    {
                                        DesireSpeed_Write_uart(0);
                                        usleep(20000);
                                        break;
                                    }
                                    DesireDistance(40, 400, 1300);
                                }
                                if (abs(difference) <= 2)
                                {
                                    DesireSpeed_Write_uart(0);
                                    SteeringServo_Write(1500);
                                    step_h = SECOND_FORWARD;
                                    Winker_Write(ALL_ON);
                                    buzzer(2, 500000, 500000);
                                    Winker_Write(ALL_OFF);
                                }
                                break;

                            case SECOND_FORWARD:
                                sprintf(data->imgData.missionString, "SECOND_FORWARD");

                                if (DistanceSensor_cm(4) <= 5)
                                {
                                    step_h = ESCAPE;
                                    break;
                                }
                                DesireSpeed_Write_uart(-27);

                                while (1)
                                {
                                    if (DistanceSensor_cm(4) <= 5)
                                    {
                                        DesireSpeed_Write_uart(0);
                                        usleep(5000);
                                        step_h = ESCAPE;
                                        break;
                                    }
                                    usleep(5000);
                                }
                                break;

                            case ESCAPE:
                                sprintf(data->imgData.missionString, "ESCAPE");
                                DesiredDistance(50, 250, 2000);
                                step_h = ESCAPE_2;
                                break;

                            case ESCAPE_2:
                                sprintf(data->imgData.missionString, "ESCAPE_2");
                                DesiredDistance(-50, 170, 1500);
                                step_h = ESCAPE_3;
                                break;

                            case ESCAPE_3:
                                sprintf(data->imgData.missionString, "ESCAPE_3");
                                DesireDistance(60, 600, 1900);
                                step_h = FINISH;
                                break;

                            case FINISH:
                                sprintf(data->imgData.missionString, "FINISH");
                                DesireDistance(60, 600, 1100);
                                data->missionData.parkingData.horizontalFlag = 0;
                                break;

                            default:
                                break;
                            }
                            usleep(5000);
                        }
                    }
                    DesireSpeed_Write_uart(35);
                    state = DONE_P;

                    gettimeofday(&et_p, NULL);
                    float parkingTime;
                    parkingTime = (((et_p.tv_sec - st_p.tv_sec) * 1000) + ((int)et_p.tv_usec / 1000 - (int)st_p.tv_usec / 1000)) / 1000.0;
                    printf("parking time : %f\n", parkingTime);
                    break;

                default:
                    break;
                }
                usleep(50000);
            }
            data->imgData.bmission = false;
            data->imgData.bprintString = false;
            return true;
        }
        else
            return false;
    }
}

bool roundaboutFunc(struct thr_data *arg)
{
    struct thr_data *data = (struct thr_data *)arg;

    if (StopLine(4)) //|| data->missionData.finish_distance != -1)	
    {
        DesireSpeed_Write_uart(0);
        sprintf(data->imgData.missionString, "round about");
        printf("roundabout IN\n");

        data->imgData.bwhiteLine = true;
        data->imgData.bprintString = true;
        data->imgData.bspeedControl = false;
       
        int speed = BASIC_SPEED;
        int flag_END = 0;        

        enum RoundaboutState state = WAIT_R;
        while (state != DONE_R)
        {
            //data->missionData.loopTime = timeCheck(&time);
            switch (state)
            {
            case NONE_R:
                break;

            case WAIT_R:
                data->imgData.bmission = true;
                if (RoundAbout_isStart(DistanceSensor_cm(1)))
                {
                    printf("ROUND_GO_1-1\n");
                    sprintf(data->imgData.missionString, "ROUND_GO_1-1");

                    state = ROUND_GO_1;
                }
                break;

            case ROUND_GO_1:
                onlyDistance(speed, 550);
                printf("ROUND_GO_1_2\n");
                sprintf(data->imgData.missionString, "ROUND_GO_1-2");
                usleep(100000);

                data->imgData.bmission = false;
                onlyDistance(speed, 1100);

                printf("ROUND_STOP\n");
                sprintf(data->imgData.missionString, "ROUND_STOP");

                state = ROUND_STOP;
                break;

            case ROUND_STOP:
                if ((DistanceSensor_cm(4) <= 25) /*|| (DistanceSensor_cm(5) <= 6)*/)
                {
                    speed = BASIC_SPEED;
                    DesireSpeed_Write_uart(speed);
                    sprintf(data->imgData.missionString, "ROUND_GO_2");
                    printf("ROUND_GO_2\n");

                    state = ROUND_GO_2;
                }
                break;

            case ROUND_GO_2:
                if ((DistanceSensor_cm(4) <= 8) /* || (DistanceSensor_cm(5) <= 6)*/)
                {
                    printf("speed up \n");
                    if (speed < 70)
                        speed += 5;
                }
                else if ((DistanceSensor_cm(1) <= 8) /* || (DistanceSensor_cm(6) <= 6)*/)
                {
                    DesireSpeed_Write_uart(0);
                    printf("stop and speed down \n");
                    if (speed > 30)
                        speed -= 10;
                    usleep(1900000);
                    break;
                }
                DesireSpeed_Write_uart(speed);
                
                //end
                //if (abs(data->controlData.steerVal - 1500) < 60)
                if (data->controlData.steerVal - 1500 < 30)
                {
                    if (flag_END < 3)
                        flag_END++;

                    if (flag_END == 3)
                    {
                        sprintf(data->imgData.missionString, "DONE_R");
                        printf("DONE_R");

                        state = DONE_R;
                    }
                }
                else
                {
                    if (flag_END > 0)
                        flag_END--;
                }

                break;

            case DONE_R:
                break;
            }
            usleep(100000);
        }

        printf("ROUNDABOUT_OFF\n");
        sprintf(data->imgData.missionString, "ROUNDABOUT_OFF");

        data->missionData.broundabout = false;
        data->imgData.bspeedControl = true;
        data->imgData.bprintString = false;
        data->imgData.bcheckFrontWhite = false;
        data->missionData.finish_distance = -1;
        return true;
    }
    else
        return false;
}

bool tunnelFunc(struct thr_data *arg)
{
    struct thr_data *data = (struct thr_data *)arg;

    data->imgData.bdark = true;
    if (data->missionData.btunnel && DistanceSensor_cm(2) < 20 && DistanceSensor_cm(6) < 20)
    {
        data->imgData.bprintString = true;
        data->imgData.bmission = true;
        data->imgData.bdark = false;

        frontLightOnOff(data->controlData.lightFlag, true);
        sprintf(data->imgData.missionString, "mission thread : tunnel detect");
        int c2 = DistanceSensor_cm(2);
        int c6 = DistanceSensor_cm(6);
        int isEnd = Tunnel_isEnd(c2, c6, 50, 50);
        do
        {
            //data->missionData.loopTime = timeCheck(&time);
            data->controlData.steerVal = Tunnel_SteerVal(c2, c6);
            sprintf(data->imgData.missionString, "steer = %d, %d : %d", data->controlData.steerVal, c2, c6);
            SteeringServo_Write(data->controlData.steerVal);
            usleep(10000);

            c2 = DistanceSensor_cm(2);
            c6 = DistanceSensor_cm(6);
            isEnd = Tunnel_isEnd(c2, c6, 50, 50);            
        } while (!isEnd)
        
        sprintf(data->imgData.missionString, "tunnel out");
        frontLightOnOff(data->controlData.lightFlag, false);

        printf("Tunnel OUT\n");

        data->imgData.bmission = false;
        data->imgData.bprintString = false;
        data->missionData.btunnel = false;
        return true;
    }
    else
        return false;
}

bool overtakeFunc(struct thr_data *arg)
{

    struct thr_data *data = (struct thr_data *)arg;

    /* �б����� ���� ????? */
    int distance_1 = DistanceSensor_cm(1);
    if (1)
    // if (data->controlData.steerVal <= 1600 &&
    //     data->controlData.steerVal >= 1400)
    {

        if (distance_1 <= 20) //????? ???????? ��??? //���� ????????? ?????????, �б����� ?????
        {
            printf("\t distance_1 = %d \n", distance_1);
            data->imgData.bmission = true;  //??????ó�� X
            data->imgData.btopview = false; //topview off
            //data->imgData.bwhiteLine = true; // ?????? ���� O
            data->imgData.bprintString = true;
            sprintf(data->imgData.missionString, "overtake");
            printf("overtake \n");
            enum OvertakeState state = FRONT_DETECT;
            data->missionData.overtakingData.headingDirection = STOP;
            data->missionData.overtakingFlag = true;
            data->imgData.bwhiteLine = true;
            bool obstacle = false;
            int thresDistance = 450;
            /*���� ?????*/
            DesireSpeed_Write_uart(0);

            DesireDistance(-50, 450, 1500);

            while (state)
            {
                //data->missionData.loopTime = timeCheck(&time);
                switch (state)
                {
                case FRONT_DETECT:
                    /* ???????? �¿�????????? ?????? ī��??? �������� */
                    sprintf(data->imgData.missionString, "Front Detect");
                    if (data->missionData.overtakingData.headingDirection == STOP)
                    {
                        data->controlData.cameraY = 1610;
                        CameraYServoControl_Write(data->controlData.cameraY);
                        data->missionData.overtakingData.updownCamera = CAMERA_UP;
                    }
                    /* ???????? �¿� ?????? ?? ���??????? ����????? ???����?????? �ڵ�*/
                    while (data->missionData.overtakingData.headingDirection == STOP)
                    {
                        //data->missionData.loopTime = timeCheck(&time);
                        usleep(50000);
                    }
                    /*?????? ����?? Camera ?????? ???ġ�� ?????*/
                    if (data->missionData.overtakingData.headingDirection != STOP)
                    {
                        data->controlData.cameraY = 1660;
                        CameraYServoControl_Write(data->controlData.cameraY);
                        data->missionData.overtakingData.updownCamera = CAMERA_DOWN;
                        data->imgData.btopview = true; //top view on
                    }
                    else
                    {
                        break;
                    }
                    /*?????? ?????? ?????? ���� ?????*/
                    if (data->missionData.overtakingData.headingDirection == RIGHT &&
                        data->missionData.overtakingData.updownCamera == CAMERA_DOWN)
                    {
                        sprintf(data->imgData.missionString, "Right to go");
                        /*���?*/
                        Winker_Write(RIGHT_ON);
                        //DesireDistance(50, thresDistance, 1100);
                        DesireDistance(50, thresDistance, 1050);
                        DesireDistance(50, 150, 1250);

                        Winker_Write(ALL_OFF);
                        /* ?????? ????? ?????? ��???*/
                        usleep(500000);
                        /*thresDistance?????? ????? ????? �Ÿ� ?????????*/
                        if (DistanceSensor_cm(1) < 20)
                        {
                            sprintf(data->imgData.missionString, "Detect Error");
                            /*?????, ????? ?? ���� ??????*/
                            DesireDistance(-50, thresDistance, 1100);
                            /*????? ?? ���� ?????? ����*/
                            data->missionData.overtakingData.headingDirection = LEFT;
                        }
                        else
                        { /*????? ��Ž??*/
                            state = SIDE_ON;
                            sprintf(data->imgData.missionString, "Detect Side");
                            /*???����??? ?????? ????? ???????? 20 ?????? ????????? SIDE_ON????? ����*/
                            DesireSpeed_Write_uart(BASIC_SPEED);
                            usleep(500000);
                        }
                    }
                    else if (data->missionData.overtakingData.headingDirection == LEFT &&
                             data->missionData.overtakingData.updownCamera == CAMERA_DOWN)
                    {

                        sprintf(data->imgData.missionString, "Left to go");
                        /*���?*/
                        Winker_Write(LEFT_ON);
                        //DesireDistance(50, thresDistance, 1900);
                        DesireDistance(50, thresDistance, 1950);
                        DesireDistance(50, 150, 1750);
                        Winker_Write(ALL_OFF);
                        /* ?????? ????? ?????? ��???*/
                        usleep(500000);
                        /*thresDistance?????? ????? ????? �Ÿ� ?????????*/
                        if (DistanceSensor_cm(1) < 20)
                        {
                            /*?????, ????? ?? ���� ??????*/
                            sprintf(data->imgData.missionString, "Detect Error");
                            DesireDistance(-50, thresDistance, 1900);
                            /*????? ??? ���� ?????? ����*/
                            data->missionData.overtakingData.headingDirection = RIGHT;
                        }
                        else
                        {
                            /*???����??? ?????? ????? ???????? 20 ?????? ????????? SIDE_ON????? ����*/
                            state = SIDE_ON;
                            sprintf(data->imgData.missionString, "Detect Side");

                            DesireSpeed_Write_uart(BASIC_SPEED);
                            usleep(500000);
                        }
                    }
                    else
                    { /*STOP??? ??????????? ���? ����*/
                    }

                    break;

                case SIDE_ON:
                    /*Auto Steering ??????*/
                    data->imgData.bmission = false;
                    /* ?????? ??????���� ?????????????????? ?????? side ??????(2,3 or 4,5)?? ��????????? �ڵ�*/
                    //right
                    if (data->missionData.overtakingData.headingDirection == RIGHT)
                    {
                        /*???????? ????? ??????*/
                        int distance_5 = DistanceSensor_cm(5);
                        int distance_6 = DistanceSensor_cm(6);

                        if (distance_5 <= 30 || distance_6 <= 30)
                        {
                            obstacle = true;
                        }
                        else if (obstacle == true)
                        {
                            /*???????? ?????*/
                            if (distance_5 > 30 && distance_6 > 30)
                            {
                                data->imgData.bmission = true; //Auto Steering off
                                usleep(100000);
                                DesireSpeed_Write_uart(0);
                                //usleep(50000);
                                obstacle = false;
                                state = SIDE_OFF;
                                sprintf(data->imgData.missionString, "Side OFF");
                            }
                        }
                        usleep(50000);
                    }
                    //left
                    else if (data->missionData.overtakingData.headingDirection == LEFT)
                    {
                        /*???????? ????? ??????*/
                        int distance_3 = DistanceSensor_cm(3);
                        int distance_2 = DistanceSensor_cm(2);
                        if (distance_3 <= 30 || distance_2 <= 30)
                        {
                            obstacle = true;
                        }
                        else if (obstacle == true)
                        {
                            /*???????? ?????*/
                            if (distance_3 > 30 && distance_2 > 30)
                            {
                                data->imgData.bmission = true; //Auto Steering off
                                usleep(100000);
                                DesireSpeed_Write_uart(0);
                                //usleep(50000);
                                obstacle = false;
                                state = SIDE_OFF;
                                sprintf(data->imgData.missionString, "Side OFF");
                            }
                        }
                        usleep(50000);
                    }
                    //error and go back step
                    else
                    {
                        state = FRONT_DETECT;
                    }
                    break;

                case SIDE_OFF:
                    /*?????? ����????? ��????????? �ڵ�*/
                    usleep(10000);
                    data->imgData.bmission = true; //Auto Steering off
                    usleep(10000);
                    //right
                    if (data->missionData.overtakingData.headingDirection == RIGHT)
                    {
                        /*��??? ��ȸ??? ���� ?????? ?? ?????*/
                        Winker_Write(LEFT_ON);
                        DesireDistance(50, thresDistance + 350, 2000);
                        Winker_Write(ALL_OFF);
                        DesireDistance(50, 600, 1000);
                    }
                    //left
                    else if (data->missionData.overtakingData.headingDirection == LEFT)
                    {
                        /*��??? ????????? ���� ??????*/
                        Winker_Write(RIGHT_ON);
                        DesireDistance(50, thresDistance + 350, 1000);
                        Winker_Write(ALL_OFF);
                        DesireDistance(50, 600, 2000);
                    }
                    /*???����?? ?????*/
                    data->imgData.bmission = false;
                    //sprintf(data->imgData.missionString, "End Overtaking");
                    data->imgData.bprintString = false;
                    DesireSpeed_Write_uart(BASIC_SPEED);
                    state = DONE_O;
                    data->missionData.overtakingFlag = false;
                    return true;
                    break;

                default:
                    break;
                }
                //usleep(1500000);
                usleep(50000); // 1,500 ms -> 50ms ?? ????, 09/01 AM 00:50 -KDH
            }
            data->imgData.bmission = false;
            data->imgData.bprintString = false;
        }
        else
            return false;
    }
    else
    {
        return false;
    }
}

bool signalLightFunc(struct thr_data *arg)
{
    struct thr_data *data = (struct thr_data *)arg;

    //data->imgData.bcheckFrontWhite = true; // ????? ???????? ??????????? ????? ON

    if (StopLine(4)) // data->missionData.finish_distance != -1) //??????????? ??????????? �Ÿ�?? ����?????
    {
        // onlyDistance(BASIC_SPEED, (data->missionData.finish_distance / 26.0) * 500); //????????��??? ????? stop
        // data->missionData.finish_distance = -1;

        data->imgData.bmission = true;
        data->imgData.bcheckSignalLight = true;
        data->imgData.bprintString = true;
        data->imgData.bprintTire = false;
        data->missionData.signalLightData.state = DETECT_RED;
        sprintf(data->imgData.missionString, "check RED");
        DesireSpeed_Write_uart(0);
        SteeringServo_Write(1500);
        usleep(50000);
        printf("signalLight\n");

        while (data->imgData.bcheckSignalLight)
            usleep(200000); //??????ó��?????? ????????? ����??? ?????? ??? ��??? ��ٸ���?.

        data->imgData.bprintTire = true;
        sprintf(data->imgData.missionString, "Distance control");
        DesireSpeed_Write_uart(BASIC_SPEED);

        bool once_back = false;
        int front_distance = DistanceSensor_cm(1);
        while (1) // ????????? ��������??? �Ÿ�?? 23,24cm ?? ����?? ?????? ?????
        {
            front_distance = DistanceSensor_cm(1);
         
            if (front_distance < 23)
            {
                once_back = true;
                DesireSpeed_Write_uart(-20);
            }
            else if (front_distance <= 24)
            {
                DesireSpeed_Write_uart(0);
                break;
            }
            else if (once_back && front_distance > 24)
                DesireSpeed_Write_uart(20);
            usleep(50000);
        }

        switch (data->missionData.signalLightData.finalDirection)
        {
        case 1:
            sprintf(data->imgData.missionString, "Turn right");
            printf("\tTurn right\n");
            DesireDistance(40, 1170, 1000);
            break;
        case -1:
            sprintf(data->imgData.missionString, "Turn left");
            printf("\tTurn left\n");
            DesireDistance(40, 1170, 2000);
            break;
        default:
            sprintf(data->imgData.missionString, "ERROR");
            printf("\tERROR\n");
            DesireDistance(40, 1150, 1000);
            break;
        }

        data->imgData.bmission = false;
        data->imgData.bprintString = false;
        return true;
    }
    else
        return false;
}

void finishFunc(struct thr_data *arg)
{
    struct thr_data *data = (struct thr_data *)arg;

    if (1) 
    {
        sprintf(data->imgData.missionString, "Finish line check");
        DesireSpeed_Write_uart(0);
        SteeringServo_Write(1500);
        data->imgData.bmission = true;
        data->imgData.bprintString = true;
        data->imgData.bcheckFinishLine = true;

        DesireSpeed_Write_uart(30);
        while (data->missionData.finish_distance == -1)
        {
            usleep(5000); //5ms
        }
        DesireSpeed_Write_uart(0);
        data->imgData.bcheckFinishLine = false;

        int rest_distance = data->missionData.finish_distance;
        rest_distance -= 6;
        sprintf(data->imgData.missionString, "Finish Driving");
        DesireDistance(40, 500 * (rest_distance / 26.0), 1500); // encoder = 500 -> 26cm??

        printf("finish end\n");
        sprintf(data->imgData.missionString, "All mission complete !");
        buzzer(3, 500000, 500000);

        data->imgData.bauto = false;
        data->imgData.bmission = false;
        data->imgData.bprintString = false;
    }
}

void DesireDistance(int SettingSpeed, int SettingDistance, int SettingSteering)
{
    ptr_data->controlData.steerVal = SettingSteering;

    if (SettingSpeed < 0)
        rearLightOnOff(ptr_data->controlData.lightFlag, 1);
    DesiredDistance(SettingSpeed, SettingDistance, SettingSteering);
    if (SettingSpeed < 0)
        rearLightOnOff(ptr_data->controlData.lightFlag, 0);
}

void SteeringServo_Write(signed short angle)
{
    ptr_data->controlData.steerVal = angle;
    SteeringServo_Write_uart(angle);
}
