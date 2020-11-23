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
#include "imgProcess.h"
#include "control_mission.h"
#include "mission.h"

#define CAPTURE_IMG_W 1280
#define CAPTURE_IMG_H 720
#define CAPTURE_IMG_SIZE (CAPTURE_IMG_W * CAPTURE_IMG_H * 2) // YUYU : 16bpp
#define CAPTURE_IMG_FORMAT "uyvy"

//해상도를 바꾸려면 이부분만 변경하면 됨
#ifndef VPEIMG
#define VPEIMG
#define VPE_OUTPUT_W 640
#define VPE_OUTPUT_H 360

#define VPE_OUTPUT_IMG_SIZE (VPE_OUTPUT_W * VPE_OUTPUT_H * 3)
#endif
#define VPE_OUTPUT_FORMAT "bgr24"

#define OVERLAY_DISP_FORCC FOURCC('A', 'R', '2', '4')
#define OVERLAY_DISP_W 480
#define OVERLAY_DISP_H 272

#define TIME_TEXT_X 340			   //320
#define TIME_TEXT_Y 260			   //240
#define TIME_TEXT_COLOR 0xffffffff //while

#define FPS_TEXT_X 40			  //320
#define FPS_TEXT_Y 260			  //240
#define FPS_TEXT_COLOR 0xffffffff //while

//#define BASIC_SPEED 65		// 프로그램 기본 주행 속도 => mission.h로 옮김
#define BUZZER_PULSE 100000 // 기본 부저 길이
#define GO_LEFT 0
#define GO_RIGHT 1

// thr_data의 정의와 각종 structure들은 control_mission.h 으로 옮김 9/8(대희)
extern struct thr_data *ptr_data;
/******************** function ********************/
static int allocate_input_buffers(struct thr_data *data)
{
	int i;
	struct vpe *vpe = data->vpe;

	data->input_bufs = calloc(NUMBUF, sizeof(*data->input_bufs));
	for (i = 0; i < NUMBUF; i++)
	{
		data->input_bufs[i] = alloc_buffer(vpe->disp, vpe->src.fourcc, vpe->src.width, vpe->src.height, false);
	}
	if (!data->input_bufs)
		ERROR("allocating shared buffer failed\n");

	for (i = 0; i < NUMBUF; i++)
	{
		/** Get DMABUF fd for corresponding buffer object */
		vpe->input_buf_dmafd[i] = omap_bo_dmabuf(data->input_bufs[i]->bo[0]);
		data->input_bufs[i]->fd[0] = vpe->input_buf_dmafd[i];
	}
	return 0;
}

static void free_input_buffers(struct buffer **buffer, uint32_t n, bool bmultiplanar)
{
	uint32_t i;
	for (i = 0; i < n; i++)
	{
		if (buffer[i])
		{
			close(buffer[i]->fd[0]);
			omap_bo_del(buffer[i]->bo[0]);
			if (bmultiplanar)
			{
				close(buffer[i]->fd[1]);
				omap_bo_del(buffer[i]->bo[1]);
			}
		}
	}
	free(buffer);
}

static void draw_operatingtime(struct display *disp, uint32_t time, uint32_t itime)
{
	FrameBuffer tmpFrame;
	unsigned char *pbuf[4];
	char strtime[128];
	char stritime[128];

	memset(strtime, 0, sizeof(strtime));
	memset(stritime, 0, sizeof(stritime));

	sprintf(strtime, "optime : %03d(ms)", time);
	sprintf(stritime, "i thrd : %03d(ms)", itime);

	if (get_framebuf(disp->overlay_p_bo, pbuf) == 0)
	{
		tmpFrame.buf = pbuf[0];
		tmpFrame.format = draw_get_pixel_foramt(disp->overlay_p_bo->fourcc); //FORMAT_RGB888; //alloc_overlay_plane() -- FOURCC('R','G','2','4');
		tmpFrame.stride = disp->overlay_p_bo->pitches[0];					 //tmpFrame.width*3;

		drawString(&tmpFrame, strtime, TIME_TEXT_X, TIME_TEXT_Y - 20, 0, TIME_TEXT_COLOR);
		drawString(&tmpFrame, stritime, TIME_TEXT_X, TIME_TEXT_Y, 0, TIME_TEXT_COLOR);
	}
}

/************************************************/
/*	Function - img_process						*/
/************************************************/
static void img_process(struct display *disp, struct buffer *cambuf, struct thr_data *t_data, float *map1, float *map2)
{
	unsigned char srcbuf[VPE_OUTPUT_W * VPE_OUTPUT_H * 3];
	uint32_t optime;
	struct timeval st, et;
	struct timeval time;
	unsigned char *cam_pbuf[4];
	bool delay_flag = false;
	bool checking_stopline = false;

	if (get_framebuf(cambuf, cam_pbuf) == 0)
	{
		memcpy(srcbuf, cam_pbuf[0], VPE_OUTPUT_W * VPE_OUTPUT_H * 3);

		gettimeofday(&st, NULL);
		timeCheck(&time);
		/********************************************************/
		/*			우리가 만든 알고리즘 함수를 넣는 부분		*/
		/********************************************************/
		//printf("\nstart1 \t\t: %d\n", timeCheck(&time));
		/* 라인 필터링이나 canny 결과 확인 */
		if (t_data->imgData.bdebug)
		{
			if (t_data->imgData.bfilteringTest)
			{
				filteringTest(srcbuf, VPE_OUTPUT_W, VPE_OUTPUT_H, srcbuf, t_data->imgData.filtering_param.h,
							  t_data->imgData.filtering_param.s,
							  t_data->imgData.filtering_param.v,
							  t_data->imgData.filtering_param.canny1,
							  t_data->imgData.filtering_param.canny2);
			}
			else
			{
				if (t_data->imgData.debugMode == 9)
					OpenCV_remap(srcbuf, VPE_OUTPUT_W, VPE_OUTPUT_H, srcbuf, map1, map2);
				debugFiltering(srcbuf, VPE_OUTPUT_W, VPE_OUTPUT_H, srcbuf, t_data->imgData.debugMode);
				//printf("debug \t\t: %d\n", timeCheck(&time));
			}
		}

		/* 미션 진행중에 처리하는 영상처리 */
		else if (t_data->imgData.bmission)
		{

			t_data->controlData.beforeSpeedVal = 0;

			/* 추월차로시에 사용 */
			if (t_data->missionData.overtakingFlag &&
				t_data->missionData.overtakingData.updownCamera == CAMERA_UP)
			{
				usleep(100000);
				/*check�?? ?��?�� camera up*/
				bool check_direction;
				check_direction = checkObstacle(srcbuf, VPE_OUTPUT_W, VPE_OUTPUT_H, srcbuf, t_data->imgData.bdebug);
				/*오른쪽인지 왼쪽인지 매 번 확인*/
				if (check_direction == true)
				{ //true=>left
					t_data->missionData.overtakingData.leftFlag++;
				}
				else
				{ //false =>right
					t_data->missionData.overtakingData.rightFlag++;
				}

				/*5회 판단 이후 확인*/
				if ((t_data->missionData.overtakingData.rightFlag + t_data->missionData.overtakingData.leftFlag) >= 5)
				{
					/*오른쪽 flag가 큰경우*/
					if (t_data->missionData.overtakingData.rightFlag > t_data->missionData.overtakingData.leftFlag)
					{
						t_data->missionData.overtakingData.headingDirection = RIGHT;
					}
					/*왼쪽 flag가 큰경우*/
					else
					{
						t_data->missionData.overtakingData.headingDirection = LEFT;
					}
					/*상황 재진입 막기 위한 Camera Down*/
					t_data->missionData.overtakingData.updownCamera = CAMERA_DOWN;
				}
				//srcbuf를 활용하여 capture한 영상을 변환
			}

			/* 신호등 확인 */
			if (t_data->imgData.bcheckSignalLight)
			{
				switch (t_data->missionData.signalLightData.state)
				{
				case DETECT_RED:
					if (checkRed(srcbuf, VPE_OUTPUT_W, VPE_OUTPUT_H, srcbuf))
					{
						sprintf(t_data->imgData.missionString, "check YELLOW");
						buzzer(1, 0, BUZZER_PULSE);
						delay_flag = true;
						t_data->missionData.signalLightData.state = DETECT_YELLOW;
					}
					break;

				case DETECT_YELLOW:
					if (checkYellow(srcbuf, VPE_OUTPUT_W, VPE_OUTPUT_H, srcbuf))
					{
						sprintf(t_data->imgData.missionString, "check GREEN");
						buzzer(1, 0, BUZZER_PULSE);
						delay_flag = true;
						t_data->missionData.signalLightData.state = DETECT_GREEN;
						t_data->missionData.signalLightData.Accumulation_greenVal = 0;
						t_data->missionData.signalLightData.ignore_frame = 3;
					}
					break;

				case DETECT_GREEN:
					if (t_data->missionData.signalLightData.ignore_frame)
					{
						if (checkGreen(srcbuf, VPE_OUTPUT_W, VPE_OUTPUT_H, srcbuf))
							t_data->missionData.signalLightData.ignore_frame--;
					}
					else
					{
						t_data->missionData.signalLightData.Accumulation_greenVal += checkGreen(srcbuf, VPE_OUTPUT_W, VPE_OUTPUT_H, srcbuf);
						if (t_data->missionData.signalLightData.Accumulation_greenVal >= 3)
						{
							buzzer(2, BUZZER_PULSE, BUZZER_PULSE);
							delay_flag = true;
							t_data->missionData.signalLightData.state = DETECTION_FINISH;
							t_data->missionData.signalLightData.finalDirection = 1;
							t_data->imgData.bcheckSignalLight = false;
						}
						else if (t_data->missionData.signalLightData.Accumulation_greenVal <= -3)
						{
							buzzer(1, 0, BUZZER_PULSE * 2);
							delay_flag = true;
							t_data->missionData.signalLightData.state = DETECTION_FINISH;
							t_data->missionData.signalLightData.finalDirection = -1;
							t_data->imgData.bcheckSignalLight = false;
						}
					}
					break;

				case DETECTION_FINISH:
					break;
				}
			}

			/* 피니시 라인과의 거리 측정 */
			if (t_data->imgData.bcheckFinishLine)
			{
				OpenCV_remap(srcbuf, VPE_OUTPUT_W, VPE_OUTPUT_H, srcbuf, map1, map2);

				topview_transform(srcbuf, VPE_OUTPUT_W, VPE_OUTPUT_H, srcbuf, 1);

				t_data->missionData.finish_distance = calculDistance_FinishLine(srcbuf, VPE_OUTPUT_W, VPE_OUTPUT_H, srcbuf);
				if (t_data->missionData.finish_distance != -1)
				{
					t_data->imgData.bcheckFinishLine = false;
				}
			}

			/*끝날 때 사용*/
			if (t_data->missionData.finishData.checkFront == true)
			{
				t_data->imgData.topMode = 1; //앞이 더 잘보이는 mode 1사용
				topview_transform(srcbuf, VPE_OUTPUT_W, VPE_OUTPUT_H, srcbuf, t_data->imgData.topMode);
				t_data->missionData.finishData.distEndLine = checkFront(srcbuf, VPE_OUTPUT_W, VPE_OUTPUT_H, srcbuf);
				/*무의미한 값인 경우 알고리즘에 맞게 steering 진행*/
				if (t_data->missionData.finishData.distEndLine == -1000)
				{
					int steerVal = autoSteering(srcbuf, VPE_OUTPUT_W, VPE_OUTPUT_H, srcbuf, t_data->imgData.bwhiteLine);
					if (steerVal != 9999)
					{
						t_data->controlData.steerVal = 1500 - steerVal;
						SteeringServoControl_Write(t_data->controlData.steerVal);

						t_data->controlData.desireSpeedVal = auto_speedMapping(steerVal, 40);
					}
					if (t_data->controlData.desireSpeedVal != t_data->controlData.beforeSpeedVal)
					{
						//이전 속도와 달라졌을 때만 속도값 인가.
						DesireSpeed_Write_uart(t_data->controlData.desireSpeedVal);
						t_data->controlData.beforeSpeedVal = t_data->controlData.desireSpeedVal;
					}
				}
				else if (t_data->missionData.finishData.distEndLine < 320)
				{ /*거리가 40(360-40)이하로 탐지된 경우 영상처리 종료*/
					t_data->missionData.finishData.checkFront = false;
				}
			}
		}

		/* 기본 상태에서 처리되는 영상처리 */
		else
		{

			if (t_data->imgData.bcheckPriority)
			{
				if (isPriorityStop(srcbuf, VPE_OUTPUT_W, VPE_OUTPUT_H, srcbuf))
				{
					if (t_data->missionData.frame_priority < 4)
						t_data->missionData.frame_priority++;
					printf("img thread : isPriorityStop() return 1; frame =%d\n", t_data->missionData.frame_priority);
				}
				else
				{
					if (t_data->missionData.frame_priority > 0)
						t_data->missionData.frame_priority--;
				}
			}

			if (t_data->imgData.bcheckFrontWhite && t_data->missionData.finish_distance == -1)
			{
				checking_stopline = checkWhiteLine(srcbuf, VPE_OUTPUT_W, VPE_OUTPUT_H);

				if (checking_stopline)
				{
					t_data->missionData.finish_distance = stopLine_distance(srcbuf, VPE_OUTPUT_W, VPE_OUTPUT_H, map1, map2);

					if (t_data->missionData.finish_distance != -1)
					{
						printf("stopLine distance %d\n", t_data->missionData.finish_distance);
					}
				}
			}

			if (t_data->imgData.bdark)
			{
				if (Tunnel(srcbuf, VPE_OUTPUT_W, VPE_OUTPUT_H, 40))
				{
					printf("img thread : dark\n");
					t_data->missionData.btunnel = true;
				}
				else
				{
					t_data->missionData.btunnel = false;
				}
			}

			if (t_data->imgData.bcalibration)
			{
				OpenCV_remap(srcbuf, VPE_OUTPUT_W, VPE_OUTPUT_H, srcbuf, map1, map2);
			}

			if (t_data->imgData.btopview && t_data->imgData.bskip == false)
			{
				topview_transform(srcbuf, VPE_OUTPUT_W, VPE_OUTPUT_H, srcbuf, t_data->imgData.topMode);
			}

			printf("before auto \t: %d\n", timeCheck(&time));
			if (t_data->imgData.bauto)
			{
				int steerVal = autoSteering(srcbuf, VPE_OUTPUT_W, VPE_OUTPUT_H, srcbuf, t_data->imgData.bwhiteLine);
				if (steerVal != 9999)
				{
					t_data->controlData.steerVal = 1500 - steerVal;
					SteeringServo_Write_uart(t_data->controlData.steerVal);
				}
				if (t_data->imgData.bspeedControl)
				{
					t_data->controlData.desireSpeedVal = auto_speedMapping(steerVal, BASIC_SPEED);
					if (t_data->controlData.desireSpeedVal != t_data->controlData.beforeSpeedVal)
					{
						//이전 속도와 달라졌을 때만 속도값 인가.
						DesireSpeed_Write_uart(t_data->controlData.desireSpeedVal);
						t_data->controlData.beforeSpeedVal = t_data->controlData.desireSpeedVal;
					}
				}
			}
			else
			{
				t_data->controlData.beforeSpeedVal = 0;
			}
			printf("after  auto \t: %d\n", timeCheck(&time));

			/*checkWhiteLineFlag�?? True?�� 경우, RoundAbout */
		}

		/********************************************************/
		/*			영상처리 종료								*/
		/********************************************************/

		/* 영상처리후 오버레이로 정보 등등 출력.*/
		// if (checking_stopline)
		// {
		// 	checking_stopline = false;
		// 	displayPrintStopLine(srcbuf, VPE_OUTPUT_W, VPE_OUTPUT_H);
		// }
		if (t_data->imgData.bprintString)
		{
			displayPrintStr(srcbuf, VPE_OUTPUT_W, VPE_OUTPUT_H, t_data->imgData.missionString);
		}
		// if (t_data->imgData.bprintMission)
		// {
		// 	displayPrintMission(srcbuf, VPE_OUTPUT_W, VPE_OUTPUT_H,
		// 						(int)t_data->missionData.ms[0], (int)t_data->missionData.ms[1], (int)t_data->missionData.ms[2],
		// 						(int)t_data->missionData.ms[3], (int)t_data->missionData.ms[4], (int)t_data->missionData.ms[5],
		// 						(int)t_data->missionData.ms[6], (int)t_data->missionData.ms[7], (int)t_data->missionData.ms[8]);
		// }
		// if (t_data->imgData.bprintSensor)
		// {
		// 	displayPrintSensor(srcbuf, VPE_OUTPUT_W, VPE_OUTPUT_H,
		// 					   DistanceSensor_cm(1), DistanceSensor_cm(2), DistanceSensor_cm(3),
		// 					   DistanceSensor_cm(4), DistanceSensor_cm(5), DistanceSensor_cm(6), StopLine(4));
		// }
		// if (t_data->imgData.bprintTire)
		// {
		// 	overlayPrintAngle(srcbuf, VPE_OUTPUT_W, VPE_OUTPUT_H, srcbuf, t_data->controlData.steerVal);
		// }
		// printf("overlay \t: %d\n", timeCheck(&time));
		/*		현재 영상 .jpg파일로 저장		*/
		// if (t_data->imgData.dump_request)
		// {
		// 	opencv_imwrite(srcbuf);
		// 	t_data->imgData.dump_request = false;
		// }
		/*		현재 영상 .avi파일로 녹화		*/
		// if (t_data->imgData.bvideoRecord)
		// {
		// 	memcpy(t_data->img_data_buf, srcbuf, VPE_OUTPUT_IMG_SIZE);
		// }

		/* 신호등 검출화면을 유지하기 위한 delay */
		if (delay_flag)
		{
			usleep(1700000); // 1700ms == 1.7s
			delay_flag = false;
		}

		memcpy(cam_pbuf[0], srcbuf, VPE_OUTPUT_W * VPE_OUTPUT_H * 3);
		gettimeofday(&et, NULL);
		optime = ((et.tv_sec - st.tv_sec) * 1000) + ((int)et.tv_usec / 1000 - (int)st.tv_usec / 1000);
		//printf("img_process() \t: %d\n", optime);
		draw_operatingtime(disp, optime, t_data->imgData.loopTime);
	}
}

/************************************************/
/*	Thread - image_process_thread				*/
/************************************************/
void *image_process_thread(void *arg)
{
	struct thr_data *data = (struct thr_data *)arg;
	struct v4l2 *v4l2 = data->v4l2;
	struct vpe *vpe = data->vpe;
	struct buffer *capt;
	struct timeval st, et;
	float map1[VPE_OUTPUT_W * VPE_OUTPUT_H] = {
		0,
	};
	float map2[VPE_OUTPUT_W * VPE_OUTPUT_H] = {
		0,
	};
	int index;
	int i;
	bool isFirst = true;

	memset(map1, 0, VPE_OUTPUT_W * VPE_OUTPUT_H);
	memset(map2, 0, VPE_OUTPUT_W * VPE_OUTPUT_H);
	calibration(map1, map2, VPE_OUTPUT_W, VPE_OUTPUT_H);
	v4l2_reqbufs(v4l2, NUMBUF);
	vpe_input_init(vpe);
	allocate_input_buffers(data);
	if (vpe->dst.coplanar)
		vpe->disp->multiplanar = true;
	else
		vpe->disp->multiplanar = false;
	printf("disp multiplanar:%d \n", vpe->disp->multiplanar);
	vpe_output_init(vpe);
	vpe_output_fullscreen(vpe, data->bfull_screen);
	for (i = 0; i < NUMBUF; i++)
		v4l2_qbuf(v4l2, vpe->input_buf_dmafd[i], i);
	for (i = 0; i < NUMBUF; i++)
		vpe_output_qbuf(vpe, i);
	v4l2_streamon(v4l2);
	vpe_stream_on(vpe->fd, V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE);
	vpe->field = V4L2_FIELD_ANY;
	data->imgData.loopTime = 0;

	while (1)
	{
		gettimeofday(&st, NULL);
		index = v4l2_dqbuf(v4l2, &vpe->field);
		vpe_input_qbuf(vpe, index);
		if (isFirst)
		{
			vpe_stream_on(vpe->fd, V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE);
			isFirst = false;
			MSG("streaming started...");
			data->bstream_start = true;
		}
		index = vpe_output_dqbuf(vpe);
		capt = vpe->disp_bufs[index];
		/********************************************************/
		/* 영상처리 시작										*/
		/********************************************************/
		img_process(vpe->disp, capt, data, map1, map2);

		/********************************************************/
		/* 영상처리 종료										*/
		/********************************************************/

		if (disp_post_vid_buffer(vpe->disp, capt, 0, 0, vpe->dst.width, vpe->dst.height))
		{
			ERROR("Post buffer failed");
			return NULL;
		}
		update_overlay_disp(vpe->disp);
		vpe_output_qbuf(vpe, index);
		index = vpe_input_dqbuf(vpe);
		v4l2_qbuf(v4l2, vpe->input_buf_dmafd[index], index);
		gettimeofday(&et, NULL);
		data->imgData.loopTime = ((et.tv_sec - st.tv_sec) * 1000) + ((int)et.tv_usec / 1000 - (int)st.tv_usec / 1000);
		//printf("img_thread() \t: %d\n\n", data->imgData.loopTime);
	}

	MSG("Ok!");
	return NULL;
}

/************************************************/
/*	Thread - input_thread						*/
/************************************************/
void *input_thread(void *arg)
{
	struct thr_data *data = (struct thr_data *)arg;

	int total_encoder = 0;
	int forward_encoder = 0;
	char cmd_input[128];
	char cmd_ready = true;

	while (!data->bstream_start)
	{
		usleep(100 * 1000);
	}

	printf("---------------------[control key]------------------	\n");
	printf("    w      : forward          |   i      : up			\n");
	printf("  a s d    : left, stop ,right| 						\n");
	printf("    x      : backward         |   k      : down			\n");
	printf("  (move)                      							\n");
	printf("----------------------------------------------------	\n");
	printf("	1 = speed up, 2 = speed down						\n");
	printf("----------------------------------------------------	\n");
	printf("	q, e = winker || z, c = front, rear lamp			\n");
	printf("----------------------------------------------------	\n");
	printf("	space bar = Alarm									\n");
	printf("----------------------------------------------------	\n");
	//MSG("\t dist   : distance sensor check");
	//MSG("\t distc  : distance sensor check by -cm-");
	MSG("\n\nInput command:");
	MSG("\t calib  : calibration ON/OFF");
	MSG("\t debug  : debug ON/OFF");
	MSG("\t auto   : auto steering ON/OFF");
	MSG("\t top    : top view ON/OFF");
	MSG("\t encoder: ");
	MSG("\t back   : ");
	MSG("\t stop   : check line sensor");
	MSG("\t white  : white line ON/OFF");
	MSG("\t mission: mission display output on/off");
	MSG("\t sensor : sensor  display output on/off");
	MSG("\t tire   : tire    display output on/off");
	MSG("\t ms	   : mission on/off");
	MSG("\t dump   : save image file");
	MSG("\t filter : filtering test");
	MSG("\n");

	int buzzerPulseWidth_us = 100000;

	while (1)
	{
		if (cmd_ready == true)
		{
			printf("\tinput : ");
			/*standby to input command */
			cmd_ready = StandbyInput(cmd_input); //define in cmd.cpp
		}
		else
		{
			if (strlen(cmd_input) == 1)
			{
				manualControl(&(data->controlData), cmd_input[0]);
			}
			else if (0 == strncmp(cmd_input, "calib", 5))
			{
				data->imgData.bcalibration = !data->imgData.bcalibration;
				buzzer(1, 0, buzzerPulseWidth_us);
				if (data->imgData.bcalibration)
					printf("\t calibration ON\n");
				else
					printf("\t calibration OFF\n");
			}
			else if (0 == strncmp(cmd_input, "debug", 5))
			{
				if (!data->imgData.bdebug)
				{
					printf("\t select debug mode\n");
					printf("1. lineFiltering() \n");
					printf("2. lineFiltering() & cannyEdge() \n");
					printf("3. checkObstacle() \n");
					printf("4. checkRedSignal() \n");
					printf("5. checkYellowSignal() \n");
					printf("6. checkGreenSignal() \n");
					printf("7. priorityStop() \n");
					printf("8. checkFront() \n");
					printf("9. calculDistance_toFinish() \n");
					printf("10. isDark() \n\n");

					printf("\t input(0~10) : ");
					scanf("%d", &data->imgData.debugMode);
					buzzer(1, 0, buzzerPulseWidth_us);
					data->imgData.bdebug = !data->imgData.bdebug;
					if (data->imgData.debugMode == 3)
						CameraYServoControl_Write(1610);
					printf("\t debug ON\n");
				}
				else
				{
					data->imgData.bdebug = !data->imgData.bdebug;
					CameraYServoControl_Write(1660);
					printf("\t debug OFF\n");
				}
			}
			else if (0 == strncmp(cmd_input, "filter", 6))
			{
				buzzer(1, 0, buzzerPulseWidth_us);
				data->imgData.bdebug = true;
				data->imgData.bfilteringTest = true;
				
				printf("\t filtering Test ON\n");

				printf("h_min, s_max, v_min = %d, %d, %d\ncanny = %d, %d\n", data->imgData.filtering_param.h,
					   data->imgData.filtering_param.s,
					   data->imgData.filtering_param.v,
					   data->imgData.filtering_param.canny1,
					   data->imgData.filtering_param.canny2);

				printf("\tfilteringTest() : func(line, canny)\t= ");
				char func[10]; 
				scanf("%s", func);

				while (1)
				{
					int number;
					char hsvMode[10];
					int canny1, canny2;
					if (cmd_ready == true)
					{
						if (0 == strncmp(func, "line", 4))
						{
							printf("\tfilteringTest() : line(h,s,v)\t: ");
							scanf("%s", hsvMode); //define in cmd.cpp
							printf("\tfilteringTest() : %s(number)\t: ", cmd_input);
							scanf("%d", &number);
						}
						else if (0 == strncmp(func, "canny", 5))
						{
							printf("\tfilteringTest() : canny1\t: ");
							scanf("%d", &canny1);
							printf("\tfilteringTest() : canny2\t: ");
							scanf("%d", &canny2);
						}
					}
					else
					{
						buzzer(1, 0, buzzerPulseWidth_us);
						if (0 == strncmp(func, "line", 4))
						{
							if (0 == strncmp(cmd_input, "h", 1))
							{
								data->imgData.filtering_param.h = number;
							}
							else if (0 == strncmp(cmd_input, "s", 1))
							{
								data->imgData.filtering_param.s = number;
							}
							else if (0 == strncmp(cmd_input, "v", 1))
							{
								data->imgData.filtering_param.v = number;
							}
						}
						else if (0 == strncmp(func, "canny", 5))
						{
							data->imgData.filtering_param.canny1 = canny1;
							data->imgData.filtering_param.canny2 = canny2;
						}
						
						printf("\nh_min, s_max, v_min = %d, %d, %d\n\tcanny = %d, %d\n\n",data->imgData.filtering_param.h,
																		data->imgData.filtering_param.s,
																		data->imgData.filtering_param.v,
																		data->imgData.filtering_param.canny1,
																		data->imgData.filtering_param.canny2);

						cmd_ready = true;
					}
				}`
			}
			else if (0 == strncmp(cmd_input, "auto", 4))
			{
				buzzer(1, 0, buzzerPulseWidth_us);
				data->imgData.bauto = !data->imgData.bauto;
				if (data->imgData.bauto)
					printf("\t auto steering ON\n");
				else
					printf("\t auto steering OFF\n");
			}
			else if (0 == strncmp(cmd_input, "top", 3))
			{
				buzzer(1, 0, buzzerPulseWidth_us);
				if (!data->imgData.btopview)
				{
					data->imgData.btopview = !data->imgData.btopview;
					printf("\t topview 1 ON\n");
				}
				else
				{
					if (data->imgData.topMode == 1)
					{
						data->imgData.topMode = 2;
						printf("\t topview 2 ON\n");
					}
					else if (data->imgData.topMode == 2)
					{
						data->imgData.topMode = 3;
						printf("\t topview 3 ON\n");
					}
					else if (data->imgData.topMode == 3)
					{
						data->imgData.topMode = 1;
						data->imgData.btopview = !data->imgData.btopview;
						printf("\t topview OFF\n");
					}
				}
			}
			else if (0 == strncmp(cmd_input, "distc", 5))
			{
				int d_data;
				int d_data_cm;
				int channel;
				int j;
				printf("channel(1~6) : ");
				scanf("%d", &channel);
				for (j = 0; j < 70; j++)
				{
					d_data_cm = DistanceSensor_cm(channel);
					d_data = DistanceSensor(channel);
					printf("channel = %d, distance = %d[cm], %d \n", channel, d_data_cm, d_data);
					usleep(300000);
				}
			}
			else if (0 == strncmp(cmd_input, "dist", 4))
			{
				int d_data;
				int channel;
				int j;
				printf("channel(1~6) : ");
				scanf("%d", &channel);
				for (j = 0; j < 70; j++)
				{
					d_data = DistanceSensor(channel);
					printf("channel = %d, distance = 0x%04X(%d) \n", channel, d_data, d_data);
					usleep(100000);
				}
			}
			else if (0 == strncmp(cmd_input, "encoder", 7))
			{
				int init_encoder = 0;
				int desire_encoder = 0;
				int on_encoder = 0;
				printf("Disired Speed : ");
				scanf("%d", &desire_encoder);
				buzzer(1, 0, buzzerPulseWidth_us);
				EncoderCounter_Write(init_encoder);
				DesireSpeed_Write(BASIC_SPEED);
				while (1)
				{
					on_encoder = Encoder_Read();
					//if (on_encoder != 65278)
					//	printf("encoder : %-3d\n", on_encoder);
					if (on_encoder >= desire_encoder && on_encoder != 65278)
					{
						DesireSpeed_Write(0);
						printf("encoder : %d, total : %d\n", on_encoder, forward_encoder);
						forward_encoder += on_encoder;
						break;
					}
					usleep(10000);
				}
				printf("Total Forward encoder : %d\n", forward_encoder);
			}
			else if (0 == strncmp(cmd_input, "back", 4))
			{
				int init_encoder = 0;
				int desire_encoder = 0;
				int on_encoder = 0;
				printf("Disired Encoder : ");
				scanf("%d", &desire_encoder);
				buzzer(1, 0, buzzerPulseWidth_us);
				EncoderCounter_Write(init_encoder);
				DesireSpeed_Write(-BASIC_SPEED);
				while (1)
				{
					on_encoder = abs(Encoder_Read());

					if (on_encoder != 65278)
						printf("encoder : %-3d\n", on_encoder);
					//if (on_encoder != 65278)
					//	printf("encoder : %-3d\n", on_encoder);
					if (on_encoder >= desire_encoder && on_encoder != 65278)
					{
						DesireSpeed_Write(0);
						printf("encoder : %d, total : %d\n", on_encoder, total_encoder);
						total_encoder += on_encoder;
						break;
					}
					usleep(10000);
				}
				printf("Total Back encoder : %d\n", total_encoder);
			}
			else if (0 == strncmp(cmd_input, "mission", 7))
			{
				buzzer(1, 0, buzzerPulseWidth_us);
				data->imgData.bprintMission = !data->imgData.bprintMission;
				if (data->imgData.bprintMission)
					printf("\t print mission ON\n");
				else
					printf("\t print mission OFF\n");
			}
			else if (0 == strncmp(cmd_input, "white", 5))
			{
				buzzer(1, 0, buzzerPulseWidth_us);
				data->imgData.bwhiteLine = !data->imgData.bwhiteLine;
				if (data->imgData.bprintSensor)

					printf("\t whiteLine ON\n");
				else
					printf("\t whiteLine OFF\n");
			}
			else if (0 == strncmp(cmd_input, "sensor", 6))
			{
				buzzer(1, 0, buzzerPulseWidth_us);
				data->imgData.bprintSensor = !data->imgData.bprintSensor;
				if (data->imgData.bprintSensor)
					printf("\t print sensor ON\n");
				else
					printf("\t print sensor OFF\n");
			}
			else if (0 == strncmp(cmd_input, "tire", 4))
			{
				buzzer(1, 0, buzzerPulseWidth_us);
				data->imgData.bprintTire = !data->imgData.bprintTire;
				if (data->imgData.bprintTire)
					printf("\t print tire ON\n");
				else
					printf("\t print tire OFF\n");
			}
			else if (0 == strncmp(cmd_input, "ms", 2))
			{
				int num;
				printf("0. start \n");
				printf("1. flyover \n");
				printf("2. priority \n");
				printf("3. parking \n");
				printf("4. tunnel \n");
				printf("5. round about \n");
				printf("6. overtake \n");
				printf("7. signal light \n");
				printf("8. finish \n\n");

				printf("\t input(0~7) : ");
				scanf("%d", &num);
				buzzer(1, 0, buzzerPulseWidth_us);
				if (num >= 0 && num <= 8)
				{
					if (num == 4)
					{
						data->imgData.bdark = true;
						printf("bdark ON \n");
					}
					if (data->missionData.ms[num] == READY)
						data->missionData.ms[num] = NONE;
					else
						data->missionData.ms[num] = READY;
					data->missionData.changeMissionState = true;
				}
				else
				{
					printf("wrong input\n");
				}
			}
			else if (0 == strncmp(cmd_input, "dump", 4))
			{
				buzzer(1, 0, buzzerPulseWidth_us);
				data->imgData.dump_request = true;
			}
			else if (0 == strncmp(cmd_input, "video", 5))
			{
				buzzer(1, 0, buzzerPulseWidth_us);
				data->imgData.bvideoRecord = true;
			}
			else if (0 == strncmp(cmd_input, "save", 3))
			{
				buzzer(1, 0, buzzerPulseWidth_us);
				data->imgData.bvideoRecord = false;
				data->imgData.bvideoSave = true;
			}
			else
			{
				printf("cmd_input:%s \n", cmd_input);
			}
			cmd_ready = true;
		}
	}

	return NULL;
}

// void *video_record_thread(void *arg)
// {
// 	struct thr_data *data = (struct thr_data *)arg;
// 	struct timeval st, et;
// 	bool videoEnd = false;
// 	int fps = 10;
// 	int delay_ms = 1000 / fps;
// 	int current_frame = 0;
// 	int recordTime_ms;
// 	while (!data->imgData.bvideoRecord)
// 	{
// 		usleep(500000);
// 	}
// 	printf("\tvideo_record_thred() : ON\n");
// 	// st체크를 이곳에서 하고, et체크를 두 번째 while에 하여 매 프레임마다 delay_ms만큼 누산시키어
// 	// 프레임이 찍히는 시점과 영상에서의 해당 시점을 근사하도록 한다. (0.1초 절대값 캡처 대신 영상의 전체길이의 비율로)
// 	gettimeofday(&st, NULL);
// 	while (1)
// 	{
// 		current_frame++;
// 		printf("video in\n");
// 		if (videoEnd)
// 		{
// 			usleep(10000000);
// 		}
// 		else if (data->imgData.bvideoSave)
// 		{
// 			opencv_videoclose();
// 			videoEnd = true;
// 		}
// 		else if (data->imgData.bvideoRecord)
// 		{
// 			opencv_videowrite(data->img_data_buf);
// 		}
// 		while (1) //영상 녹화 싱크를 맞춰주기 위한 delay
// 		{
// 			gettimeofday(&et, NULL);
// 			recordTime_ms = ((et.tv_sec - st.tv_sec) * 1000) + ((int)et.tv_usec / 1000 - (int)st.tv_usec / 1000);
// 			if (recordTime_ms > (delay_ms * current_frame))
// 			{
// 				break;
// 			}
// 			else
// 			{
// 				usleep(5000); // 5ms delay
// 			}
// 		}
// 	}
// }

/************************************************/
/*	Thread - mission_thread						*/
/************************************************/
void *mission_thread(void *arg)
{
	struct thr_data *data = (struct thr_data *)arg;
	enum MissionState start = READY;
	enum MissionState flyover = NONE;
	enum MissionState priority = READY;
	enum MissionState parking = READY;
	enum MissionState roundabout = NONE;
	enum MissionState tunnel = READY;
	enum MissionState overtake = NONE;
	enum MissionState signalLight = NONE;
	enum MissionState finish = NONE;
	data->missionData.ms[0] = start;
	data->missionData.ms[1] = flyover;
	data->missionData.ms[2] = priority;
	data->missionData.ms[3] = parking;
	data->missionData.ms[4] = tunnel;
	data->missionData.ms[5] = roundabout;
	data->missionData.ms[6] = overtake;
	data->missionData.ms[7] = signalLight;
	data->missionData.ms[8] = finish;
	//각 미션이 수행되고나면 detect를 하지 않도록 변수설정.

	while (1)
	{
		if (start && start != DONE)
		{
			data->missionData.ms[0] = start;
			startFunc(data);
			start = DONE;
			flyover = READY;
			data->missionData.ms[0] = start;
			data->missionData.ms[1] = flyover;
		}

		if (flyover && flyover != DONE)
		{
			flyoverFunc(data);
			flyover = DONE;
			// priority 'Ready'
		}

		if (priority && priority != DONE)
		{
			if (priorityFunc(data))
			{
				priority = DONE;
				parking = READY;
				roundabout = READY;
			}
			// parking 켜기
		}

		if (parking && parking != DONE)

		{
			if (parkingFunc(data))
			{
				if (parking == READY)
				{
					parking = REMAIN;
					printf("First Parking is Done!\n");
				}
				else if (parking == REMAIN)
				{
					parking = DONE;
					printf("Second Parking is Done!\n");
					roundabout = READY;
				}
			}
		}

		if (roundabout && roundabout != DONE && priority == DONE)
		{
			if (roundaboutFunc(data))
			{
				roundabout = DONE;
				tunnel = READY;
			}
		}

		if (tunnel && tunnel != DONE)
		{
			if (tunnelFunc(data))
			{
				tunnel = DONE;
				overtake = READY;
			}
		}

		if (roundabout && roundabout != DONE)
		{
			if (roundaboutFunc(data))
			{
				roundabout = DONE;
				tunnel = READY;
			}
		}

		if (overtake && overtake != DONE)
		{
			int distance_1 = DistanceSensor_cm(1);
			data->imgData.bwhiteLine = true;
			if (distance_1 <= 30)
			{
				data->imgData.bmission = true;
				data->imgData.btopview = false; //topview off

				data->imgData.bprintString = true;
				sprintf(data->imgData.missionString, "overtake");
				printf("overtake \n");
				enum OvertakeState state = FRONT_DETECT;
				data->missionData.overtakingData.headingDirection = STOP;
				data->missionData.overtakingFlag = true;
				data->imgData.bwhiteLine = true;
				bool obstacle = false;
				int thresDistance = 450;
				usleep(10000);
				DesireSpeed_Write_uart(0);
				usleep(300000);
				int distance_2;
				int distance_3;
				int distance_5;
				int distance_6;
				while (state)
				{
					switch (state)
					{
					case FRONT_DETECT:
						sprintf(data->imgData.missionString, "Front Detect");
						if (data->missionData.overtakingData.headingDirection == STOP)
						{
							data->controlData.cameraY = 1610;
							CameraYServoControl_Write(data->controlData.cameraY);
							usleep(300000);
							data->missionData.overtakingData.updownCamera = CAMERA_UP;
						}
						while (data->missionData.overtakingData.headingDirection == STOP)
						{
							usleep(50000);
						}
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
						if (data->missionData.overtakingData.headingDirection == RIGHT &&
							data->missionData.overtakingData.updownCamera == CAMERA_DOWN)
						{
							sprintf(data->imgData.missionString, "Right to go");
							Winker_Write(RIGHT_ON);
							laneChange(GO_RIGHT, BASIC_SPEED, 0);

							Winker_Write(ALL_OFF);
							usleep(400000);
							if (DistanceSensor_cm(1) < 20)
							{
								sprintf(data->imgData.missionString, "Detect Error");
								DesireDistance(-BASIC_SPEED, thresDistance, 1100);
								data->missionData.overtakingData.headingDirection = LEFT;
							}
							else
							{
								state = SIDE_ON;
								sprintf(data->imgData.missionString, "Detect Side");
								data->imgData.bmission = false;
								DesireSpeed_Write_uart(BASIC_SPEED);
								usleep(500000);
							}
						}
						else if (data->missionData.overtakingData.headingDirection == LEFT &&
								 data->missionData.overtakingData.updownCamera == CAMERA_DOWN)
						{

							sprintf(data->imgData.missionString, "Left to go");
							Winker_Write(LEFT_ON);
							laneChange(GO_LEFT, BASIC_SPEED, 0);

							Winker_Write(ALL_OFF);
							usleep(400000);
							if (DistanceSensor_cm(1) < 20)
							{
								sprintf(data->imgData.missionString, "Detect Error");
								DesireDistance(-BASIC_SPEED, thresDistance, 1900);
								data->missionData.overtakingData.headingDirection = RIGHT;
							}
							else
							{
								state = SIDE_ON;
								sprintf(data->imgData.missionString, "Detect Side");
								data->imgData.bmission = false;
								DesireSpeed_Write_uart(BASIC_SPEED);
								usleep(500000);
							}
						}
						else
						{
						}

						break;

					case SIDE_ON:

						// right
						switch (data->missionData.overtakingData.headingDirection)
						{
						case RIGHT:
							distance_5 = DistanceSensor_cm(5);
							distance_6 = DistanceSensor_cm(6);

							if (distance_5 <= 30 || distance_6 <= 30)
							{
								obstacle = true;
							}
							else if (obstacle == true && distance_5 > 30 && distance_6 > 30)
							{								   //side-off condition
								data->imgData.bmission = true; // Auto Steering off
								usleep(100000);
								DesireSpeed_Write_uart(0);
								obstacle = false;
								state = SIDE_OFF;
								sprintf(data->imgData.missionString, "Side OFF");
							}
							usleep(50000);
							break;
						case LEFT:
							distance_3 = DistanceSensor_cm(3);
							distance_2 = DistanceSensor_cm(2);
							if (distance_3 <= 30 || distance_2 <= 30)
							{
								obstacle = true;
							}
							else if (obstacle == true && distance_3 > 30 && distance_2 > 30)
							{
								data->imgData.bmission = true; //Auto Steering off
								usleep(100000);
								DesireSpeed_Write_uart(0);
								//usleep(50000);
								obstacle = false;
								state = SIDE_OFF;
								sprintf(data->imgData.missionString, "Side OFF");
							}
							usleep(50000);
							break;
						default:
							state = FRONT_DETECT;
							break;
						}
						//error and go back step
						break;

					case SIDE_OFF:
						usleep(10000);
						data->imgData.bmission = true; //Auto Steering off
						usleep(10000);
						switch (data->missionData.overtakingData.headingDirection)
						{
						case RIGHT: //return left
							Winker_Write(LEFT_ON);
							laneChange(GO_LEFT, BASIC_SPEED, 50);
							Winker_Write(ALL_OFF);
							break;
						case LEFT: //return right
							Winker_Write(RIGHT_ON);
							laneChange(GO_RIGHT, BASIC_SPEED, 50);
							Winker_Write(ALL_OFF);
							break;
						default:
							break;
						}
						data->imgData.bmission = false;
						data->imgData.bprintString = false;
						DesireSpeed_Write_uart(BASIC_SPEED);
						state = DONE_O;
						data->missionData.overtakingFlag = false;
						break;

					default:
						break;
					}
					usleep(50000);
				}
				data->imgData.bmission = false;
				data->imgData.bprintString = false;
				overtake = DONE;
				signalLight = READY;
			}
		}

		if (signalLight && signalLight != DONE)
		{
			if (signalLightFunc(data))
			{
				signalLight = DONE;
				finish = READY;

				data->missionData.ms[7] = signalLight;
				data->missionData.ms[8] = finish;
			}
		}

		if (finish && finish != DONE)
		{
			finishFunc(data);
			finish = DONE;
		}

		if (data->missionData.changeMissionState)
		{
			start = data->missionData.ms[0];
			flyover = data->missionData.ms[1];
			priority = data->missionData.ms[2];
			parking = data->missionData.ms[3];
			tunnel = data->missionData.ms[4];
			roundabout = data->missionData.ms[5];
			overtake = data->missionData.ms[6];
			signalLight = data->missionData.ms[7];
			finish = data->missionData.ms[8];

			data->missionData.changeMissionState = false;
		}
		else
		{
			data->missionData.ms[0] = start;
			data->missionData.ms[1] = flyover;
			data->missionData.ms[2] = priority;
			data->missionData.ms[3] = parking;
			data->missionData.ms[4] = tunnel;
			data->missionData.ms[5] = roundabout;
			data->missionData.ms[6] = overtake;
			data->missionData.ms[7] = signalLight;
			data->missionData.ms[8] = finish;
		}

		usleep(80000); //50ms -> 70ms
	}
}

static struct thr_data *pexam_data = NULL;

void signal_handler(int sig)
{
	if (sig == SIGINT)
	{
		pthread_cancel(pexam_data->threads[0]);
		pthread_cancel(pexam_data->threads[1]);
		pthread_cancel(pexam_data->threads[2]);
		pthread_cancel(pexam_data->threads[3]);

		msgctl(pexam_data->msgq_id, IPC_RMID, 0);

		v4l2_streamoff(pexam_data->v4l2);
		vpe_stream_off(pexam_data->vpe->fd, V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE);
		vpe_stream_off(pexam_data->vpe->fd, V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE);

		disp_free_buffers(pexam_data->vpe->disp, NUMBUF);
		free_input_buffers(pexam_data->input_bufs, NUMBUF, false);
		free_overlay_plane(pexam_data->vpe->disp);

		disp_close(pexam_data->vpe->disp);
		vpe_close(pexam_data->vpe);
		v4l2_close(pexam_data->v4l2);

		printf("-- alSSU team program End --\n");
	}
}

/************************************************/
/*	MAIN	main	MAIN	main				*/
/************************************************/
int main(int argc, char **argv)
{
	struct v4l2 *v4l2;
	struct vpe *vpe;
	struct thr_data tdata;
	ptr_data = &tdata;
	int disp_argc = 3;
	int ret = 0;
	char *disp_argv[] = {"dummy", "-s", "4:480x272", "\0"}; // 추후 �??�?? ?���?? ?��?�� ?�� 처리..

	printf("------ main start ------\n");

	CarControlInit();
	PositionControlOnOff_Write(UNCONTROL); // position controller must be OFF !!!
	SpeedControlOnOff_Write(CONTROL);	   // speed controller must be also ON !!!

	DesireSpeed_Write(0);
	SpeedPIDProportional_Write(40);
	SpeedPIDIntegral_Write(40);
	SpeedPIDProportional_Write(40);

	/******************** imgProcess Data ********************/
	cSettingStatic(VPE_OUTPUT_W, VPE_OUTPUT_H);
	tdata.imgData.dump_request = false;
	tdata.imgData.bskip = false;
	tdata.imgData.bvideoRecord = false;
	tdata.imgData.bvideoSave = false;
	tdata.imgData.bcalibration = false;
	tdata.imgData.bdebug = false;
	tdata.imgData.bfilteringTest = false;
	tdata.imgData.debugMode = 1;
	tdata.imgData.btopview = true;
	tdata.imgData.topMode = 3;
	tdata.imgData.bauto = false;
	tdata.imgData.bwhiteLine = false;
	tdata.imgData.bspeedControl = true;
	tdata.imgData.bmission = false;
	tdata.imgData.bdark = false;
	tdata.imgData.bcheckFrontWhite = false;
	tdata.imgData.bcheckPriority = false;
	tdata.imgData.bcheckSignalLight = false;
	tdata.imgData.bcheckFinishLine = false;
	tdata.imgData.bprintString = false;
	tdata.imgData.bprintSensor = false;
	tdata.imgData.bprintMission = true;
	tdata.imgData.bprintTire = true;
	tdata.imgData.filtering_param.h = 75;
	tdata.imgData.filtering_param.s = 50;
	tdata.imgData.filtering_param.v = 200;
	tdata.imgData.filtering_param.canny1 = 118;
	tdata.imgData.filtering_param.canny2 = 242;
	sprintf(tdata.imgData.missionString, "(null)");

	/******************** Control Data ********************/
	tdata.controlData.settingSpeedVal = 40;
	tdata.controlData.desireSpeedVal = 0;
	tdata.controlData.beforeSpeedVal = 0;
	tdata.controlData.lightFlag = 0x00;
	tdata.controlData.steerVal = 1500;
	tdata.controlData.cameraY = 1660;
	CarLight_Write(0x00);
	CameraXServoControl_Write(1500);
	SteeringServoControl_Write(1500);
	CameraYServoControl_Write(1660);
	Winker_Write(ALL_OFF);

	/******************** Mission Data ********************/
	tdata.missionData.btunnel = false;
	tdata.missionData.broundabout = false;
	tdata.missionData.changeMissionState = false;
	tdata.missionData.frame_priority = 0;
	tdata.missionData.finish_distance = -1;
	tdata.missionData.parkingData.bparking = false;
	tdata.missionData.parkingData.horizontalFlag = false;
	tdata.missionData.parkingData.verticalFlag = false;
	tdata.missionData.parkingData.frontRight = false;
	tdata.missionData.parkingData.rearRight = false;
	tdata.missionData.overtakingFlag = false;
	tdata.missionData.overtakingData.updownCamera = CAMERA_DOWN;
	tdata.missionData.overtakingData.headingDirection = STOP;
	tdata.missionData.overtakingData.leftFlag = 0;
	tdata.missionData.overtakingData.rightFlag = 0;
	tdata.missionData.signalLightData.finalDirection = 0;
	tdata.missionData.finishData.checkFront = false;
	tdata.missionData.checkWhiteLineFlag = false;

	int i = 0;
	for (i = 0; i < 8; i++)
	{
		tdata.missionData.ms[i] = NONE;
	}

	// open vpe
	vpe = vpe_open();
	if (!vpe)
	{
		return 1;
	}

	vpe->src.width = CAPTURE_IMG_W;
	vpe->src.height = CAPTURE_IMG_H;
	describeFormat(CAPTURE_IMG_FORMAT, &vpe->src);

	// vpe output (disp data)
	vpe->dst.width = VPE_OUTPUT_W;
	vpe->dst.height = VPE_OUTPUT_H;
	describeFormat(VPE_OUTPUT_FORMAT, &vpe->dst);

	vpe->disp = disp_open(disp_argc, disp_argv);
	if (!vpe->disp)
	{
		ERROR("disp open error!");
		vpe_close(vpe);
		return 1;
	}

	set_z_order(vpe->disp, vpe->disp->overlay_p.id);
	set_global_alpha(vpe->disp, vpe->disp->overlay_p.id);
	set_pre_multiplied_alpha(vpe->disp, vpe->disp->overlay_p.id);
	alloc_overlay_plane(vpe->disp, OVERLAY_DISP_FORCC, 0, 0, OVERLAY_DISP_W, OVERLAY_DISP_H);
	// z-order, alpha, multiplied-alpha ?��?�� (overlay�?? ?��?�� plane �?? ?��?��)

	//vpe->deint = 0;
	vpe->translen = 1;

	MSG("Input(Camera) = %d x %d (%.4s)\nOutput(LCD) = %d x %d (%.4s)",
		vpe->src.width, vpe->src.height, (char *)&vpe->src.fourcc,
		vpe->dst.width, vpe->dst.height, (char *)&vpe->dst.fourcc);

	if (vpe->src.height < 0 || vpe->src.width < 0 || vpe->src.fourcc < 0 ||
		vpe->dst.height < 0 || vpe->dst.width < 0 || vpe->dst.fourcc < 0)
	{
		ERROR("Invalid parameters\n");
	}

	v4l2 = v4l2_open(vpe->src.fourcc, vpe->src.width, vpe->src.height);

	if (!v4l2)
	{
		ERROR("v4l2 open error!");
		disp_close(vpe->disp);
		vpe_close(vpe);
		return 1;
	}

	tdata.disp = vpe->disp;
	tdata.v4l2 = v4l2;
	tdata.vpe = vpe;
	tdata.bfull_screen = true;
	tdata.bstream_start = false;

	pexam_data = &tdata;

	ret = pthread_create(&tdata.threads[0], NULL, image_process_thread, &tdata);
	if (ret)
	{
		MSG("Failed creating image_process_thread");
	}
	pthread_detach(tdata.threads[0]);

	ret = pthread_create(&tdata.threads[1], NULL, mission_thread, &tdata);
	if (ret)
	{
		MSG("Failed creating mission_thread");
	}
	pthread_detach(tdata.threads[1]);

	ret = pthread_create(&tdata.threads[2], NULL, input_thread, &tdata);
	if (ret)
	{
		MSG("Failed creating input_thread");
	}
	pthread_detach(tdata.threads[2]);

	// ret = pthread_create(&tdata.threads[3], NULL, video_record_thread, &tdata);
	// if (ret)
	// {
	// 	MSG("Failed creating video_record_thread");
	// }
	// pthread_detach(tdata.threads[3]);

	/* register signal handler for <CTRL>+C in order to clean up */
	if (signal(SIGINT, signal_handler) == SIG_ERR)
	{
		MSG("could not register signal handler");
		closelog();
		exit(EXIT_FAILURE);
	}
	// signal error

	pause();

	return ret;
}
