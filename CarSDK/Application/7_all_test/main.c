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

#define CAPTURE_IMG_W 1280
#define CAPTURE_IMG_H 720
#define CAPTURE_IMG_SIZE (CAPTURE_IMG_W * CAPTURE_IMG_H * 2) // YUYU : 16bpp
#define CAPTURE_IMG_FORMAT "uyvy"

//해상도를 바꾸려면 이부분만 변경하면 됨
#define VPE_OUTPUT_W 640
#define VPE_OUTPUT_H 360

#define VPE_OUTPUT_IMG_SIZE (VPE_OUTPUT_W * VPE_OUTPUT_H * 3)
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

#define BASIC_SPEED 55		// 프로그램 기본 바퀴속도, autoSteer와 주로 사용.
#define BUZZER_PULSE 100000 // 기본 부저 길이

// 각종 structure는 control_mission.h 에 이동됨. 9/8(화)

struct thr_data
{
	struct display* disp;
	struct v4l2* v4l2;
	struct vpe* vpe;
	struct buffer** input_bufs;
	struct ControlData controlData;
	struct MissionData missionData;
	struct ImgProcessData imgData;
	unsigned char img_data_buf[VPE_OUTPUT_IMG_SIZE];

	int msgq_id;

	bool bfull_screen;
	bool bstream_start;
	pthread_t threads[4]; //�����尳��
};

static struct thr_data* ptr_data;
/******************** function ********************/
static void DesireDistance(int SettingSpeed, int SettingDistance, int SettingSteering);

static void SteeringServo_Write(signed short angle);

static int allocate_input_buffers(struct thr_data* data)
{
	int i;
	struct vpe* vpe = data->vpe;

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

static void free_input_buffers(struct buffer** buffer, uint32_t n, bool bmultiplanar)
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

static void draw_operatingtime(struct display* disp, uint32_t time, uint32_t itime, uint32_t mtime)
{
	FrameBuffer tmpFrame;
	unsigned char* pbuf[4];
	char strmtime[128];
	char strtime[128];
	char stritime[128];
	char strfps[128];

	memset(strmtime, 0, sizeof(strmtime));
	memset(strtime, 0, sizeof(strtime));
	memset(stritime, 0, sizeof(stritime));
	memset(strfps, 0, sizeof(strfps));

	sprintf(strmtime, "m thrd : %03d(ms)", mtime);
	sprintf(strtime, "optime : %03d(ms)", time);
	sprintf(stritime, "i thrd : %03d(ms)", itime);
	sprintf(strfps, "%4d(fps)", (itime == 0) ? 1000 : 1000 / itime);

	if (get_framebuf(disp->overlay_p_bo, pbuf) == 0)
	{
		tmpFrame.buf = pbuf[0];
		tmpFrame.format = draw_get_pixel_foramt(disp->overlay_p_bo->fourcc); //FORMAT_RGB888; //alloc_overlay_plane() -- FOURCC('R','G','2','4');
		tmpFrame.stride = disp->overlay_p_bo->pitches[0];					 //tmpFrame.width*3;

		drawString(&tmpFrame, strmtime, TIME_TEXT_X, TIME_TEXT_Y - 40, 0, TIME_TEXT_COLOR);
		drawString(&tmpFrame, strtime, TIME_TEXT_X, TIME_TEXT_Y - 20, 0, TIME_TEXT_COLOR);
		drawString(&tmpFrame, stritime, TIME_TEXT_X, TIME_TEXT_Y, 0, TIME_TEXT_COLOR);
		drawString(&tmpFrame, strfps, FPS_TEXT_X, FPS_TEXT_Y, 0, FPS_TEXT_COLOR);
	}
}

/************************************************/
/*	Function - img_process						*/
/************************************************/
static void img_process(struct display* disp, struct buffer* cambuf, struct thr_data* t_data, float* map1, float* map2)
{
	unsigned char srcbuf[VPE_OUTPUT_W * VPE_OUTPUT_H * 3];
	uint32_t optime;
	struct timeval st, et;
	unsigned char* cam_pbuf[4];
	bool delay_flag = false;

	if (get_framebuf(cambuf, cam_pbuf) == 0)
	{
		memcpy(srcbuf, cam_pbuf[0], VPE_OUTPUT_W * VPE_OUTPUT_H * 3);
		gettimeofday(&st, NULL);

		/********************************************************/
		/*			우리가 만든 알고리즘 함수를 넣는 부분		*/
		/********************************************************/

		/* 라인 필터링이나 canny 결과 확인 */
		if (t_data->imgData.bdebug)
		{
			if (t_data->imgData.debugMode == 9)
				OpenCV_remap(srcbuf, VPE_OUTPUT_W, VPE_OUTPUT_H, srcbuf, map1, map2);
			debugFiltering(srcbuf, VPE_OUTPUT_W, VPE_OUTPUT_H, srcbuf, t_data->imgData.debugMode);
		}

		/* 미션 진행중에 처리하는 영상처리 */
		else if (t_data->imgData.bmission)
		{
			/* 추월차로시에 사용 */
			if (t_data->missionData.overtakingFlag &&
				t_data->missionData.overtakingData.updownCamera == CAMERA_UP)
			{
				usleep(100000);
				/*check를 위한 camera up*/
				bool check_direction;
				check_direction = checkObstacle(srcbuf, VPE_OUTPUT_W, VPE_OUTPUT_H, srcbuf);
				/*오른쪽인지 왼쪽인지 매 번 확인*/
				if (check_direction == true)
				{ //true=>left
					t_data->missionData.overtakingData.leftFlag++;
				}
				else
				{ //false =>right
					t_data->missionData.overtakingData.rightFlag++;
				}

				/*3회 판단 이후 확인*/
				if ((t_data->missionData.overtakingData.rightFlag + t_data->missionData.overtakingData.leftFlag) >= 3)
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
						buzzer(1, 0, BUZZER_PULSE);
						delay_flag = true;
						sprintf(t_data->imgData.missionString, "check YELLOW");
						t_data->missionData.signalLightData.state = DETECT_YELLOW;
					}
					break;

				case DETECT_YELLOW:
					if (checkYellow(srcbuf, VPE_OUTPUT_W, VPE_OUTPUT_H, srcbuf))
					{
						buzzer(1, 0, BUZZER_PULSE);
						delay_flag = true;
						sprintf(t_data->imgData.missionString, "check GREEN");
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
							t_data->missionData.signalLightData.state = DETECTION_FINISH;
							t_data->missionData.signalLightData.finalDirection = 1;
							t_data->imgData.bcheckSignalLight = false;
						}
						else if (t_data->missionData.signalLightData.Accumulation_greenVal <= -3)
						{
							buzzer(1, 0, BUZZER_PULSE * 2);
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
						DesireSpeed_Write(t_data->controlData.desireSpeedVal);
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
					if (t_data->missionData.frame_priority < 2)
						t_data->missionData.frame_priority++;
					printf("img thread : isPriorityStop() return 1; frame =%d\n", t_data->missionData.frame_priority);
				}
				else
				{
					if (t_data->missionData.frame_priority > 0)
						t_data->missionData.frame_priority--;
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

			if (t_data->imgData.bcheckFrontWhite && t_data->missionData.finish_distance == -1) {
				t_data->missionData.checkWhiteLineFlag = checkWhiteLine(srcbuf, VPE_OUTPUT_W, VPE_OUTPUT_H);
				if (t_data->missionData.checkWhiteLineFlag) {
					t_data->imgData.btopview = false;
					t_data->imgData.bauto = false;
				}

			}

			if (t_data->imgData.btopview && t_data->imgData.bskip == false)
			{
				topview_transform(srcbuf, VPE_OUTPUT_W, VPE_OUTPUT_H, srcbuf, t_data->imgData.topMode);
			}

			if (t_data->imgData.bauto)
			{
				int steerVal = autoSteering(srcbuf, VPE_OUTPUT_W, VPE_OUTPUT_H, srcbuf, t_data->imgData.bwhiteLine);
				if (steerVal != 9999)
				{
					t_data->controlData.steerVal = 1500 - steerVal;
					SteeringServoControl_Write(t_data->controlData.steerVal);
				}
				if (t_data->imgData.bspeedControl)
				{
					t_data->controlData.desireSpeedVal = auto_speedMapping(steerVal, BASIC_SPEED);
					if (t_data->controlData.desireSpeedVal != t_data->controlData.beforeSpeedVal)
					{
						//이전 속도와 달라졌을 때만 속도값 인가.
						DesireSpeed_Write(t_data->controlData.desireSpeedVal);
						t_data->controlData.beforeSpeedVal = t_data->controlData.desireSpeedVal;
					}
				}
			}
			if (t_data->missionData.checkWhiteLineFlag) {
				OpenCV_remap(srcbuf, VPE_OUTPUT_W, VPE_OUTPUT_H, srcbuf, map1, map2);

				topview_transform(srcbuf, VPE_OUTPUT_W, VPE_OUTPUT_H, srcbuf, 1);

				t_data->missionData.finish_distance = calculDistance_FinishLine(srcbuf, VPE_OUTPUT_W, VPE_OUTPUT_H, srcbuf);
				t_data->missionData.checkWhiteLineFlag = false;
				t_data->imgData.btopview = true;
				t_data->imgData.bauto = true;
			}

			/*checkWhiteLineFlag가 True인 경우, RoundAbout */

		}

		/********************************************************/
		/*			영상처리 종료								*/
		/********************************************************/

		/* 영상처리후 오버레이로 정보 등등 출력.*/
		if (t_data->imgData.bprintString)
		{
			displayPrintStr(srcbuf, VPE_OUTPUT_W, VPE_OUTPUT_H, t_data->imgData.missionString);
		}
		if (t_data->imgData.bprintMission)
		{
			displayPrintMission(srcbuf, VPE_OUTPUT_W, VPE_OUTPUT_H,
				(int)t_data->missionData.ms[0], (int)t_data->missionData.ms[1], (int)t_data->missionData.ms[2],
				(int)t_data->missionData.ms[3], (int)t_data->missionData.ms[4], (int)t_data->missionData.ms[5],
				(int)t_data->missionData.ms[6], (int)t_data->missionData.ms[7], (int)t_data->missionData.ms[8]);
		}
		if (t_data->imgData.bprintSensor)
		{
			displayPrintSensor(srcbuf, VPE_OUTPUT_W, VPE_OUTPUT_H,
				DistanceSensor_cm(1), DistanceSensor_cm(2), DistanceSensor_cm(3),
				DistanceSensor_cm(4), DistanceSensor_cm(5), DistanceSensor_cm(6), StopLine(4));
		}
		if (t_data->imgData.bprintTire)
		{
			overlayPrintAngle(srcbuf, VPE_OUTPUT_W, VPE_OUTPUT_H, srcbuf, t_data->controlData.steerVal);
		}

		/*		현재 영상 .bmp파일로 저장		*/
		if (t_data->imgData.dump_request)
		{
			opencv_imwrite(srcbuf);
			t_data->imgData.dump_request = false;
		}

		/*		현재 영상 .avi파일로 녹화		*/
		if (t_data->imgData.bvideoRecord)
		{
			memcpy(t_data->img_data_buf, srcbuf, VPE_OUTPUT_IMG_SIZE);
		}

		/* 신호등 검출화면을 유지하기 위한 delay */
		if (delay_flag)
		{
			usleep(1000000); // 1000ms
			delay_flag = false;
		}

		memcpy(cam_pbuf[0], srcbuf, VPE_OUTPUT_W * VPE_OUTPUT_H * 3);
		gettimeofday(&et, NULL);
		optime = ((et.tv_sec - st.tv_sec) * 1000) + ((int)et.tv_usec / 1000 - (int)st.tv_usec / 1000);
		draw_operatingtime(disp, optime, t_data->imgData.loopTime, t_data->missionData.loopTime);
	}
}

/************************************************/
/*	Thread - image_process_thread				*/
/************************************************/
void* image_process_thread(void* arg)
{
	struct thr_data* data = (struct thr_data*)arg;
	struct v4l2* v4l2 = data->v4l2;
	struct vpe* vpe = data->vpe;
	struct buffer* capt;
	struct timeval st, et;
	bool isFirst = true;
	int index;
	int i;
	float map1[VPE_OUTPUT_W * VPE_OUTPUT_H] = {
		0,
	};
	float map2[VPE_OUTPUT_W * VPE_OUTPUT_H] = {
		0,
	};
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
	}

	MSG("Ok!");
	return NULL;
}

/************************************************/
/*	Thread - input_thread						*/
/************************************************/
void* input_thread(void* arg)
{
	struct thr_data* data = (struct thr_data*)arg;

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
	MSG("\t mission: mission display output on/off");
	MSG("\t sensor : sensor  display output on/off");
	MSG("\t tire   : tire    display output on/off");
	MSG("\t ms	   : mission on/off");
	MSG("\t dump   : save image file");
	MSG("\t video  : video record start");
	MSG("\t save   : save video file");
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

void* video_record_thread(void* arg)
{
	struct thr_data* data = (struct thr_data*)arg;
	struct timeval st, et;
	bool videoEnd = false;
	int fps = 10;
	int delay_ms = 1000 / fps;
	int current_frame = 0;
	int recordTime_ms;

	while (!data->imgData.bvideoRecord)
	{
		usleep(500000);
	}
	printf("\tvideo_record_thred() : ON\n");

	// st체크를 이곳에서 하고, et체크를 두 번째 while에 하여 매 프레임마다 delay_ms만큼 누산시키어
	// 프레임이 찍히는 시점과 영상에서의 해당 시점을 근사하도록 한다. (0.1초 절대값 캡처 대신 영상의 전체길이의 비율로)
	gettimeofday(&st, NULL);

	while (1)
	{
		current_frame++;

		if (videoEnd)
		{
			usleep(10000000);
		}
		else if (data->imgData.bvideoSave)
		{
			opencv_videoclose();
			videoEnd = true;
		}
		else if (data->imgData.bvideoRecord)
		{
			opencv_videowrite(data->img_data_buf);
		}

		while (1) //영상 녹화 싱크를 맞춰주기 위한 delay
		{
			gettimeofday(&et, NULL);
			recordTime_ms = ((et.tv_sec - st.tv_sec) * 1000) + ((int)et.tv_usec / 1000 - (int)st.tv_usec / 1000);
			if (recordTime_ms > (delay_ms * current_frame))
			{
				break;
			}
			else
			{
				usleep(5000); // 5ms delay
			}
		}
	}
}

/************************************************/
/*	Thread - mission_thread						*/
/************************************************/
void* mission_thread(void* arg)
{
	struct thr_data* data = (struct thr_data*)arg;
	struct timeval time;
	time.tv_sec = 0;

	enum MissionState start = NONE;
	enum MissionState flyover = NONE;
	enum MissionState priority = NONE;
	enum MissionState parking = NONE;
	enum MissionState roundabout = NONE;
	enum MissionState tunnel = NONE;
	enum MissionState overtake = NONE;
	enum MissionState signalLight = NONE;
	enum MissionState finish = NONE;

	//int i = 0;
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
		data->missionData.loopTime = timeCheck(&time);

		if (start && start != DONE)
		{
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

		if (flyover && flyover != DONE)
		{
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

		if (priority && priority != DONE)
		{
			//imgProcess에서 우선정지표지판 체크 활성화
			data->imgData.bcheckPriority = true;

			if (data->missionData.frame_priority >= 2) //우선정지표지판 2프레임 검출.
			{
				data->imgData.bskip = true;
				DesireSpeed_Write(0);
				Winker_Write(ALL_ON);
				while (data->imgData.bcheckPriority)
				{
					if (data->missionData.frame_priority == 0) //우선정지표지판 사라지면
					{
						data->imgData.bcheckPriority = false; //imgProcess에서 우선정지표지판 체크 비활성화
					}
					usleep(100000);
				}
				usleep(1000000);    //1초 대기
				Winker_Write(ALL_OFF);
				DesireSpeed_Write(BASIC_SPEED);
				priority = DONE;
				data->imgData.bskip = false;
			}
		}

		if (parking && parking != DONE)
		{
			if ((data->controlData.steerVal <= 1600 && data->controlData.steerVal >= 1400) || parking == REMAIN) 
			{
				if (DistanceSensor_cm(2) <= 28) //처음 벽이 감지되었을 경우
				{
					struct timeval st_p, et_p;
					gettimeofday(&st_p, NULL);

					data->imgData.bprintString = true;
					sprintf(data->imgData.missionString, "Parking");
					int parking_width = 0;
					//int first_error_distance = 0;
					//int second_error_distance = 0;
					//int first_error_flag = 1;
					bool wrong_detection = 1;
					int encoderVal = 0;

					enum ParkingState state = FIRST_WALL;
					enum HorizontalStep step_h = FIRST_BACKWARD;
					enum VerticalStep step_v = FIRST_BACKWARD_V;

					while (state && wrong_detection) // state == END가 아닌이상 루프 진행
					{
						data->missionData.loopTime = timeCheck(&time);

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
							/*
							주차 폭에 대한 거리를 측정하기 위해 거리 측정 시작
							*/

							/*
							우선 정지 표지판을 감지하면 주차에서 탈출되도록 조치
							*/
							if (data->missionData.frame_priority)
							{
								wrong_detection = 0;
								break;
							}
							// Encoder 측정
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
								// 주차 공간 측정이 안되는 경우, 잘못된 주차 분기 진입으로 판단하고 탈출.
							}
							if (data->missionData.parkingData.frontRight == true)
							{
								/*
								거리 측정 종료 -> 측정 거리를 변수에 담는다.
								*/
								printf("Result Width : %-3d\n", parking_width);

								if (parking_width <= 850)
									data->missionData.parkingData.verticalFlag = true;
								else
									data->missionData.parkingData.horizontalFlag = true;

								state = SECOND_WALL;
							}
							break;

						case SECOND_WALL:
							sprintf(data->imgData.missionString, "Second Wall");
							if (data->missionData.parkingData.rearRight == true)
							{
								state = PARKING_START;
								data->imgData.bmission = true;
								// 두번 째 벽에 차량 우측 후방 센서가 걸린 상태이다. -> 수직 또는 수평 주차 진행.
							}
							break;

						case PARKING_START:
							sprintf(data->imgData.missionString, "Parking Start");
							/*
							수직 및 수평 주차 구문 추가.
							*/

							if (data->missionData.parkingData.verticalFlag && data->missionData.parkingData.horizontalFlag == false)
							{
								// DesireSpeed_Write(0);
								// usleep(50000);
								// EncoderCounter_Write(0);
								// usleep(50000);
								DesireDistance(60, 200, 1500);
								while (data->missionData.parkingData.verticalFlag)
								{
									data->missionData.loopTime = timeCheck(&time);
									switch (step_v)
									{
									case FIRST_BACKWARD_V:
										sprintf(data->imgData.missionString, "FIRST_BACKWARD_V");
										SteeringServo_Write(1050);
										// 회전 각 수정 부분
										usleep(150000);
										DesireSpeed_Write(-60);
										usleep(200000);
										if (DistanceSensor_cm(3) <= 13 && DistanceSensor_cm(5) <= 13)
										{
											DesireSpeed_Write(0);
											usleep(50000);
											step_v = SECOND_BACKWARD_V;
										}
										break;

									case SECOND_BACKWARD_V:
										sprintf(data->imgData.missionString, "SECOND_BACKWARD_V");
										SteeringServo_Write(1500);
										usleep(100000);
										DesireSpeed_Write(-35);
										usleep(50000);
										// 바퀴를 일자로 맞춰주고 후진한다.
										if (DistanceSensor_cm(2) <= 11)
										{
											DesireSpeed_Write(0);
											usleep(50000);
											step_v = RIGHT_FRONT_V;
											break;
										}
										else if (DistanceSensor_cm(6) <= 11)
										{
											DesireSpeed_Write(0);
											usleep(50000);
											step_v = LEFT_FRONT_V;
											break;
										}
										if (DistanceSensor_cm(5) <= 6)
										{
											// 언더스티어 상황
											printf("under steer\n");
											DesireSpeed_Write(0);
											usleep(200000);
											step_v = UNDER_STEER_V;
											break;
										}
										else if (DistanceSensor_cm(3) <= 6)
										{
											// 오버스티어 상황
											printf("over steer\n");
											DesireSpeed_Write(0);
											usleep(50000);
											step_v = OVER_STEER_V;
											break;
										}
										break;

									case UNDER_STEER_V:
										sprintf(data->imgData.missionString, "UNDER_STEER");
										DesireDistance(60, 100, 1300);
										usleep(30000);
										DesireDistance(60, 100, 1500);
										usleep(30000);
										DesireDistance(60, 200, 1700);
										usleep(30000);

										step_v = SECOND_BACKWARD_V;
										break;

									case OVER_STEER_V:
										sprintf(data->imgData.missionString, "OVER_STEER");
										//DesireDistance(23, 100, 1300);
										//usleep(200000);
										DesireDistance(-60, 500, 1500);
										//usleep(300000);
										//DesireDistance(23, 200, 1700);
										//usleep(200000);
										step_v = SECOND_BACKWARD_V;
										break;

									case RIGHT_FRONT_V:
										sprintf(data->imgData.missionString, "RIGHT_FRONT_V");
										int right_difference = DistanceSensor_cm(2) - DistanceSensor_cm(3);
										DesireDistance(60, 100, 1500 - (right_difference * 100));
										//usleep(200000);
										DesireDistance(-60, 400, 1500);
										//usleep(200000);
										if (abs(right_difference) < 3)
										{
											DesireSpeed_Write(0);
											usleep(50000);
											step_v = FIRST_FORWARD_V;
										}
										break;

									case LEFT_FRONT_V:
										sprintf(data->imgData.missionString, "RIGHT_FRONT_V");
										int left_difference = DistanceSensor_cm(6) - DistanceSensor_cm(5);
										DesireDistance(60, 100, 1500 + (left_difference * 100));
										//usleep(200000);
										DesireDistance(-60, 400, 1500);
										//usleep(200000);
										if (abs(left_difference) < 3)
										{
											DesireSpeed_Write(0);
											usleep(50000);
											step_v = FIRST_FORWARD_V;
										}
										break;

									case FIRST_FORWARD_V:
										sprintf(data->imgData.missionString, "FIRST_FORWARD_V");
										DesireDistance(-50, 400, 1500);
										usleep(1000000);
										step_v = SECOND_FORWARD_V;
										Winker_Write(ALL_ON);
										buzzer(1, 0, 300000);
										Winker_Write(ALL_OFF);
										break;

									case SECOND_FORWARD_V:
										sprintf(data->imgData.missionString, "SECOND_FORWARD_V");
										DesireSpeed_Write(60);
										usleep(50000);
										if (DistanceSensor_cm(2) >= 20 && DistanceSensor_cm(6) >= 20)
										{
											DesireSpeed_Write(0);
											usleep(50000);
											DesireDistance(60, 200, 1500);
											step_v = FINISH_V;
										}
										break;

									case FINISH_V:
										sprintf(data->imgData.missionString, "FINISH_V");
										DesireDistance(60, 1150, 1050);
										data->missionData.parkingData.verticalFlag = 0;
										break;

									default:
										break;
									}
									usleep(150000);
								}
							}
							else if (data->missionData.parkingData.verticalFlag == false && data->missionData.parkingData.horizontalFlag)
							{
								DesireDistance(60, 230, 1500);
								// 주차 각 수정 부분
								while (data->missionData.parkingData.horizontalFlag)
								{
									data->missionData.loopTime = timeCheck(&time);
									switch (step_h)
									{
									case FIRST_BACKWARD:
										sprintf(data->imgData.missionString, "FIRST_BACKWARD");
										DesireDistance(-60, 820, 1050);
										//usleep(200000);
										DesireDistance(-60, 370, 1500);
										//usleep(200000);
										SteeringServo_Write(1900);
										usleep(100000);
										DesireSpeed_Write(-35);
										usleep(50000);
										while (1)
										{
											if (DistanceSensor_cm(4) <= 6)
											{
												DesireSpeed_Write(0);
												break;
											}
											usleep(50000);
										}
										SteeringServo_Write(1250);
										usleep(150000);
										DesireSpeed_Write(35);
										usleep(50000);
										while (1)
										{
											if ((abs(DistanceSensor_cm(2) - DistanceSensor_cm(3)) <= 3) || DistanceSensor_cm(1) <= 4)
											{
												DesireSpeed_Write(0);
												step_h = SECOND_BACKWARD;
												break;
											}
											usleep(30000);
										}
										usleep(50000);
										break;

									case SECOND_BACKWARD:
										sprintf(data->imgData.missionString, "SECOND_BACKWARD");
										//sprintf(data->imgData.missionString, "d1 = %d, d2 = %d, d3 = %d", DistanceSensor_cm(1), DistanceSensor_cm(2), DistanceSensor_cm(3));
										int difference = DistanceSensor_cm(2) - DistanceSensor_cm(3);
										if (difference < -2)
										{
											//sprintf(data->imgData.missionString, "d1 = %d, d2 = %d, d3 = %d", DistanceSensor_cm(1), DistanceSensor_cm(2), DistanceSensor_cm(3));
											DesireDistance(-40, 400, 1300);
											//usleep(200000);
											if (abs(DistanceSensor_cm(2) - DistanceSensor_cm(3)) <= 2)
											{
												DesireSpeed_Write(0);
												usleep(20000);
												break;
											}
											DesireDistance(40, 400, 1700);
											//usleep(200000);
										}
										else if (difference > 2)
										{
											//sprintf(data->imgData.missionString, "d1 = %d, d2 = %d, d3 = %d", DistanceSensor_cm(1), DistanceSensor_cm(2), DistanceSensor_cm(3));
											DesireDistance(-40, 400, 1700);
											//usleep(200000);
											if (abs(DistanceSensor_cm(2) - DistanceSensor_cm(3)) <= 2)
											{
												DesireSpeed_Write(0);
												usleep(20000);
												break;
											}
											DesireDistance(40, 400, 1300);
											//usleep(200000);
										}
										if (abs(difference) <= 2)
										{
											//sprintf(data->imgData.missionString, "d1 = %d, d2 = %d, d3 = %d", DistanceSensor_cm(1), DistanceSensor_cm(2), DistanceSensor_cm(3));
											DesireSpeed_Write(0);
											usleep(100000);
											SteeringServo_Write(1500);
											usleep(1000000);
											step_h = SECOND_FORWARD;
											Winker_Write(ALL_ON);
											buzzer(1, 0, 300000);
											Winker_Write(ALL_OFF);
										}
										break;

										//case FIRST_FORWARD:
										//	DesireDistance(30, 250, 1000);
										//	step = SECOND_FORWARD;
										//	break;

									case SECOND_FORWARD:
										sprintf(data->imgData.missionString, "SECOND_FORWARD");
										DesireSpeed_Write(-40);
										usleep(30000);
										if (DistanceSensor_cm(4) <= 5)
										{
											DesireSpeed_Write(0);
											usleep(10000);
											SteeringServo_Write(1750);
											usleep(100000);

											step_h = ESCAPE;
										}
										break;

									case ESCAPE:
										sprintf(data->imgData.missionString, "ESCAPE");
										DesireSpeed_Write(40);
										usleep(100000);
										if (DistanceSensor_cm(1) <= 8 || DistanceSensor_cm(1) >= 20)
										{
											DesireSpeed_Write(0);
											SteeringServo_Write(1400);
											usleep(150000);
											step_h = ESCAPE_2;
										}
										break;

									case ESCAPE_2:
										sprintf(data->imgData.missionString, "ESCAPE_2");
										DesireSpeed_Write(-40);
										usleep(100000);
										if (DistanceSensor_cm(4) <= 9 || DistanceSensor_cm(3) <= 5)
										{
											DesireSpeed_Write(0);
											usleep(50000);
											step_h = ESCAPE_3;
										}
										break;

									case ESCAPE_3:
										sprintf(data->imgData.missionString, "ESCAPE_3");
										DesireDistance(60, 600, 1950);
										step_h = FINISH;
										break;

									case FINISH:
										sprintf(data->imgData.missionString, "FINISH");
										DesireDistance(60, 700, 1300);
										data->missionData.parkingData.horizontalFlag = 0;
										break;

									default:
										break;
									}
									usleep(150000);
								}
							}
							state = DONE_P;

							gettimeofday(&et_p, NULL);
							int parkingTime = (((et_p.tv_sec - st_p.tv_sec) * 1000) + ((int)et_p.tv_usec / 1000 - (int)st_p.tv_usec / 1000)) / 1000;
							printf("parking time : %d\n", parkingTime);

							if (parking == READY)
								parking = REMAIN;
							else if (parking == REMAIN)
								parking = DONE;

							break;

						default:
							break;
						}
						usleep(50000);
					}
					data->imgData.bmission = false;
					data->imgData.bprintString = false;
					if (parking == REMAIN)
					{
						printf("First Parking is Done!\n");
						usleep(5000000);
					}
					if (parking == DONE)
					{
						printf("Second Parking is Dome!\n");
						usleep(5000000);
					}
				}
			}
		}

		if (tunnel && tunnel != DONE)
		{
			data->imgData.bdark = true;
			if (data->missionData.btunnel && DistanceSensor_cm(2) < 20 && DistanceSensor_cm(6) < 20)
			{
				data->imgData.bprintString = true;
				data->imgData.bmission = true;
				data->imgData.bdark = false;

				frontLightOnOff(data->controlData.lightFlag, true);
				sprintf(data->imgData.missionString, "mission thread : tunnel detect");

				while (true)
				{
					data->missionData.loopTime = timeCheck(&time);
					int c2 = DistanceSensor_cm(2);
					int c6 = DistanceSensor_cm(6);
					if (Tunnel_isEnd(c2, c6, 50, 50))
					{
						sprintf(data->imgData.missionString, "tunnel out");
						break;
					}

					data->controlData.steerVal = Tunnel_SteerVal(c2, c6);
					sprintf(data->imgData.missionString, "steer = %d, %d : %d", data->controlData.steerVal, c2, c6);

					SteeringServo_Write(data->controlData.steerVal);

					usleep(10000);
				}
				DesireSpeed_Write(0);

				frontLightOnOff(data->controlData.lightFlag, false);

				buzzer(1, 0, 500000);
				usleep(100000);

				DesireDistance(-40, 400, 1500);
				usleep(100000);

				printf("Tunnel OUT\n");
				tunnel = DONE;
				data->imgData.bmission = false;
				data->imgData.bprintString = false;
				data->missionData.btunnel = false;
			}
		}

		if (roundabout && roundabout != DONE)
		{
			data->imgData.bcheckFrontWhite = true;
			//printf("roundabout 분기 \n");
			if (StopLine(5) || data->missionData.finish_distance!=-1)
			{
				onlyDistance(BASIC_SPEED, data->missionData.finish_distance);
				data->missionData.finish_distance = -1;
				data->imgData.bwhiteLine = true;
				data->imgData.bprintString = true;
				sprintf(data->imgData.missionString, "round about");
				printf("roundabout IN\n");
				int speed = BASIC_SPEED;
				int flag_END = 0;

				DesireSpeed_Write(0);
				data->imgData.bspeedControl = false;
				//SteeringServo_Write(1500);
				enum RoundaboutState state = WAIT_R;
				while (state != DONE_R)
				{
					data->missionData.loopTime = timeCheck(&time);
					switch (state)
					{
					case NONE_R:
						break;

					case WAIT_R:
						data->imgData.bmission = true;
						if (RoundAbout_isStart(DistanceSensor_cm(1)))
						{
							sprintf(data->imgData.missionString, "ROUND_GO_1-1");
							printf("ROUND_GO_1-1\n");

							state = ROUND_GO_1;
						}
						break;

					case ROUND_GO_1:
						//DesireDistance(speed, 600, 1500, &(data->controlData)); //앞 센서 받아오면서 일정거리 가는 함수 추가.
						onlyDistance(speed, 400);
						sprintf(data->imgData.missionString, "ROUND_GO_1-2");
						printf("ROUND_GO_1_2\n");
						usleep(100000);

						data->imgData.bmission = false;
						onlyDistance(speed, 1250);

						sprintf(data->imgData.missionString, "ROUND_STOP");
						printf("ROUND_STOP\n");

						state = ROUND_STOP;
						break;

					case ROUND_STOP:
						if ((DistanceSensor_cm(4) <= 25) /*|| (DistanceSensor_cm(5) <= 6)*/)
						{
							speed = 80;
							DesireSpeed_Write(speed);
							sprintf(data->imgData.missionString, "ROUND_GO_2");
							printf("ROUND_GO_2\n");

							state = ROUND_GO_2;
						}
						break;

					case ROUND_GO_2:
						if ((DistanceSensor_cm(4) <= 8) /* || (DistanceSensor_cm(5) <= 6)*/)
						{
							printf("speed up \n");
							if (speed < 100)
								speed += 5;
						}
						else if ((DistanceSensor_cm(1) <= 8) /* || (DistanceSensor_cm(6) <= 6)*/)
						{
							DesireSpeed_Write(0);
							printf("stop and speed down \n");
							if (speed > 30)
								speed -= 10;
							usleep(1900000);
							break;
						}
						DesireSpeed_Write(speed);
						//if (abs(data->controlData.steerVal - 1500) < 60)
						if (data->controlData.steerVal - 1500 < 30) // steerVal 1500 이상으로 유지되다가 직진 구간이 나올 때
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
				roundabout = DONE;
				data->missionData.broundabout = false;
				data->imgData.bspeedControl = true;
				data->imgData.bprintString = false;
				data->imgData.bcheckFrontWhite = false;
				data->missionData.finish_distance = -1;
			}
		}

		if (overtake && overtake != DONE)
		{
			/* 분기진입 명령 지시 */
			if (data->controlData.steerVal <= 1600 &&
				data->controlData.steerVal >= 1400)
			{
				if (DistanceSensor_cm(1) < 30) //전방 장애물 감지 //주차 상황이 아닐때, 분기진입 가능
				{
					data->imgData.btopview = false;	 //topview off
					data->imgData.bmission = true;	 //영상처리 X
					//data->imgData.bwhiteLine = true; // 흰색 직선 O
					data->imgData.bprintString = true;
					sprintf(data->imgData.missionString, "overtake");
					printf("overtake \n");
					enum OvertakeState state = FRONT_DETECT;
					data->missionData.overtakingData.headingDirection = STOP;
					data->missionData.overtakingFlag = true;
					data->imgData.bwhiteLine = true;
					bool obstacle = false;
					int thresDistance = 450;
					/*차량 정지*/
					DesireSpeed_Write(0);

					while (state)
					{
						data->missionData.loopTime = timeCheck(&time);
						switch (state)
						{
						case FRONT_DETECT:
							/* 장애물 좌우판단을 위한 카메라 각도조절 */
							sprintf(data->imgData.missionString, "Front Detect");
							if (data->missionData.overtakingData.headingDirection == STOP)
							{
								data->controlData.cameraY = 1610;
								CameraYServoControl_Write(data->controlData.cameraY);
								data->missionData.overtakingData.updownCamera = CAMERA_UP;
							}
							/* 장애물 좌우 판단 및 비어있는 차선으로 전진하려는 코드*/
							while (data->missionData.overtakingData.headingDirection == STOP)
							{
								data->missionData.loopTime = timeCheck(&time);
								usleep(50000);
							}
							/*판단 받으면 Camera 원래 위치로 돌림*/
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
							/*판단 이후 해당 방향 전진*/
							if (data->missionData.overtakingData.headingDirection == RIGHT &&
								data->missionData.overtakingData.updownCamera == CAMERA_DOWN)
							{
								sprintf(data->imgData.missionString, "Right to go");
								/*출발*/
								Winker_Write(RIGHT_ON);
								DesireDistance(50, thresDistance, 1100);
								Winker_Write(ALL_OFF);
								/* 센서 오류 인식 방지*/
								usleep(500000);
								/*thresDistance이상 가서 전방 거리 재확인*/
								if (DistanceSensor_cm(1) < 20)
								{
									sprintf(data->imgData.missionString, "Detect Error");
									/*정지, 후진 및 방향 전환*/
									DesireDistance(-50, thresDistance, 1100);
									/*정지 및 방향 전환 명령*/
									data->missionData.overtakingData.headingDirection = LEFT;
								}
								else
								{ /*전방 미탐지*/
									state = SIDE_ON;
									sprintf(data->imgData.missionString, "Detect Side");
									/*전진하는 동안 전방 센서가 20 이상 멀어지면 SIDE_ON으로 진행*/
									DesireSpeed_Write(BASIC_SPEED);
								}
							}
							else if (data->missionData.overtakingData.headingDirection == LEFT &&
								data->missionData.overtakingData.updownCamera == CAMERA_DOWN)
							{

								sprintf(data->imgData.missionString, "Left to go");
								/*출발*/
								Winker_Write(LEFT_ON);
								DesireDistance(50, thresDistance, 1900);
								Winker_Write(ALL_OFF);
								/* 센서 오류 인식 방지*/
								usleep(500000);
								/*thresDistance이상 가서 전방 거리 재확인*/
								if (DistanceSensor_cm(1) < 20)
								{
									/*정지, 후진 및 방향 전환*/
									sprintf(data->imgData.missionString, "Detect Error");
									DesireDistance(-50, thresDistance, 1900);
									/*정지 후 방향 전환 명령*/
									data->missionData.overtakingData.headingDirection = RIGHT;
								}
								else
								{
									/*전진하는 동안 전방 센서가 20 이상 멀어지면 SIDE_ON으로 진행*/
									state = SIDE_ON;
									sprintf(data->imgData.missionString, "Detect Side");
									DesireSpeed_Write(BASIC_SPEED);
								}
							}
							else
							{ /*STOP이 유지되는 경우 멈춤*/
							}

							break;

						case SIDE_ON:
							/*Auto Steering 동작*/
							data->imgData.bmission = false;
							/* 현재 장애물이 어디있느냐에 따라 side 센서(2,3 or 4,5)로 감지하는 코드*/
							//right
							if (data->missionData.overtakingData.headingDirection == RIGHT)
							{
								/*장애물 통과 확인*/
								if (DistanceSensor_cm(5) < 30 || DistanceSensor_cm(6) < 30)
								{
									obstacle = true;
								}
								else if (obstacle == true)
								{
									/*장애물 통과*/
									if (DistanceSensor_cm(5) > 30 && DistanceSensor_cm(6) > 30)
									{
										DesireSpeed_Write(0);
										usleep(50000);
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
								/*장애물 통과 확인*/
								if (DistanceSensor_cm(3) < 30 || DistanceSensor_cm(2) < 30)
								{
									obstacle = true;
								}
								else if (obstacle == true)
								{
									/*장애물 통과*/
									if (DistanceSensor_cm(3) > 30 && DistanceSensor_cm(2) > 30)
									{
										DesireSpeed_Write(0);
										usleep(50000);
										obstacle = false;
										state = SIDE_OFF;
										sprintf(data->imgData.missionString, "Side OFF");
									}
								}
								usleep(500000);
							}
							//error and go back step
							else
							{
								state = FRONT_DETECT;
							}
							break;

						case SIDE_OFF:
							/*원래 차선으로 복귀하는 코드*/
							usleep(10000);
							data->imgData.bmission = true; //Auto Steering off
							//right
							if (data->missionData.overtakingData.headingDirection == RIGHT)
							{
								/*복귀 좌회전 방향 설정 및 전진*/
								Winker_Write(LEFT_ON);
								DesireDistance(50, thresDistance + 100, 1900);
								Winker_Write(ALL_OFF);
							}
							//left
							else if (data->missionData.overtakingData.headingDirection == LEFT)
							{
								/*복귀 우회전 방향 설정*/
								Winker_Write(RIGHT_ON);
								DesireDistance(50, thresDistance + 100, 1100);
								Winker_Write(ALL_OFF);
							}
							/*알고리즘 전진*/
							data->imgData.bmission = false;
							sprintf(data->imgData.missionString, "End Overtaking");
							DesireSpeed_Write(BASIC_SPEED);
							state = DONE_O;
							overtake = DONE;
							data->missionData.overtakingFlag = false;
							break;

						default:
							break;
						}
						//usleep(1500000);
						usleep(50000); // 1,500 ms -> 50ms 로 변경, 09/01 AM 00:50 -KDH
					}
					signalLight = READY;
					data->imgData.bmission = false;
					data->imgData.bprintString = false;
				}
			}
		}

		if (signalLight && signalLight != DONE)
		{
			if (1)
			{
				DesireSpeed_Write(0);
				SteeringServo_Write(1500);
				data->imgData.bmission = true;
				data->imgData.bprintString = true;
				data->imgData.bcheckSignalLight = true;
				data->imgData.bprintTire = false;
				data->missionData.signalLightData.state = DETECT_RED;
				sprintf(data->imgData.missionString, "check RED");
				printf("signalLight\n");

				while (data->imgData.bcheckSignalLight)
					usleep(200000); //영상처리에서 일련의 과정이 끝날 때 까지 기다린다.

				data->imgData.bprintTire = true;
				sprintf(data->imgData.missionString, "Distance control");
				DesireSpeed_Write(BASIC_SPEED);

				bool once_back = false;
				while (1) // 신호등 구조물과의 거리를 23,24cm 로 맞추기 위해 전진
				{
					int front_distance = DistanceSensor_cm(1);
					if (front_distance < 23)
					{
						once_back = true;
						DesireSpeed_Write(-20);
					}
					else if (front_distance <= 24)
					{
						DesireSpeed_Write(0);
						break;
					}
					else if (once_back && front_distance > 24)
						DesireSpeed_Write(20);
					usleep(50000);
				}

				if (data->missionData.signalLightData.finalDirection == 1)
				{
					sprintf(data->imgData.missionString, "Turn right");
					printf("\tTurn right\n");
					DesireDistance(40, 1170, 1000);
				}
				else if (data->missionData.signalLightData.finalDirection == -1)
				{
					sprintf(data->imgData.missionString, "Turn left");
					printf("\tTurn left\n");
					DesireDistance(40, 1170, 2000);
				}
				else
				{
					sprintf(data->imgData.missionString, "ERROR");
					printf("\tERROR\n");
					DesireDistance(40, 1150, 1000);
				}

				signalLight = DONE;
				finish = READY;
				data->missionData.ms[7] = signalLight;
				data->missionData.ms[8] = finish;
				data->imgData.bmission = false;
				data->imgData.bprintString = false;
			}
		}

		if (finish && finish != DONE)
		{
			// if(0)
			// {
			// data->missionData.finishData.checkFront = false; /*비활성화*/
			// if (1)											 /*노란색 가로 직선이 일정이하로 떨어지면 입력*/
			// {												 //Encoder 사용해서 일정 직진하면 종료하게 설정
			// 	//끝나고 삐소리
			// 	/*이미 이동 상태*/
			// 	data->missionData.finishData.distEndLine = -1000;
			// 	data->imgData.bmission = true;
			// 	data->imgData.bprintString = true;
			// 	/*box filtering*/
			// 	data->missionData.finishData.checkFront = true;/*전방 노란라인 탐지 활성화*/
			// 	/*encoding을 이용한 전진*/
			// 	//data->missionData.finishData.encodingStart = false;
			// 	/*check front signal waiting*/
			// 	while (data->missionData.finishData.checkFront == true || data->missionData.finishData.distEndLine == -1000)
			// 	{
			// 		usleep(500000);
			// 		/*checkFront 가 false가 되어 종료 됐거나 distEndline값이 무의미하지 않을경우 종료*/
			// 		if (data->missionData.finishData.checkFront == false || data->missionData.finishData.distEndLine > 320)
			// 		{
			// 			sprintf(data->imgData.missionString, "End Check Front");
			// 			break; /*앞에 탐지시 종료*/
			// 		}
			// 		sprintf(data->imgData.missionString, "Check Front");
			// 	}
			// 	/*더이상 확인하지 않도록 종료(double check)*/
			// 	data->missionData.finishData.checkFront = false;
			// 	DesireSpeed_Write(0);
			// 	/*이동 후 종료*/
			// 	Winker_Write(ALL_ON);
			// 	usleep(1000000);
			// 	Winker_Write(ALL_OFF);
			// 	/*밑에 흰색이 하나라도 탐지되는지 확인 후 있다면 정지*/
			// 	///추가필요/////
			// }

			if (1) //무조건 진입
			{

				DesireSpeed_Write(0);
				SteeringServo_Write(1500);
				data->imgData.bmission = true;
				sprintf(data->imgData.missionString, "Finish line check");
				data->imgData.bprintString = true;
				data->imgData.bcheckFinishLine = true;

				DesireSpeed_Write(30);
				while (data->missionData.finish_distance == -1)
				{
					usleep(5000); //5ms
				}
				DesireSpeed_Write(0);
				data->imgData.bcheckFinishLine = false;

				int rest_distance = data->missionData.finish_distance;
				rest_distance -= 4;
				sprintf(data->imgData.missionString, "Finish Driving");
				DesireDistance(40, 500 * (rest_distance / 26.0), 1500); // encoder = 500 -> 26cm로 측정

				printf("finish end\n");
				sprintf(data->imgData.missionString, "All mission complete !");
				buzzer(3, 500000, 500000);

				data->imgData.bauto = false;
				data->imgData.bmission = false;
				data->imgData.bprintString = false;
				finish = DONE;
			}
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

		usleep(50000); //50ms
	}
}

static struct thr_data* pexam_data = NULL;

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

		printf("-- 6_camera_opencv_disp example End --\n");
	}
}

/************************************************/
/*	MAIN	main	MAIN	main				*/
/************************************************/
int main(int argc, char** argv)
{
	struct v4l2* v4l2;
	struct vpe* vpe;
	struct thr_data tdata;
	ptr_data = &tdata;
	int disp_argc = 3;
	char* disp_argv[] = { "dummy", "-s", "4:480x272", "\0" }; // 추후 변경 여부 확인 후 처리..
	int ret = 0;
	memset(tdata.img_data_buf, 0, sizeof(tdata.img_data_buf));

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
	sprintf(tdata.imgData.missionString, "(null)");

	/******************** Control Data ********************/
	tdata.controlData.settingSpeedVal = 40;
	tdata.controlData.desireSpeedVal = 0;
	tdata.controlData.beforeSpeedVal = 0;
	CarLight_Write(0x00);
	tdata.controlData.lightFlag = 0x00;
	CameraXServoControl_Write(1500);
	SteeringServoControl_Write(1500);
	tdata.controlData.steerVal = 1500;
	CameraYServoControl_Write(1660);
	tdata.controlData.cameraY = 1660;
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
	// vpe 구조체 생성 및 초기화
	// vpe input (v4l cameradata)
	vpe->src.width = CAPTURE_IMG_W;
	vpe->src.height = CAPTURE_IMG_H;
	describeFormat(CAPTURE_IMG_FORMAT, &vpe->src);
	// 입력 이미지에 대한 파라미터(사이즈 및 포맷)를 vpe 입력 이미지 멤버에 할당한다.

	// vpe output (disp data)
	vpe->dst.width = VPE_OUTPUT_W;
	vpe->dst.height = VPE_OUTPUT_H;
	describeFormat(VPE_OUTPUT_FORMAT, &vpe->dst);
	// 출력 이미지에 대한 파라미터(사이즈 및 포맷)를 vpe 출력 이미지 멤버에 할당한다.

	vpe->disp = disp_open(disp_argc, disp_argv);
	if (!vpe->disp)
	{
		ERROR("disp open error!");
		vpe_close(vpe);
		return 1;
	}
	// 영상 출력을 위해 vpe의 display 멤버 구조체 초기화

	set_z_order(vpe->disp, vpe->disp->overlay_p.id);
	set_global_alpha(vpe->disp, vpe->disp->overlay_p.id);
	set_pre_multiplied_alpha(vpe->disp, vpe->disp->overlay_p.id);
	alloc_overlay_plane(vpe->disp, OVERLAY_DISP_FORCC, 0, 0, OVERLAY_DISP_W, OVERLAY_DISP_H);
	// z-order, alpha, multiplied-alpha 설정 (overlay를 위한 plane 값 설정)

	//vpe->deint = 0;
	vpe->translen = 1;

	MSG("Input(Camera) = %d x %d (%.4s)\nOutput(LCD) = %d x %d (%.4s)",
		vpe->src.width, vpe->src.height, (char*)&vpe->src.fourcc,
		vpe->dst.width, vpe->dst.height, (char*)&vpe->dst.fourcc);
	// 입출력 이미지의 크기 및 포맷정보 출력
	// 입력 이미지 : 1280x720, Format = UYUV422
	// 출력 이미지 : 320x180. Format = BGR24

	if (vpe->src.height < 0 || vpe->src.width < 0 || vpe->src.fourcc < 0 ||
		vpe->dst.height < 0 || vpe->dst.width < 0 || vpe->dst.fourcc < 0)
	{
		ERROR("Invalid parameters\n");
	}

	v4l2 = v4l2_open(vpe->src.fourcc, vpe->src.width, vpe->src.height);
	// 이미지 캡쳐를 위해 vpe 구조체를 바탕으로 v412 구조체 초기화

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

	ret = pthread_create(&tdata.threads[3], NULL, video_record_thread, &tdata);
	if (ret)
	{
		MSG("Failed creating video_record_thread");
	}
	pthread_detach(tdata.threads[3]);

	/* register signal handler for <CTRL>+C in order to clean up */
	if (signal(SIGINT, signal_handler) == SIG_ERR)
	{
		MSG("could not register signal handler");
		closelog();
		exit(EXIT_FAILURE);
	}
	// signal error 검출

	pause();

	return ret;
}


void DesireDistance(int SettingSpeed, int SettingDistance, int SettingSteering)
{
	ptr_data->controlData.steerVal = SettingSteering;

	if (SettingSpeed < 0) rearLightOnOff(ptr_data->controlData.lightFlag, 1);
	DesiredDistance(SettingSpeed, SettingDistance, SettingSteering);
	if (SettingSpeed < 0) rearLightOnOff(ptr_data->controlData.lightFlag, 0);
}

void SteeringServo_Write(signed short angle)
{
	ptr_data->controlData.steerVal = angle;	//오버레이 연동

	SteeringServoControl_Write(angle);
}