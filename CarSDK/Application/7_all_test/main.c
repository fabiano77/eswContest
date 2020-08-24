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

#define TIME_TEXT_X 350			   //320
#define TIME_TEXT_Y 260			   //240
#define TIME_TEXT_COLOR 0xffffffff //while

#define FPS_TEXT_X 40			  //320
#define FPS_TEXT_Y 260			  //240
#define FPS_TEXT_COLOR 0xffffffff //while

/******************** enumerator ********************/
enum MissionState
{
	NONE,
	READY,
	REMAIN,
	DONE = NONE
};

enum ParkingState
{
	NONE_P,
	FIRST_WALL,
	DISTANCE_CHECK,
	SECOND_WALL,
	PARKING_START,
	DONE_P = NONE_P
};

enum HorizentalStep
{
	FINISH,
	FIRST_BACKWARD,
	FIRST_FORWARD,
	SECOND_BACKWARD,
	SECOND_FORWARD
};

enum OvertakeState
{
	NONE_O,
	FRONT_DETECT,
	SIDE_ON,
	SIDE_OFF,
	DONE_O = NONE_O
};

enum CameraVerticalState
{
	CAMERA_UP,	//장애물 인식을 위해 올린상태
	CAMERA_DOWN //원래 상태 -->이 부분 조정 필요 MS
};

enum DirectionState
{
	LEFT,
	RIGHT,
	STOP //앞에 장애물이 있다면 스탑(overtaking이 on일 때만)
};


/******************** mission struct ********************/
struct Parking
{
	bool frontRight;
	bool rearRight;
	bool bparking;		 // 주차 중 거리 정보 출력을 위한 변수
	bool verticalFlag;	 // 수직 주차 활성화를 나타내는 플래그
	bool horizontalFlag; // 수평 주차 활성화를 나타내는 플래그
};

struct Overtaking
{
	bool sideSensorFlag;				   // 차량의 사이드 탐지 활성화 플래그
	enum DirectionState headingDirection;  //차량의 이동방향 결정
	enum CameraVerticalState updownCamera; //카메라를 위로 올릴지 말지 결정하는 부분
};

struct TUNNEL
{
	bool btunnel; // 터널 플래그
	bool Tstart;  // 분기의 시작을 알리는 변수
	bool Tend;	  // 분기의 끝을 알리는 변수
};

struct Finish {
	bool checkFront;//앞의 수직 노란선 파악
	int distEndLine;//결승선까지의 거리
};

/******************** thread struct ********************/
struct MissionData
{
	uint32_t loopTime;	// mission 스레드 루프 시간
	bool broundabout;
	bool overtakingFlag;			  // 추월차로 플래그 ->MS 이후 overtaking struct 추가할 것

	struct TUNNEL tunnel;
	struct Parking parkingData;		  // 주차에 필요한 플래그를 담는 구조체
	struct Overtaking overtakingData; //추월에 필요한 플래그 담는 구조체
	struct Finish finishData;
};

struct ControlData
{
	int steerVal;
	int cameraY;
	int desireSpeedVal;
	int beforeSpeedVal;
	int settingSpeedVal;
	unsigned short lightFlag;
};

struct ImgProcessData
{
	uint32_t loopTime; //img 스레드 루프 시간
	bool bcalibration;
	bool bdebug;
	bool btopview;
	bool bmission;
	bool bauto;
	bool bwhiteLine;
	bool bprintString;
	char missionString[20];
	int topMode;
};

struct thr_data
{
	struct display* disp;
	struct v4l2* v4l2;
	struct vpe* vpe;
	struct buffer** input_bufs;
	struct ControlData controlData;
	struct MissionData missionData;
	struct ImgProcessData imgData;

	int msgq_id;

	bool bfull_screen;
	bool bstream_start;
	pthread_t threads[3];
};

/******************** function ********************/
static void manualControl(struct ControlData* cdata, char key);

static uint32_t timeCheck(struct timeval* tempTime);

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
			debugFiltering(srcbuf, VPE_OUTPUT_W, VPE_OUTPUT_H, srcbuf);
		}

		/* 미션 진행중에 처리하는 영상처리 */
		else if (t_data->imgData.bmission)
		{
			/* 추월차로시에 사용 */
			if (t_data->missionData.overtakingFlag &&
				t_data->missionData.overtakingData.updownCamera == CAMERA_UP)
			{
				usleep(50000);
				/*check를 위한 camera up*/
				bool check_direction;
				check_direction = checkObstacle(srcbuf, VPE_OUTPUT_W, VPE_OUTPUT_H, srcbuf);
				if (check_direction == true)
				{ //true=>left
					t_data->missionData.overtakingData.headingDirection = LEFT;
				}
				else
				{ //false =>right
					t_data->missionData.overtakingData.headingDirection = RIGHT;
				}
				t_data->missionData.overtakingData.updownCamera = CAMERA_DOWN;
				//srcbuf를 활용하여 capture한 영상을 변환
			}

			if (t_data->missionData.tunnel.btunnel)
			{
				if (Tunnel(srcbuf, VPE_OUTPUT_W, VPE_OUTPUT_H, 65))
				{
					t_data->missionData.tunnel.Tstart = true;
					t_data->missionData.tunnel.Tend = true;
				}
				else
				{
					t_data->missionData.tunnel.Tstart = false;
					if (t_data->missionData.tunnel.Tend)
					{
						t_data->missionData.tunnel.Tend = false;
						t_data->missionData.tunnel.btunnel = false;
					}
				}
			}
			if (t_data->missionData.finishData.checkFront == true)
			{
				OpenCV_topview_transform(srcbuf, VPE_OUTPUT_W, VPE_OUTPUT_H, srcbuf, t_data->imgData.topMode);
				t_data->missionData.finishData.distEndLine = checkFront(srcbuf, VPE_OUTPUT_W, VPE_OUTPUT_H, srcbuf);
				/*무의미한 값인 경우 알고리즘에 맞게 steering 진행*/
				if (t_data->missionData.finishData.distEndLine == -1000) {
					int steerVal = autoSteering(srcbuf, VPE_OUTPUT_W, VPE_OUTPUT_H, srcbuf, t_data->imgData.bwhiteLine);
					if (steerVal != 9999)
					{
						t_data->controlData.steerVal = 1500 - steerVal;
						SteeringServoControl_Write(data->controlData.steerVal);

						t_data->controlData.desireSpeedVal = auto_speedMapping(steerVal, 40);
					}
					if (t_data->controlData.desireSpeedVal != t_data->controlData.beforeSpeedVal)
					{
						//이전 속도와 달라졌을 때만 속도값 인가.
						DesireSpeed_Write(t_data->controlData.desireSpeedVal);
						t_data->controlData.beforeSpeedVal = t_data->controlData.desireSpeedVal;
					}
				}
				else {/*거리가 탐지된 경우 영상처리 종료*/
					t_data->missionData.finishData.checkFront = false;
				}
			}
		}

		/* 기본 상태에서 처리되는 영상처리 */
		else
		{
			if (t_data->imgData.bcalibration)
			{
				OpenCV_remap(srcbuf, VPE_OUTPUT_W, VPE_OUTPUT_H, srcbuf, map1, map2);
			}

			if (t_data->imgData.btopview)
			{
				OpenCV_topview_transform(srcbuf, VPE_OUTPUT_W, VPE_OUTPUT_H, srcbuf, t_data->imgData.topMode);
			}

			if (t_data->imgData.bauto)
			{
				int steerVal = autoSteering(srcbuf, VPE_OUTPUT_W, VPE_OUTPUT_H, srcbuf, t_data->imgData.bwhiteLine);
				if (steerVal != 9999)
				{
					t_data->controlData.steerVal = 1500 - steerVal;
					SteeringServoControl_Write(data->controlData.steerVal);

					t_data->controlData.desireSpeedVal = auto_speedMapping(steerVal, 40);
				}
				if (t_data->controlData.desireSpeedVal != t_data->controlData.beforeSpeedVal)
				{
					//이전 속도와 달라졌을 때만 속도값 인가.
					DesireSpeed_Write(t_data->controlData.desireSpeedVal);
					t_data->controlData.beforeSpeedVal = t_data->controlData.desireSpeedVal;
				}
			}
		}


		/********************************************************/
		/*			영상처리 종료								*/
		/********************************************************/

		/* 상단에 문자열(missionStr) 출력 */
		if (t_data->imgData.bprintString)
		{
			displayPrint(srcbuf, VPE_OUTPUT_W, VPE_OUTPUT_H, srcbuf, t_data->imgData.missionString);
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
	OpenCV_calibration(map1, map2, VPE_OUTPUT_W, VPE_OUTPUT_H);
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
	MSG("\n\nInput command:");
	MSG("\t calib : calibration ON/OFF");
	MSG("\t debug : debug ON/OFF");
	MSG("\t top   : top view ON/OFF");
	MSG("\t auto  : auto steering ON/OFF");
	MSG("\t dist  : distance sensor check");
	MSG("\t distc : distance sensor check by -cm-");
	MSG("\t distl : distance sensor check constantly");
	MSG("\t encoder : ");
	MSG("\t parking : ");
	MSG("\n");

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
				if (data->imgData.bcalibration)
					printf("\t calibration ON\n");
				else
					printf("\t calibration OFF\n");
			}
			else if (0 == strncmp(cmd_input, "debug", 5))
			{
				data->imgData.bdebug = !data->imgData.bdebug;
				if (data->imgData.bdebug)
					printf("\t debug ON\n");
				else
					printf("\t debug OFF\n");
			}
			else if (0 == strncmp(cmd_input, "auto", 4))
			{
				data->imgData.bauto = !data->imgData.bauto;
				if (data->imgData.bauto)
					printf("\t auto steering ON\n");
				else
					printf("\t auto steering OFF\n");
			}
			else if (0 == strncmp(cmd_input, "top", 3))
			{
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
			else if (0 == strncmp(cmd_input, "distl", 5))
			{
				int d_data;
				int channel;
				int j;
				printf("channel(1~6) : ");
				scanf("%d", &channel);
				for (j = 0; j < 50000; j++)
				{
					d_data = DistanceSensor(channel);
					printf("channel = %d, distance = 0x%04X(%d) \n", channel, d_data, d_data);
					usleep(300000);
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
				EncoderCounter_Write(init_encoder);
				DesireSpeed_Write(30);
				while (1)
				{
					on_encoder = EncoderCounter_Read();
					if (on_encoder != 65278)
						printf("encoder : %-3d\n", on_encoder);
					if (on_encoder >= desire_encoder && on_encoder != 65278)
					{
						DesireSpeed_Write(0);
						break;
					}
					usleep(100000);
				}
				printf("Test End.\n");
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
	enum MissionState parking = READY;
	enum MissionState roundabout = READY;
	enum MissionState tunnel = NONE;
	enum MissionState overtake = READY;
	enum MissionState signalLight = NONE;
	enum MissionState finish = NONE;

	int i = 0;

	//각 미션이 수행되고나면 detect를 하지 않도록 변수설정.

	while (1)
	{
		data->missionData.loopTime = timeCheck(&time);

		if (start)
		{
			if (0)
			{
				data->imgData.bmission = true;
				data->imgData.bprintString = true;
				sprintf(data->imgData.missionString, "start");

				start = DONE;
				data->imgData.bmission = false;
				data->imgData.bprintString = false;
			}
		}

		if (flyover)
		{
			if (0)
			{
				data->imgData.bmission = true;
				data->imgData.bprintString = true;
				sprintf(data->imgData.missionString, "flyover");

				flyover = DONE;
				data->imgData.bmission = false;
				data->imgData.bprintString = false;
			}
		}

		if (parking)
		{
			if (DistanceSensor_cm(2) <= 20) //처음 벽이 감지되었을 경우
			{
				data->imgData.bprintString = true;
				sprintf(data->imgData.missionString, "Parking");
				int parking_width = 0;
				enum ParkingState state = FIRST_WALL;
				enum HorizentalStep step = FIRST_BACKWARD;

				while (state) // state == END가 아닌이상 루프 진행
				{
					data->missionData.loopTime = timeCheck(&time);
					data->missionData.parkingData.frontRight = (DistanceSensor_cm(2) <= 18) ? true : false;
					data->missionData.parkingData.rearRight = (DistanceSensor_cm(3) <= 18) ? true : false;

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
						if (EncoderCounter_Read() != 65278)
						{
							parking_width = EncoderCounter_Read();
							sprintf(data->imgData.missionString, "parking_width : %d", parking_width);
						}
						if (data->missionData.parkingData.frontRight == true)
						{
							/*
							거리 측정 종료 -> 측정 거리를 변수에 담는다.
							*/
							printf("Result Width : %-3d\n", parking_width);

							if (parking_width <= 700)
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
							// 수직 주차 구문
						}
						else
						{
							DesiredDistance(50, 100, 1500);
							while (data->missionData.parkingData.horizontalFlag)
							{
								data->missionData.loopTime = timeCheck(&time);
								switch (step)
								{
								case FIRST_BACKWARD:
									SteeringServoControl_Write(1000);
									DesireSpeed_Write(-40);
									if (DistanceSensor_cm(4) <= 20)
									{
										DesireSpeed_Write(0);
										SteeringServoControl_Write(1500);
										DesireSpeed_Write(-40);
										if (DistanceSensor_cm(4) <= 7)
										{
											step = FIRST_FORWARD;
											DesireSpeed_Write(0);
										}
									}
									break;

								case FIRST_FORWARD:
									DesiredDistance(50, 700, 1350);
									step = SECOND_BACKWARD;
									break;

								case SECOND_BACKWARD:
									SteeringServoControl_Write(2000);
									DesireSpeed_Write(-40);
									if (DistanceSensor_cm(4) <= 6 && DistanceSensor_cm(3) <= 6 && DistanceSensor_cm(2) <= 6)
									{
										DesireSpeed_Write(0);
										usleep(3000000);
										step = SECOND_FORWARD;
									}
									break;

								case SECOND_FORWARD:
									DesiredDistance(30, 600, 1800);
									buzzer(2, 200000, 200000);
									usleep(3000000);
									step = FINISH;
									break;

								case FINISH:
									data->missionData.parkingData.horizontalFlag = 0;
									break;

								default:
									break;
								}
								usleep(200000);
							}
						}
						state = DONE;

						if (parking == READY)
							parking = REMAIN;
						else if (parking == REMAIN)
							parking = DONE;

						break;

					default:
						break;
					}
					usleep(200000);
				}
				data->imgData.bmission = false;
				data->imgData.bprintString = false;
				if (parking == REMAIN)
				{
					printf("First Parking is Done!\n");
					usleep(10000000);
				}
				if (parking == DONE)
					printf("Second Parking is Dome!\n");
			}
		}

		if (tunnel)
		{
			if (data->missionData.tunnel.Tstart)
			{
				int steerVal;
				data->imgData.bmission = true;
				data->imgData.bprintString = true;

				frontLightOnOff(data->controlData.lightFlag, true);

				while (Tunnel_isEnd(DistanceSensor_cm(2), DistanceSensor_cm(6), DistanceSensor_cm(3), DistanceSensor_cm(5)))
				{
					data->missionData.loopTime = timeCheck(&time);
					steerVal = Tunnel_SteerVal(DistanceSensor_cm(2), DistanceSensor_cm(6));
					SteeringServoControl_Write(steerVal);
				}
				DesireSpeed_Write(0);
				printf("tunnel_OFF\n");

				frontLightOnOff(data->controlData.lightFlag, false);

				data->imgData.bmission = false;
				data->missionData.tunnel.btunnel = false;
				usleep(150000);

				DesireSpeed_Write(40);
				tunnel = DONE;
				data->imgData.bprintString = false;
			}
		}

		if (roundabout)
		{
			if (StopLine(4))
			{
				data->imgData.bwhiteLine = true;
				data->imgData.bprintString = true;
				sprintf(data->imgData.missionString, "roundabout");
				int speed = 30;
				bool delay = false;
				DesireSpeed_Write(0);
				while (1)
				{
					data->missionData.loopTime = timeCheck(&time);
					if (RoundAbout_isStart(DistanceSensor_cm(1)))
					{
						data->missionData.broundabout = true;
						break;
					}
					else
					{
						DesireSpeed_Write(0);
					}
				}
				while (!RoundAbout_isEnd(DistanceSensor_cm(1), DistanceSensor_cm(4)))
				{
					data->missionData.loopTime = timeCheck(&time);
					if (RoundAbout_isDelay(DistanceSensor_cm(1)))
					{
						DesireSpeed_Write(0);
						delay = true;
					}
					else
					{
						if (delay && (speed > 20))
						{
							speed = speed - 5;
							delay = false;
						}
						DesireSpeed_Write(speed);
					}
				}
				DesireSpeed_Write(50);

				printf("ROUNDABOUT_OFF\n");

				data->missionData.broundabout = false;
				roundabout = DONE;
				signalLight = READY;
				data->imgData.bmission = false;
				data->imgData.bprintString = false;
			}
		}

		if (overtake)
		{
			/*MS 분기진입 명령 지시*/
			if (DistanceSensor_cm(1) < 30) //전방 장애물 감지 //주차 상황이 아닐때, 분기진입 가능
			{
				data->imgData.btopview = false; //topview off
				data->imgData.bmission = true;	//영상처리 X
				data->imgData.bprintString = true;
				sprintf(data->imgData.missionString, "overtake");
				printf("overtake \n");
				bool farFront = false;
				enum OvertakeState state = FRONT_DETECT;
				data->missionData.overtakingData.headingDirection = STOP;
				data->missionData.overtakingFlag = true;
				data->imgData.bwhiteLine = true;
				bool obstacle = false;
				int thresDistance = 500;
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
							DesiredDistance(50, thresDistance, 1100);
							Winker_Write(ALL_OFF);
							/*thresDistance이상 가서 전방 거리 재확인*/
							if (DistanceSensor_cm(1) < 30)
							{
								farFront = false;
							}
							else
							{
								farFront = true; /*전방 미탐지*/
							}
							/*전진하는 동안 전방 센서가 30 이상 멀어지면 SIDE_ON으로 진행*/
							if (farFront == true)
							{
								state = SIDE_ON;
								DesireSpeed_Write(50);
							}
							else
							{
								sprintf(data->imgData.missionString, "Detect Error");
								/*정지, 후진 및 방향 전환*/
								DesiredDistance(-50, thresDistance, 1900);
								/*정지 및 방향 전환 명령*/
								data->missionData.overtakingData.headingDirection = LEFT;
							}
						}
						else if (data->missionData.overtakingData.headingDirection == LEFT &&
							data->missionData.overtakingData.updownCamera == CAMERA_DOWN)
						{

							sprintf(data->imgData.missionString, "Left to go");
							/*출발*/
							Winker_Write(LEFT_ON);
							DesiredDistance(50, thresDistance, 1900);
							Winker_Write(ALL_OFF);
							/*thresDistance이상 가서 전방 거리 재확인*/
							if (DistanceSensor_cm(1) < 30)
							{
								farFront = false;
							}
							else
							{
								farFront = true;
							}
							/*전진하는 동안 전방 센서가 30 이상 멀어지면 SIDE_ON으로 진행*/
							if (farFront == true)
							{
								state = SIDE_ON;
								DesireSpeed_Write(50);
							}
							else
							{
								/*정지, 후진 및 방향 전환*/
								sprintf(data->imgData.missionString, "Detect Error");
								DesiredDistance(-50, thresDistance, 1100);
								/*정지 후 방향 전환 명령*/
								data->missionData.overtakingData.headingDirection = RIGHT;
							}
						}
						else
						{ /*STOP이 유지되는 경우 멈춤*/
						}

						break;

					case SIDE_ON:
						sprintf(data->imgData.missionString, "Detect Side");
						/*Auto Steering 동작*/
						data->imgData.bmission = false;
						/* 현재 장애물이 어디있느냐에 따라 side 센서(2,3 or 4,5)로 감지하는 코드*/
						//right
						if (data->missionData.overtakingData.headingDirection == RIGHT)
						{
							/*장애물 통과 확인*/
							if (DistanceSensor_cm(5) < 30 && DistanceSensor_cm(6) < 30)
							{
								obstacle = true;
							}
							else if (obstacle == true)
							{
								/*장애물 통과*/
								if (DistanceSensor_cm(5) < 30 && DistanceSensor_cm(6) > 30)
								{
									obstacle = false;
									state = SIDE_OFF;
								}
							}
						}
						//left
						else if (data->missionData.overtakingData.headingDirection == LEFT)
						{
							/*장애물 통과 확인*/
							if (DistanceSensor_cm(2) < 30 && DistanceSensor_cm(2) < 30)
							{
								obstacle = true;
							}
							else if (obstacle == true)
							{
								/*장애물 통과*/
								if (DistanceSensor_cm(3) < 30 && DistanceSensor_cm(2) > 30)
								{
									obstacle = false;
									state = SIDE_OFF;
								}
							}
						}
						//error and go back step
						else
						{
							state = FRONT_DETECT;
						}
						break;

					case SIDE_OFF:
						sprintf(data->imgData.missionString, "Side OFF");
						/*원래 차선으로 복귀하는 코드*/
						data->imgData.bmission = true; //Auto Steering off
						//right
						if (data->missionData.overtakingData.headingDirection == RIGHT)
						{
							/*복귀 좌회전 방향 설정 및 전진*/
							Winker_Write(LEFT_ON);
							DesiredDistance(50, thresDistance, 1900);
							Winker_Write(ALL_OFF);
						}
						//left
						else if (data->missionData.overtakingData.headingDirection == LEFT)
						{
							/*복귀 우회전 방향 설정*/
							Winker_Write(RIGHT_ON);
							DesiredDistance(50, thresDistance, 1100);
							Winker_Write(ALL_OFF);
						}
						/*알고리즘 전진*/
						data->imgData.bmission = false;
						sprintf(data->imgData.missionString, "End Overtaking");
						DesireSpeed_Write(40);
						state = DONE;
						overtake = DONE;
						data->missionData.overtakingFlag = false;
						break;

					default:
						break;
					}
					usleep(1500000);
				}
				data->imgData.bmission = false;
				data->imgData.bprintString = false;
			}
		}

		if (signalLight)
		{
			if ((roundabout != READY) && StopLine(4))
			{
				data->imgData.bmission = true;
				data->imgData.bprintString = true;
				sprintf(data->imgData.missionString, "signalLight");
				printf("signalLight\n");

				data->imgData.bmission = false;
				data->imgData.bprintString = false;
			}
		}

		if (finish)/*MS*/
		{
			if (0)/*노란색 가로 직선이 일정이하로 떨어지면 입력*/
			{//Encoder 사용해서 일정 직진하면 종료하게 설정
				//끝나고 삐소리 
				data->missionData.finishData.distEndLine = -1000;
				data->imgData.bmission = true;
				data->imgData.bprintString = true;
				/*box filtering*/
				data->missionData.finishData.checkFront = true;
				/*encoding을 이용한 전진*/
				data->missionData.finishData.encodingStart = false;
				/*check front signal waiting*/
				while (data->missionData.finishData.checkFront == true || data->missionData.finishData.distEndLine == -1000) {
					usleep(500000);
					/*checkFront 가 false가 되어 종료 됐거나 distEndline값이 무의미하지 않을경우 종료*/
					if (data->missionData.finishData.checkFront == false || data->missionData.finishData.distEndLine != -1000) {
						sprintf(data->imgData.missionString, "End Check Front");
						printf("need to go %d", (360 - data->missionData.finishData.distEndLine) * 6);
						break; /*앞에 탐지시 종료*/
					}
					sprintf(data->imgData.missionString, "Check Front");
				}
				/*더이상 확인하지 않도록 종료(double check)*/
				data->missionData.finishData.checkFront = false;

				/*현재 주행상태 유지 및 이동*/
				sprintf(data->imgData.missionString, "Finish is comming");
				int dist_go = data->missionData.finishData.distEndLine;
				DesiredDistance(40, (360 - dist_go) * 6, 1500);
				/*이동 후 종료*/

				/*밑에 흰색이 하나라도 탐지되는지 확인 후 있다면 정지*/
				///추가필요/////


				sprintf(data->imgData.missionString, "Finish Driving");
				printf("finish\n");

				data->imgData.bmission = false;
				data->imgData.bprintString = false;
			}
		}

		usleep(200000);
		data->missionData.loopTime = timeCheck(&time);
		//시간측정
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

void manualControl(struct ControlData* cdata, char key)
{
	int i;
	switch (key)
	{
	case 'a': //steering left		: servo 조향값 (2000(좌) ~ 1500(중) ~ 1000(우)
		cdata->steerVal += 50;
		SteeringServoControl_Write(cdata->steerVal);
		printf("angle_steering = %d\n", cdata->steerVal);
		printf("SteeringServoControl_Read() = %d\n", SteeringServoControl_Read()); //default = 1500, 0x5dc
		break;

	case 'd': //steering right	: servo 조향값 (2000(좌) ~ 1500(중) ~ 1000(우)
		cdata->steerVal -= 50;
		SteeringServoControl_Write(cdata->steerVal);
		printf("angle_steering = %d\n", cdata->steerVal);
		printf("SteeringServoControl_Read() = %d\n", SteeringServoControl_Read()); //default = 1500, 0x5dc
		break;

	case 's':
		DesireSpeed_Write(0);
		break;

	case 'w': //go forward

		cdata->desireSpeedVal = cdata->settingSpeedVal;
		DesireSpeed_Write(cdata->desireSpeedVal);
		break;

	case 'x': //go backward	speed 음수 인가하면 후진.

		cdata->desireSpeedVal = -cdata->settingSpeedVal;
		DesireSpeed_Write(cdata->desireSpeedVal);
		break;

		// case 'j':	//cam left
		// 	angle_cameraX += 50;
		// 	CameraXServoControl_Write(angle_cameraX);
		// 	printf("angle_cameraX = %d\n", angle_cameraX);
		// 	printf("CameraXServoControl_Read() = %d\n", CameraXServoControl_Read());    //default = 1500, 0x5dc
		// 	break;

		// case 'l':	//cam right
		// 	angle_cameraX -= 50;
		// 	CameraXServoControl_Write(angle_cameraX);
		// 	printf("angle_cameraX = %d\n", angle_cameraX);
		// 	printf("CameraXServoControl_Read() = %d\n", CameraXServoControl_Read());    //default = 1500, 0x5dc
		// 	break;

	case 'i': //cam up
		cdata->cameraY -= 50;
		CameraYServoControl_Write(cdata->cameraY);
		printf("cdata->cameraY = %d\n", cdata->cameraY);
		printf("CameraYServoControl_Read() = %d\n", CameraYServoControl_Read()); //default = 1500, 0x5dc
		break;

	case 'k': //cam down
		cdata->cameraY += 50;
		CameraYServoControl_Write(cdata->cameraY);
		printf("angle_cameraY = %d\n", cdata->cameraY);
		printf("CameraYServoControl_Read() = %d\n", CameraYServoControl_Read()); //default = 1500, 0x5dc
		break;

	case '1': //speed up		최대 스피드 500
		cdata->settingSpeedVal += 10;
		printf("speed = %d\n", cdata->settingSpeedVal);
		break;

	case '2': //speed down
		cdata->settingSpeedVal -= 10;
		printf("speed = %d\n", cdata->settingSpeedVal);
		break;

	case 'q': //Flashing left winker 3 s

		Winker_Write(LEFT_ON);
		usleep(3000000); // 3 000 000 us
		Winker_Write(ALL_OFF);
		break;

	case 'e': //Flashing right winker 3 s

		Winker_Write(RIGHT_ON);
		usleep(3000000); // 3 000 000 us
		Winker_Write(ALL_OFF);
		break;

	case 'z':										//front lamp on/off
		cdata->lightFlag = cdata->lightFlag ^ 0x01; // 00000000 ^ 00000001 (XOR)���� : 0����Ʈ�� XOR�����Ѵ�.
		CarLight_Write(cdata->lightFlag);
		break;

	case 'c':										//rear lamp on/off
		cdata->lightFlag = cdata->lightFlag ^ 0x02; // 00000000 ^ 00000010 (XOR)���� : 1����Ʈ�� XOR�����Ѵ�.
		CarLight_Write(cdata->lightFlag);
		break;

	case ' ': //alarm
		for (i = 0; i < 2; i++)
		{
			Alarm_Write(ON);
			usleep(200000);
			Alarm_Write(OFF);
			usleep(200000);
		}
		break;

	case '0':
		SpeedPIDProportional_Write(40);
		SpeedPIDIntegral_Write(40);
		SpeedPIDProportional_Write(40);
		break;
	case '\n':
		break;

	default:
		printf("wrong key input.\n");
		break;
	}
}

uint32_t timeCheck(struct timeval* tempTime)
{
	struct timeval prevTime = *tempTime;
	struct timeval nowTime;
	gettimeofday(&nowTime, NULL);

	uint32_t retVal = ((nowTime.tv_sec - prevTime.tv_sec) * 1000) + ((int)nowTime.tv_usec / 1000 - (int)prevTime.tv_usec / 1000);
	if (*tempTime.tv_sec == 0) retVal = 0;

	*tempTime = nowTime;

	return retVal;
}


/************************************************/
/*	MAIN	main	MAIN	main				*/
/************************************************/
int main(int argc, char** argv)
{
	struct v4l2* v4l2;
	struct vpe* vpe;
	struct thr_data tdata;
	int disp_argc = 3;
	char* disp_argv[] = { "dummy", "-s", "4:480x272", "\0" }; // 추후 변경 여부 확인 후 처리..
	int ret = 0;

	printf("------ main start ------\n");

	CarControlInit();
	PositionControlOnOff_Write(UNCONTROL); // position controller must be OFF !!!
	SpeedControlOnOff_Write(CONTROL);	   // speed controller must be also ON !!!

	DesireSpeed_Write(0);
	SpeedPIDProportional_Write(40);
	SpeedPIDIntegral_Write(40);
	SpeedPIDProportional_Write(40);

	/******************** imgProcess Data ********************/
	tdata.imgData.bcalibration = false;
	tdata.imgData.bdebug = false;
	tdata.imgData.btopview = true;
	tdata.imgData.topMode = 3;
	tdata.imgData.bauto = false;
	tdata.imgData.bmission = false;
	tdata.imgData.bwhiteLine = false;
	tdata.imgData.bprintString = false;
	sprintf(tdata.imgData.missionString, "(null)");

	/******************** Control Data ********************/
	tdata.controlData.settingSpeedVal = 40;
	tdata.controlData.desireSpeedVal = 0;
	tdata.controlData.beforeSpeedVal = 0;
	tdata.controlData.lightFlag = 0x00;
	CameraXServoControl_Write(1500);
	tdata.controlData.steerVal = 1500;
	CameraYServoControl_Write(1660);
	tdata.controlData.cameraY = 1660;

	/******************** Mission Data ********************/
	tdata.missionData.tunnel.btunnel = true;
	tdata.missionData.tunnel.Tstart = false;
	tdata.missionData.tunnel.Tend = false;
	tdata.missionData.broundabout = false;
	tdata.missionData.parkingData.bparking = false;
	tdata.missionData.parkingData.horizontalFlag = false;
	tdata.missionData.parkingData.verticalFlag = false;
	tdata.missionData.parkingData.frontRight = false;
	tdata.missionData.parkingData.rearRight = false;
	tdata.missionData.overtakingFlag = false;
	tdata.missionData.overtakingData.updownCamera = CAMERA_DOWN;
	tdata.missionData.overtakingData.headingDirection = STOP;


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