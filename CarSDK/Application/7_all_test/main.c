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

#define CAPTURE_IMG_W       1280
#define CAPTURE_IMG_H       720
#define CAPTURE_IMG_SIZE    (CAPTURE_IMG_W*CAPTURE_IMG_H*2) // YUYU : 16bpp
#define CAPTURE_IMG_FORMAT  "uyvy"

//해상도를 바꾸려면 이부분만 변경하면 됨
#define VPE_OUTPUT_W        640
#define VPE_OUTPUT_H        360

#define VPE_OUTPUT_IMG_SIZE    (VPE_OUTPUT_W*VPE_OUTPUT_H*3)
#define VPE_OUTPUT_FORMAT       "bgr24"

#define OVERLAY_DISP_FORCC      FOURCC('A','R','2','4')
#define OVERLAY_DISP_W          480
#define OVERLAY_DISP_H          272

#define TIME_TEXT_X             350 //320
#define TIME_TEXT_Y             260 //240
#define TIME_TEXT_COLOR         0xffffffff //while

#define FPS_TEXT_X             40 //320
#define FPS_TEXT_Y             260 //240
#define FPS_TEXT_COLOR         0xffffffff //while

enum MissionState {
	NONE,
	READY,
	REMAIN,
	DONE = NONE
};

enum ParkingState {
	NONE_P,
	FIRST_WALL,
	DISTANCE_CHECK,
	SECOND_WALL,
	PARKING_START,
	DONE_P = NONE_P
};

enum OvertakeState {
	NONE_O,
	FRONT_DETECT,
	SIDE_ON,
	SIDE_OFF,
	DONE_O = NONE_O
};

enum CameraVerticalState {
	CAMERA_UP, //장애물 인식을 위해 올린상태
	CAMERA_DOWN //원래 상태 -->이 부분 조정 필요 MS
};
enum DirectionState {
	LEFT,
	RIGHT,
	STOP //앞에 장애물이 있다면 스탑(overtaking이 on일 때만)
};
struct Parking {
	bool frontRight;
	bool rearRight;
	bool on_parkingFlag; // 주차 진행 중을 나타내는 플래그
	bool bparking;	// 주차 중 거리 정보 출력을 위한 변수
	bool verticalFlag; // 수직 주차 활성화를 나타내는 플래그
	bool horizontalFlag; // 수평 주차 활성화를 나타내는 플래그
};

struct Overtaking {
	enum DirectionState headingDirection; //차량의 이동방향 결정
	bool sideSensorFlag; // 차량의 사이드 탐지 활성화 플래그
	enum CameraVerticalState updownCamera; //카메라를 위로 올릴지 말지 결정하는 부분
};

struct ROUNDABOUT {
	bool RAstart; // 분기의 시작을 알리는 변수
	bool RAend; // 분기의 끝을 알리는 변수
};

struct MissionData {
	uint32_t loopTime;	// mission 스레드 루프 시간
	bool on_processing; // 어떠한 미션이 진행 중임을 나타내는 플래그 -> 미션 쓰레드에서 다른 미션을 활성화 시키지 않도록 한다.

	bool btunnel; // 터널 플래그
	struct ROUNDABOUT roundabout; // 회전교차로 플래그
	bool overtakingFlag; // 추월차로 플래그 ->MS 이후 overtaking struct 추가할 것
	struct Parking parkingData; // 주차에 필요한 플래그를 담는 구조체
	struct Overtaking overtakingData;//추월에 필요한 플래그 담는 구조체

};

struct ControlData {
	uint32_t loopTime;	//control 스레드 루프 시간
	bool stopFlag;
	bool steerWrite;
	bool speedWrite;
	int steerVal;
	int cameraY;
	int desireSpeedVal;
	int settingSpeedVal;
	unsigned short lightFlag;
};

struct ImgProcessData {
	bool bcalibration;
	bool btopview;
	bool bauto;
	bool bmission;
	char missionString[20];
	int topMode;
};

struct thr_data {
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
	pthread_t threads[4];
};

static void manualControl(struct ControlData* cdata, char key);

static int allocate_input_buffers(struct thr_data* data)
{
	int i;
	struct vpe* vpe = data->vpe;

	data->input_bufs = calloc(NUMBUF, sizeof(*data->input_bufs));
	for (i = 0; i < NUMBUF; i++) {
		data->input_bufs[i] = alloc_buffer(vpe->disp, vpe->src.fourcc, vpe->src.width, vpe->src.height, false);
	}
	if (!data->input_bufs)
		ERROR("allocating shared buffer failed\n");

	for (i = 0; i < NUMBUF; i++) {
		/** Get DMABUF fd for corresponding buffer object */
		vpe->input_buf_dmafd[i] = omap_bo_dmabuf(data->input_bufs[i]->bo[0]);
		data->input_bufs[i]->fd[0] = vpe->input_buf_dmafd[i];
	}
	return 0;
}

static void free_input_buffers(struct buffer** buffer, uint32_t n, bool bmultiplanar)
{
	uint32_t i;
	for (i = 0; i < n; i++) {
		if (buffer[i]) {
			close(buffer[i]->fd[0]);
			omap_bo_del(buffer[i]->bo[0]);
			if (bmultiplanar) {
				close(buffer[i]->fd[1]);
				omap_bo_del(buffer[i]->bo[1]);
			}
		}
	}
	free(buffer);
}

static void draw_operatingtime(struct display* disp, uint32_t time, uint32_t ctime, uint32_t mtime)
{
	FrameBuffer tmpFrame;
	unsigned char* pbuf[4];
	char strtime[128];
	char strctime[128];
	char strmtime[128];
	char strfps[128];

	memset(strtime, 0, sizeof(strtime));
	memset(strctime, 0, sizeof(strctime));
	memset(strmtime, 0, sizeof(strmtime));
	memset(strfps, 0, sizeof(strfps));

	sprintf(strctime, "c thr : %03d(ms)", ctime);
	sprintf(strmtime, "m thr : %03d(ms)", mtime);
	sprintf(strtime, "i thr : %03d(ms)", time);
	sprintf(strfps, "%4d(fps)", (time == 0) ? 1000 : 1000 / time);


	if (get_framebuf(disp->overlay_p_bo, pbuf) == 0) {
		tmpFrame.buf = pbuf[0];
		tmpFrame.format = draw_get_pixel_foramt(disp->overlay_p_bo->fourcc);//FORMAT_RGB888; //alloc_overlay_plane() -- FOURCC('R','G','2','4');
		tmpFrame.stride = disp->overlay_p_bo->pitches[0];//tmpFrame.width*3;

		drawString(&tmpFrame, strctime, TIME_TEXT_X, TIME_TEXT_Y - 40, 0, TIME_TEXT_COLOR);
		drawString(&tmpFrame, strmtime, TIME_TEXT_X, TIME_TEXT_Y - 20, 0, TIME_TEXT_COLOR);
		drawString(&tmpFrame, strtime, TIME_TEXT_X, TIME_TEXT_Y, 0, TIME_TEXT_COLOR);
		drawString(&tmpFrame, strfps, FPS_TEXT_X, FPS_TEXT_Y, 0, FPS_TEXT_COLOR);

	}
}



/************************************************
*	img_process
*************************************************/
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

		/*******************************************************
		*	 우리가 만든 알고리즘 함수를 넣는 부분.
		********************************************************/

		if (t_data->imgData.bmission)
		{
			displayPrint(srcbuf, VPE_OUTPUT_W, VPE_OUTPUT_H, srcbuf, t_data->imgData.missionString);
		}
		else
		{
			if (t_data->imgData.bcalibration) OpenCV_remap(srcbuf, VPE_OUTPUT_W, VPE_OUTPUT_H, srcbuf, map1, map2);
			if (t_data->imgData.btopview) OpenCV_topview_transform(srcbuf, VPE_OUTPUT_W, VPE_OUTPUT_H, srcbuf, t_data->imgData.topMode);
			if (t_data->imgData.bauto)
		{
			int steerVal = autoSteering(srcbuf, VPE_OUTPUT_W, VPE_OUTPUT_H, srcbuf);
			if (steerVal != 0)
			{
				t_data->controlData.steerVal = 1500 - steerVal;
				t_data->controlData.steerWrite = 1;
			}
		}
			/*MS 추월차로시에 사용*/
			if (t_data->imgData.bcalibration && t_data->missionData.overtakingFlag)//camera 올라간 flag도 필요함
		{
			t_data->missionData.overtakingData.updownCamera = CAMERA_UP;

			//srcbuf를 활용하여 capture한 영상을 변환
		}
		}
		/*******************************************************
		*			 영상처리 종료
		********************************************************/

		memcpy(cam_pbuf[0], srcbuf, VPE_OUTPUT_W * VPE_OUTPUT_H * 3);
		gettimeofday(&et, NULL);
		optime = ((et.tv_sec - st.tv_sec) * 1000) + ((int)et.tv_usec / 1000 - (int)st.tv_usec / 1000);
		draw_operatingtime(disp, optime, t_data->controlData.loopTime, t_data->missionData.loopTime);
	}
}


/************************************************
*	image_process_thread
*************************************************/
void* image_process_thread(void* arg)
{
	struct thr_data* data = (struct thr_data*)arg;
	struct v4l2* v4l2 = data->v4l2;
	struct vpe* vpe = data->vpe;
	struct buffer* capt;
	bool isFirst = true;
	int index;
	int i;
	float map1[VPE_OUTPUT_W * VPE_OUTPUT_H] = { 0, };
	float map2[VPE_OUTPUT_W * VPE_OUTPUT_H] = { 0, };
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

		/*******************************************************
		* 영상처리 시작.
		********************************************************/

		img_process(vpe->disp, capt, data, map1, map2);

		/*******************************************************
		* 영상처리 종료
		********************************************************/
		if (disp_post_vid_buffer(vpe->disp, capt, 0, 0, vpe->dst.width, vpe->dst.height))
		{
			ERROR("Post buffer failed");
			return NULL;
		}
		update_overlay_disp(vpe->disp);
		vpe_output_qbuf(vpe, index);
		index = vpe_input_dqbuf(vpe);
		v4l2_qbuf(v4l2, vpe->input_buf_dmafd[index], index);
	}

	MSG("Ok!");
	return NULL;
}


/************************************************
*	input_thread
*************************************************/
void* input_thread(void* arg)
{
	struct thr_data* data = (struct thr_data*)arg;

	char cmd_input[128];
	char cmd_ready = true;

	while (!data->bstream_start) {
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
			cmd_ready = StandbyInput(cmd_input);     //define in cmd.cpp
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
				if (data->imgData.bcalibration) printf("\t calibration ON\n");
				else printf("\t calibration OFF\n");
			}
			else if (0 == strncmp(cmd_input, "parking", 7))
			{
				int j;
				int c1, c2, c3, c4, c5, c6;
				data->missionData.parkingData.bparking = !data->missionData.parkingData.bparking;
				if (data->missionData.parkingData.bparking) {
					printf("\t parking ON\n");
					for (j = 0; j < 50000; j++)
					{
						c1 = DistanceSensor_cm(1);
						c2 = DistanceSensor_cm(2);
						c3 = DistanceSensor_cm(3);
						c4 = DistanceSensor_cm(4);
						c5 = DistanceSensor_cm(5);
						c6 = DistanceSensor_cm(6);
						printf("1 : %-2d, 2 : %-2d, 3 : %-2d, 4: %-2d, 5 : %-2d, 6 : %-2d \n", c1, c2, c3, c4, c5, c6);
						usleep(500000); // 0.5초 마다 출력
					}
				}
				else printf("\t parking OFF\n");
			}
			else if (0 == strncmp(cmd_input, "auto", 4))
			{
				data->imgData.bauto = !data->imgData.bauto;
				if (data->imgData.bauto) printf("\t auto steering ON\n");
				else printf("\t auto steering OFF\n");
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
					else if(data->imgData.topMode == 2)
					{
						data->imgData.topMode = 3;
						printf("\t topview 3 ON\n");
					}
					else if(data->imgData.topMode == 3)
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
				while (1) {
					on_encoder = EncoderCounter_Read();
					if (on_encoder != 65278) printf("encoder : %-3d\n", on_encoder);
					if (on_encoder >= desire_encoder && on_encoder != 65278) {
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

/************************************************
*	mission_thread
*************************************************/
void* mission_thread(void* arg)
{
	struct thr_data* data = (struct thr_data*)arg;
	struct timeval st, et;

	enum MissionState start = NONE;
	enum MissionState flyover = NONE;
	enum MissionState parking = READY;
	enum MissionState roundabout = NONE;
	enum MissionState tunnel = NONE;
	enum MissionState overtake = READY;
	enum MissionState signalLight = NONE;
	enum MissionState finish = NONE;
	

	//각 미션이 수행되고나면 detect를 하지 않도록 변수설정.

	while (1)
	{
		gettimeofday(&st, NULL);

		if (start)
		{
			if (0)
			{
				data->imgData.bmission = true;
				sprintf(data->imgData.missionString, "start");


				start = DONE;
				data->imgData.bmission = false;
			}
		}

		if (flyover)
		{
			if (0)
			{
				data->imgData.bmission = true;
				sprintf(data->imgData.missionString, "flyover");
				data->missionData.on_processing = true;


				flyover = DONE;
				data->imgData.bmission = false;
			}
		}

		if (parking)
		{
			if (DistanceSensor_cm(2) <= 10) //처음 벽이 감지되었을 경우
			{
				data->imgData.bmission = true;
				sprintf(data->imgData.missionString, "Parking");
				int parking_width = 0;
				enum ParkingState state = FIRST_WALL;
				while (state)	// state == END가 아닌이상 루프 진행
				{
					data->missionData.parkingData.frontRight = (DistanceSensor_cm(2) <= 10) ? true : false;
					data->missionData.parkingData.rearRight = (DistanceSensor_cm(3) <= 10) ? true : false;

					switch (state)
					{
					case FIRST_WALL:
						if (data->missionData.parkingData.frontRight == false) {
							state = DISTANCE_CHECK;
							EncoderCounter_Write(0);
						}
						break;

					case DISTANCE_CHECK:
						/*
						주차 폭에 대한 거리를 측정하기 위해 거리 측정 시작
						*/
						if (EncoderCounter_Read() != 65278)
							parking_width = EncoderCounter_Read();
						if (data->missionData.parkingData.frontRight == true)
						{
							state = SECOND_WALL;
							/*
							거리 측정 종료 -> 측정 거리를 변수에 담는다.
							*/
							printf("Result Width : %d",parking_width);

							if (parking_width <= 45)
								data->missionData.parkingData.verticalFlag = true;
							else
								data->missionData.parkingData.horizontalFlag = true;
						}
						break;

					case SECOND_WALL:
						if (data->missionData.parkingData.rearRight == true)
							state = PARKING_START;
						break;
						// 두번 째 벽에 차량 우측 후방 센서가 걸린 상태이다. -> 수직 또는 수평 주차 진행.

					case PARKING_START:
						data->missionData.on_processing = true;
						data->missionData.parkingData.on_parkingFlag = true;
						/*
						수직 및 수평 주차 구문 추가.
						*/

						if (data->missionData.parkingData.verticalFlag && data->missionData.parkingData.horizontalFlag == false) {
							// 수직 주차 구문
						}
						else  {

						}
						state = DONE;

						if (parking == READY) parking = REMAIN;
						else if (parking == REMAIN) parking = DONE;

						break;

					default:
						break;
					}
					usleep(50000);
				}
				data->imgData.bmission = false;
			}
		}

		if (tunnel)
		{
			if (/*on_tunnel()*/0) {
				data->imgData.bmission = true;
				sprintf(data->imgData.missionString, "tunnel");
				data->missionData.on_processing = true;
				data->missionData.btunnel = true;
				printf("tunnel\n");


				tunnel = DONE;
				data->imgData.bmission = false;
			}
		}

		if (roundabout)
		{
			if (StopLine(4)) {
				data->imgData.bmission = true;
				sprintf(data->imgData.missionString, "roundabout");
				data->missionData.on_processing = true;
				//data->missionData.bround = true;
				printf("roundabout\n");

				roundabout = DONE;
				signalLight = READY;
				data->imgData.bmission = false;
			}
		}

		if (overtake)
		{
			/*MS 분기진입 명령 지시*/
			if (DistanceSensor_cm(1) < 30) //전방 장애물 감지 //주차 상황이 아닐때, 분기진입 가능
			{
				data->imgData.bmission = true;
				sprintf(data->imgData.missionString, "overtake");
				printf("overtake \n");
				bool farFront = false;
				//30넘을 때 control thread 변환을 주어야함 MS
				data->missionData.on_processing = true;
				enum OvertakeState state = FRONT_DETECT;
				data->missionData.overtakingData.headingDirection = STOP;
				bool obstacle = false;
				while (state)
				{
					switch (state)
					{
					case FRONT_DETECT:
						/* 장애물 좌우판단을 위한 카메라 각도조절 */
						data->missionData.overtakingFlag = true;
						data->controlData.cameraY = 1500;
						CameraYServoControl_Write(data->controlData.cameraY);
						data->missionData.overtakingData.updownCamera = CAMERA_UP;
						/* 장애물 좌우 판단 및 비어있는 차선으로 전진하는 코드*/
						if (data->missionData.overtakingData.headingDirection == RIGHT) {
							/*출발*/
							//#TODO 변경예정
							int i_time = 0;
							while (!farFront) {
								/*일정시간마다 거리센서 측정하여 멀어지면 SIDE_ON진행*/
								

								/*특정시간이 지났음에도 멀어지지 않는다면 BACK진행*/
								if (i_time > 5000) {
									break;
									i_time = 0;
								}
							}
							/*전진하는 동안 전방 센서가 30 이상 멀어지면 SIDE_ON으로 진행*/
							if (farFront == true ) { state = SIDE_ON; }

						}
						else if(data->missionData.overtakingData.headingDirection == LEFT) {

							/*전진하는 동안 전방 센서가 30 이상 멀어지면 SIDE_ON으로 진행*/
							state = SIDE_ON;
						}
						else { /*STOP이 유지되는 경우 멈춤*/ }

						break;

					case SIDE_ON:
						data->missionData.overtakingData.updownCamera = CAMERA_DOWN;
						/* 현재 장애물이 어디있느냐에 따라 side 센서(2,3 or 4,5)로 감지하는 코드*/

						/*
							if(오른쪽)		obstacle = (DistanceSensor_cm(2) < 10) ? true : false;
							else if (왼쪽)	obstacle = (DistanceSensor_cm(5) < 10) ? true : false;
						*/

						if (obstacle == false)
						{
							state = SIDE_OFF;
						}
						break;

					case SIDE_OFF:
						/*원래 차선으로 복귀하는 코드*/

						overtake = DONE;
						break;

					default:
						break;
					}
				}
				data->imgData.bmission = false;
			}
		}

		if (signalLight)
		{
			if (roundabout == DONE && StopLine(4))
			{
				data->imgData.bmission = true;
				sprintf(data->imgData.missionString, "signalLight");
				printf("signalLight\n");


				data->imgData.bmission = false;
			}
		}

		if (finish)
		{
			if (0)
			{	
				data->imgData.bmission = true;
				sprintf(data->imgData.missionString, "finish");
				printf("finish\n");


				data->imgData.bmission = false;
			}
		}
		usleep(500000);
		gettimeofday(&et, NULL);
		data->missionData.loopTime = ((et.tv_sec - st.tv_sec) * 1000) + ((int)et.tv_usec / 1000 - (int)st.tv_usec / 1000);
		//시간측정
	}
}


/************************************************
*	control_thread
*************************************************/
void* control_thread(void* arg)
{
	struct thr_data* data = (struct thr_data*)arg;
	struct timeval st, et;

	int currentSpeed;
	double err_P = 0;
	double err_I = 0;
	double err_D = 0;
	double err_B = 0;
	int goal;
	int writeVal;
	bool isStop = 1;

	SpeedPIDProportional_Write(20);
	SpeedPIDIntegral_Write(20);
	SpeedPIDProportional_Write(20);

	while (1)
	{
		gettimeofday(&st, NULL);

		if (data->missionData.parkingData.verticalFlag == 1) {
			DesiredDistance(-30, 500);
			data->missionData.on_processing = 0;
			data->missionData.parkingData.on_parkingFlag = 0;
			data->missionData.parkingData.frontRight = 0;
			data->missionData.parkingData.rearRight = 0;
			data->missionData.parkingData.verticalFlag = 0;
		}

		if (data->missionData.parkingData.horizontalFlag) {
			// 주차 분기가 활성화 되었을 때 실행 할 부분
			// switch case 문 이용하여 차량을 절차적으로 제어한다.

		}

		if (!data->missionData.on_processing)	//미션 수행중이 아닐 때 
		{
			if (data->controlData.steerWrite)
			{
				data->controlData.steerWrite = 0;
				SteeringServoControl_Write(data->controlData.steerVal);
				//usleep(100000); //100ms
			}

			if (data->controlData.stopFlag)
			{
				if (!isStop) DesireSpeed_Write(0);
				isStop = 1;
				err_P = 0;
				err_I = 0;
				err_D = 0;
				err_B = 0;
				//usleep(100000); //100ms
			}
			else if (data->controlData.speedWrite)
			{
				isStop = 0;
				goal = data->controlData.desireSpeedVal;
				currentSpeed = DesireSpeed_Read();
				usleep(10000); //10ms

				if (abs(currentSpeed - goal) < 3)
				{
					data->controlData.speedWrite = 0;
					continue;
				}
				if (currentSpeed < -500 || currentSpeed>500) continue;
				err_P = currentSpeed - goal;
				err_I += err_P;
				err_D = err_B - err_P;
				err_B = err_P;

				writeVal = goal - (err_P * 0.11 + err_I * 0.08 + err_D * 0.15);
				// printf("currentSpeed = %d, writeVal = %d\n", currentSpeed, writeVal);
				// printf("errP = %4.1f, errI = %4.1f, errD = %4.1f\n\n",err_P, err_I, err_D);
				DesireSpeed_Write(writeVal);

				usleep(70000); //70ms
			}
		}

		gettimeofday(&et, NULL);
		data->controlData.loopTime = ((et.tv_sec - st.tv_sec) * 1000) + ((int)et.tv_usec / 1000 - (int)st.tv_usec / 1000);
	}

	return NULL;
}

static struct thr_data* pexam_data = NULL;

void signal_handler(int sig)
{
	if (sig == SIGINT) {
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
	case 'a':	//steering left		: servo 조향값 (2000(좌) ~ 1500(중) ~ 1000(우)
		cdata->steerVal += 50;
		SteeringServoControl_Write(cdata->steerVal);
		printf("angle_steering = %d\n", cdata->steerVal);
		printf("SteeringServoControl_Read() = %d\n", SteeringServoControl_Read());    //default = 1500, 0x5dc
		break;

	case 'd':	//steering right	: servo 조향값 (2000(좌) ~ 1500(중) ~ 1000(우)
		cdata->steerVal -= 50;
		SteeringServoControl_Write(cdata->steerVal);
		printf("angle_steering = %d\n", cdata->steerVal);
		printf("SteeringServoControl_Read() = %d\n", SteeringServoControl_Read());    //default = 1500, 0x5dc
		break;

	case 's':
		cdata->stopFlag = 1;
		break;

	case 'w':	//go forward
		cdata->stopFlag = 0;
		cdata->speedWrite = 1;
		cdata->desireSpeedVal = cdata->settingSpeedVal;
		break;

	case 'x':	//go backward	speed 음수 인가하면 후진.
		cdata->stopFlag = 0;
		cdata->speedWrite = 1;
		cdata->desireSpeedVal = -cdata->settingSpeedVal;
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

	case 'i':	//cam up
		cdata->cameraY -= 50;
		CameraYServoControl_Write(cdata->cameraY);
		printf("cdata->cameraY = %d\n", cdata->cameraY);
		printf("CameraYServoControl_Read() = %d\n", CameraYServoControl_Read());    //default = 1500, 0x5dc
		break;

	case 'k':	//cam down
		cdata->cameraY += 50;
		CameraYServoControl_Write(cdata->cameraY);
		printf("angle_cameraY = %d\n", cdata->cameraY);
		printf("CameraYServoControl_Read() = %d\n", CameraYServoControl_Read());    //default = 1500, 0x5dc
		break;

	case '1':	//speed up		최대 스피드 500
		cdata->settingSpeedVal += 10;
		printf("speed = %d\n", cdata->settingSpeedVal);
		break;

	case '2':	//speed down
		cdata->settingSpeedVal -= 10;
		printf("speed = %d\n", cdata->settingSpeedVal);
		break;

	case 'q':	//Flashing left winker 3 s

		Winker_Write(LEFT_ON);
		usleep(3000000);		// 3 000 000 us
		Winker_Write(ALL_OFF);
		break;

	case 'e':	//Flashing right winker 3 s 

		Winker_Write(RIGHT_ON);
		usleep(3000000);		// 3 000 000 us
		Winker_Write(ALL_OFF);
		break;

	case 'z':	//front lamp on/off
		cdata->lightFlag = cdata->lightFlag ^ 0x01;	// 00000000 ^ 00000001 (XOR)���� : 0����Ʈ�� XOR�����Ѵ�.
		CarLight_Write(cdata->lightFlag);
		break;

	case 'c':	//rear lamp on/off
		cdata->lightFlag = cdata->lightFlag ^ 0x02;	// 00000000 ^ 00000010 (XOR)���� : 1����Ʈ�� XOR�����Ѵ�.
		CarLight_Write(cdata->lightFlag);
		break;

	case ' ':	//alarm 
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


/************************************************
*	main
*************************************************/
int main(int argc, char** argv)
{
	struct v4l2* v4l2;
	struct vpe* vpe;
	struct thr_data tdata;
	int disp_argc = 3;
	char* disp_argv[] = { "dummy", "-s", "4:480x272", "\0" }; // 추후 변경 여부 확인 후 처리..
	int ret = 0;

	printf("-- 7_all_test Start --\n");

	CarControlInit();
	PositionControlOnOff_Write(UNCONTROL); // position controller must be OFF !!!
	SpeedControlOnOff_Write(CONTROL);   // speed controller must be also ON !!!

	tdata.imgData.bcalibration = false;
	tdata.imgData.btopview = true;
	tdata.imgData.topMode = 2;
	tdata.imgData.bauto = true;
	tdata.imgData.bmission = false;
	sprintf(tdata.imgData.missionString, "0");

	tdata.controlData.settingSpeedVal = 30;
	tdata.controlData.desireSpeedVal = 0;
	tdata.controlData.lightFlag = 0x00;
	CameraXServoControl_Write(1500);
	tdata.controlData.steerVal = 1500;
	CameraYServoControl_Write(1660);
	tdata.controlData.cameraY = 1660;
	tdata.controlData.stopFlag = false;
	tdata.controlData.steerWrite = false;
	tdata.controlData.speedWrite = false;

	tdata.missionData.on_processing = false;
	//tdata.missionData.bround = false;
	tdata.missionData.btunnel = false;
	tdata.missionData.parkingData.bparking = false;
	tdata.missionData.parkingData.horizontalFlag = false;
	tdata.missionData.parkingData.verticalFlag = false;
	tdata.missionData.parkingData.on_parkingFlag = false;
	tdata.missionData.parkingData.frontRight = false;
	tdata.missionData.parkingData.rearRight = false;
	tdata.missionData.overtakingFlag = false;
	tdata.missionData.overtakingData.updownCamera = CAMERA_DOWN;
	tdata.missionData.overtakingData.headingDirection = STOP;


	// open vpe
	vpe = vpe_open();
	if (!vpe) {
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
	if (!vpe->disp) {
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

	if (vpe->src.height < 0 || vpe->src.width < 0 || vpe->src.fourcc < 0 || \
		vpe->dst.height < 0 || vpe->dst.width < 0 || vpe->dst.fourcc < 0) {
		ERROR("Invalid parameters\n");
	}

	v4l2 = v4l2_open(vpe->src.fourcc, vpe->src.width, vpe->src.height);
	// 이미지 캡쳐를 위해 vpe 구조체를 바탕으로 v412 구조체 초기화

	if (!v4l2) {
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
	if (ret) {
		MSG("Failed creating image_process_thread");
	}
	pthread_detach(tdata.threads[0]);

	ret = pthread_create(&tdata.threads[1], NULL, mission_thread, &tdata);
	if (ret) {
		MSG("Failed creating mission_thread");
	}
	pthread_detach(tdata.threads[1]);

	ret = pthread_create(&tdata.threads[2], NULL, input_thread, &tdata);
	if (ret) {
		MSG("Failed creating input_thread");
	}
	pthread_detach(tdata.threads[2]);

	ret = pthread_create(&tdata.threads[3], NULL, control_thread, &tdata);
	if (ret) {
		MSG("Failed creating control_thread");
	}
	pthread_detach(tdata.threads[3]);

	/* register signal handler for <CTRL>+C in order to clean up */
	if (signal(SIGINT, signal_handler) == SIG_ERR) {
		MSG("could not register signal handler");
		closelog();
		exit(EXIT_FAILURE);
	}
	// signal error 검출

	pause();

	return ret;
}