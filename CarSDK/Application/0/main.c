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

#define CAPTURE_IMG_W       1280
#define CAPTURE_IMG_H       720
#define CAPTURE_IMG_SIZE    (CAPTURE_IMG_W*CAPTURE_IMG_H*2) // YUYU : 16bpp
#define CAPTURE_IMG_FORMAT  "uyvy"

//해상도를 바꾸려면 이부분만 변경하면 됨. 현재 calib지원 해상도는, 1280x720, 640x360, 320x180
#define VPE_OUTPUT_W        640
#define VPE_OUTPUT_H        360

#define VPE_OUTPUT_IMG_SIZE    (VPE_OUTPUT_W*VPE_OUTPUT_H*3)
#define VPE_OUTPUT_FORMAT       "bgr24"

#define OVERLAY_DISP_FORCC      FOURCC('A','R','2','4')
#define OVERLAY_DISP_W          480
#define OVERLAY_DISP_H          272

#define TIME_TEXT_X             385 //320
#define TIME_TEXT_Y             260 //240
#define TIME_TEXT_COLOR         0xffffffff //while

#define FPS_TEXT_X             40 //320
#define FPS_TEXT_Y             260 //240
#define FPS_TEXT_COLOR         0xffffffff //while

#define DUMP_MSGQ_KEY           1020
#define DUMP_MSGQ_MSG_TYPE      0x02

typedef enum {
	DUMP_NONE,
	DUMP_CMD,
	DUMP_READY,
	DUMP_WRITE_TO_FILE,
	DUMP_DONE
}DumpState;

typedef struct _DumpMsg {
	long type;
	int  state_msg;
}DumpMsg;

struct ControlData {
	bool stopFlag;
	bool steerWrite;
	bool speedWrite;
	int steerVal;
	int cameraY;
	int desireSpeedVal;
	int settingSpeedVal;
	unsigned short lightFlag;
};

struct thr_data {
	struct display* disp;
	struct v4l2* v4l2;
	struct vpe* vpe;
	struct buffer** input_bufs;
	struct ControlData controlData;
	// struct SensorData sensorData;

	DumpState dump_state;
	unsigned char dump_img_data[VPE_OUTPUT_IMG_SIZE];

	int msgq_id;
	bool bcalibration;
	bool btopview;
	bool bauto;
	bool btrack;
	int topMode;
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

static void draw_operatingtime(struct display* disp, uint32_t time)
{
	FrameBuffer tmpFrame;
	unsigned char* pbuf[4];
	char strtime[128];
	char strfps[128];

	memset(strtime, 0, sizeof(strtime));
	memset(strfps, 0, sizeof(strtime));

	sprintf(strtime, "%03d(ms)", time);
	sprintf(strfps, "%4d(fps)", (time == 0) ? 1000 : 1000 / time);


	if (get_framebuf(disp->overlay_p_bo, pbuf) == 0) {
		tmpFrame.buf = pbuf[0];
		tmpFrame.format = draw_get_pixel_foramt(disp->overlay_p_bo->fourcc);//FORMAT_RGB888; //alloc_overlay_plane() -- FOURCC('R','G','2','4');
		tmpFrame.stride = disp->overlay_p_bo->pitches[0];//tmpFrame.width*3;

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

		if (1) OpenCV_remap(srcbuf, VPE_OUTPUT_W, VPE_OUTPUT_H, srcbuf, map1, map2);
		if (1) // tracking Object
		{
			int steerVal = 0;
			int speedVal = 0;
			int *ptrsteerVal = &steerVal;
			int *ptrspeedVal = &speedVal;
			tracking(srcbuf, VPE_OUTPUT_W, VPE_OUTPUT_H, srcbuf, ptrsteerVal, ptrspeedVal);
			printf("1");
			t_data->controlData.steerVal = steerVal;
			printf("2");
			t_data->controlData.steerWrite = 1;
			printf("3");
			t_data->controlData.desireSpeedVal = speedVal;
			printf("4");
			t_data->controlData.speedWrite = 1;
			printf("5");
		}


		/*******************************************************
		*			 영상처리 종료
		********************************************************/

		memcpy(cam_pbuf[0], srcbuf, VPE_OUTPUT_W * VPE_OUTPUT_H * 3);
		gettimeofday(&et, NULL);
		optime = ((et.tv_sec - st.tv_sec) * 1000) + ((int)et.tv_usec / 1000 - (int)st.tv_usec / 1000);
		draw_operatingtime(disp, optime);
	}
}

void* capture_dump_thread(void* arg)
{
	struct thr_data* data = (struct thr_data*)arg;
	FILE* fp;
	char file[50];
	struct timeval timestamp;
	struct tm* today;
	DumpMsg dumpmsg;

	while (1)
	{
		if (msgrcv(data->msgq_id, &dumpmsg, sizeof(DumpMsg) - sizeof(long), DUMP_MSGQ_MSG_TYPE, 0) >= 0)
		{
			switch (dumpmsg.state_msg)
			{
			case DUMP_CMD:
				gettimeofday(&timestamp, NULL);
				today = localtime(&timestamp.tv_sec);
				sprintf(file, "dump_%04d%02d%02d_%02d%02d%02d.%s", today->tm_year + 1900, today->tm_mon + 1, today->tm_mday, today->tm_hour, today->tm_min, today->tm_sec, VPE_OUTPUT_FORMAT);
				data->dump_state = DUMP_READY;
				MSG("file name:%s", file);
				break;

			case DUMP_WRITE_TO_FILE:
				if ((fp = fopen(file, "w+")) == NULL)
				{
					ERROR("Fail to fopen");
				}
				else
				{
					fwrite(data->dump_img_data, VPE_OUTPUT_IMG_SIZE, 1, fp);
				}
				fclose(fp);
				data->dump_state = DUMP_DONE;
				break;

			default:
				MSG("dump msg wrong (%d)", dumpmsg.state_msg);
				break;
			}
		}
	}

	return NULL;
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
		if (data->dump_state == DUMP_READY)
		{
			DumpMsg dumpmsg;
			unsigned char* pbuf[4];

			if (get_framebuf(capt, pbuf) == 0)
			{
				switch (capt->fourcc)
				{
				case FOURCC('Y', 'U', 'Y', 'V'):
				case FOURCC('B', 'G', 'R', '3'):
					memcpy(data->dump_img_data, pbuf[0], VPE_OUTPUT_IMG_SIZE);
					break;
				case FOURCC('N', 'V', '1', '2'):
					memcpy(data->dump_img_data, pbuf[0], VPE_OUTPUT_W * VPE_OUTPUT_H); // y data
					memcpy(data->dump_img_data + VPE_OUTPUT_W * VPE_OUTPUT_H, pbuf[1], VPE_OUTPUT_W * VPE_OUTPUT_H / 2); // uv data
					break;
				default:
					MSG("DUMP.. not yet support format : %.4s\n", (char*)&capt->fourcc);
					break;
				}
			}
			else
			{
				MSG("dump capture buf fail !");
			}
			// 이미지 포맷에 따라서 이미지를 YUYV 포맷으로 변환한다.

			dumpmsg.type = DUMP_MSGQ_MSG_TYPE;
			dumpmsg.state_msg = DUMP_WRITE_TO_FILE;
			data->dump_state = DUMP_WRITE_TO_FILE;
			if (-1 == msgsnd(data->msgq_id, &dumpmsg, sizeof(DumpMsg) - sizeof(long), 0))
			{
				MSG("state:%d, msg send fail\n", dumpmsg.state_msg);
			}
		}
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
	MSG("\t dump  : display image(%s, %dx%d) dump", VPE_OUTPUT_FORMAT, VPE_OUTPUT_W, VPE_OUTPUT_H);
	MSG("\t dist  : distance sensor check");
	MSG("\t distc : distance sensor check by -cm-");
	MSG("\t distloop : distance sensor check constantly");
	MSG("\t calib : calibration ON/OFF");
	MSG("\t top   : top view ON/OFF");
	MSG("\t auto  : auto steering ON/OFF");
	MSG("\t track : tracking object ON/OFF");
	MSG("\n");

	while (1)
	{
		printf("input : ");
		if (cmd_ready == true)
		{
			/*standby to input command */
			cmd_ready = StandbyInput(cmd_input);     //define in cmd.cpp
		}
		else
		{
			if (0 == strncmp(cmd_input, "dump", 4))
			{
				DumpMsg dumpmsg;
				dumpmsg.type = DUMP_MSGQ_MSG_TYPE;
				dumpmsg.state_msg = DUMP_CMD;
				data->dump_state = DUMP_CMD;
				MSG("image dump start");
				if (-1 == msgsnd(data->msgq_id, &dumpmsg, sizeof(DumpMsg) - sizeof(long), 0))
				{
					printf("dump cmd msg send fail\n");
				}

				while (data->dump_state != DUMP_DONE)
				{
					usleep(5 * 1000);
				}
				data->dump_state = DUMP_NONE;
				MSG("image dump done");
			}
			else if (0 == strncmp(cmd_input, "calib", 5))
			{
				data->bcalibration = !data->bcalibration;
				if (data->bcalibration) printf("\t calibration ON\n");
				else printf("\t calibration OFF\n");
			}
			else if (0 == strncmp(cmd_input, "auto", 4))
			{
				data->bauto = !data->bauto;
				if (data->bauto) printf("\t auto steering ON\n");
				else printf("\t auto steering OFF\n");
			}
			else if (0 == strncmp(cmd_input, "track", 5))
			{
				data->btrack = !data->btrack;
				if (data->btrack) printf("\t tracking object ON\n");
				else printf("\t tracking object OFF\n");
			}
			else if (0 == strncmp(cmd_input, "top", 3))
			{
				if (!data->btopview)
				{
					data->btopview = !data->btopview;
					printf("\t topview 1 ON\n");
				}
				else
				{
					if (data->topMode == 1)
					{
						data->topMode = 2;
						printf("\t topview 2 ON\n");
					}
					else
					{
						data->topMode = 1;
						data->btopview = !data->btopview;
						printf("\t topview OFF\n");
					}
				}
			}
			else if (strlen(cmd_input) == 1)
			{
				manualControl(&(data->controlData), cmd_input[0]);
			}
			else if (0 == strncmp(cmd_input, "distloop", 8))
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
*	control_thread
*************************************************/
void* control_thread(void* arg)
{
	struct thr_data* data = (struct thr_data*)arg;

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
		if (data->controlData.steerWrite == 1)
		{
			data->controlData.steerWrite = 0;
			SteeringServoControl_Write(data->controlData.steerVal);
			//usleep(100000); //100ms
		}

		if (data->controlData.stopFlag == 1)
		{
			if (!isStop) DesireSpeed_Write(0);
			isStop = 1;
			err_P = 0;
			err_I = 0;
			err_D = 0;
			err_B = 0;
			//usleep(100000); //100ms
		}

		if (data->controlData.speedWrite == 1)
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

	case 's':	//stop
		// DesireSpeed_Write(0);
		// printf("DC motor stop\n");
		cdata->stopFlag = 1;
		break;

	case 'w':	//go forward
		cdata->stopFlag = 0;
		cdata->speedWrite = 1;
		cdata->desireSpeedVal = cdata->settingSpeedVal;
		// DesireSpeed_Write(cdata->desireSpeedVal);
		// usleep(100000);	//1 000 000 us
		// printf("cdata->desireSpeedVal = %d\n", cdata->desireSpeedVal);
		// printf("DesireSpeed_Read() = %d\n", DesireSpeed_Read());
		break;

	case 'x':	//go backward	speed 음수 인가하면 후진.
		cdata->stopFlag = 0;
		cdata->speedWrite = 1;
		cdata->desireSpeedVal = -cdata->settingSpeedVal;
		// DesireSpeed_Write(0 - cdata->desireSpeedVal);
		// usleep(100000);	//1 000 000 us
		// printf("DesireSpeed_Read() = %d\n", DesireSpeed_Read());
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
	CameraYServoControl_Write(1660);
	CameraXServoControl_Write(1500);
	PositionControlOnOff_Write(UNCONTROL); // position controller must be OFF !!!
	SpeedControlOnOff_Write(CONTROL);   // speed controller must be also ON !!!
	tdata.bcalibration = true;
	tdata.btopview = true;
	tdata.bauto = true;
	tdata.btrack = true;
	tdata.topMode = 2;
	tdata.controlData.settingSpeedVal = 30;
	tdata.controlData.desireSpeedVal = 0;
	tdata.controlData.lightFlag = 0x00;
	tdata.controlData.steerVal = 1500;
	tdata.controlData.cameraY = 1660;
	tdata.controlData.stopFlag = 0;
	tdata.controlData.steerWrite = 0;
	tdata.controlData.speedWrite = 0;

	//캘리브레이션 활성화 상태로 동작.
	tdata.dump_state = DUMP_NONE;
	// Dump State를 키 입력 대기 상태로 초기화
	memset(tdata.dump_img_data, 0, sizeof(tdata.dump_img_data));
	// dump_img_data에 Dump 이미지 공간 할당 (1280x720x2)

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

	if (-1 == (tdata.msgq_id = msgget((key_t)DUMP_MSGQ_KEY, IPC_CREAT | 0666))) {
		fprintf(stderr, "%s msg create fail!!!\n", __func__);
		return -1;
	}

	pexam_data = &tdata;

	ret = pthread_create(&tdata.threads[0], NULL, image_process_thread, &tdata);
	if (ret) {
		MSG("Failed creating capture thread");
	}
	pthread_detach(tdata.threads[0]);

	ret = pthread_create(&tdata.threads[1], NULL, capture_dump_thread, &tdata);
	if (ret) {
		MSG("Failed creating capture dump thread");
	}
	pthread_detach(tdata.threads[1]);

	ret = pthread_create(&tdata.threads[2], NULL, input_thread, &tdata);
	if (ret) {
		MSG("Failed creating input thread");
	}
	pthread_detach(tdata.threads[2]);

	ret = pthread_create(&tdata.threads[3], NULL, control_thread, &tdata);
	if (ret) {
		MSG("Failed creating capture dump thread");
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