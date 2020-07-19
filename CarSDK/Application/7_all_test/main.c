#include <signal.h>
#include <pthread.h>
#include <sys/ipc.h>
#include <sys/msg.h>
#include <termios.h>
#include <sys/time.h>
#include <errno.h>
#include <syslog.h>

#include "util.h"

#include "display-kms.h"
#include "v4l2.h"
#include "vpe-common.h"
#include "drawing.h"
#include "input_cmd.h"
#include "exam_cv.h"
#include "car_lib.h"


#define CAPTURE_IMG_W       1280
#define CAPTURE_IMG_H       720
#define CAPTURE_IMG_SIZE    (CAPTURE_IMG_W*CAPTURE_IMG_H*2) // YUYU : 16bpp
#define CAPTURE_IMG_FORMAT  "uyvy"

#define VPE_OUTPUT_W        320
#define VPE_OUTPUT_H        180

// display output & dump  format: NV12, w:320, h:180
//#define VPE_OUTPUT_IMG_SIZE    (VPE_OUTPUT_W*VPE_OUTPUT_H*3/2) // NV12 : 12bpp
//#define VPE_OUTPUT_FORMAT       "nv12"

// display output & dump  format: yuyv, w:320, h:180
//#define VPE_OUTPUT_IMG_SIZE    (VPE_OUTPUT_W*VPE_OUTPUT_H*2)
//#define VPE_OUTPUT_FORMAT       "yuyv"

#define VPE_OUTPUT_IMG_SIZE    (VPE_OUTPUT_W*VPE_OUTPUT_H*3)
#define VPE_OUTPUT_FORMAT       "bgr24"

#define OVERLAY_DISP_FORCC      FOURCC('A','R','2','4')
#define OVERLAY_DISP_W          480
#define OVERLAY_DISP_H          272

#define TIME_TEXT_X             385 //320
#define TIME_TEXT_Y             260 //240
#define TIME_TEXT_COLOR         0xffffffff //while

#define DUMP_MSGQ_KEY           1020
#define DUMP_MSGQ_MSG_TYPE      0x02

typedef enum {
	DUMP_NONE,
	DUMP_CMD,
	DUMP_READY,
	DUMP_WRITE_TO_FILE,
	DUMP_DONE
}DumpState;

typedef enum {
	NONE,
	VERTICAL,
	HORIZONTAL
}ParkingState;

typedef enum {
	NONE,
	RED,
	YELLOW,
	GREEN_LEFT,
	GREEN_RIGHT
}SignalLight;

typedef struct _DumpMsg {
	long type;
	int  state_msg;
}DumpMsg;

struct ControlData {
	SignalLight signal;
	bool stopFlag;
	int steerVal;
	int desireSpeedVal;
	int currentSpeedVal;
};

struct SensorData {
	ParkingState parkingState;
	int distance[6];
	int lineSensor[7];
};

struct thr_data {
	struct display* disp;
	struct v4l2* v4l2;
	struct vpe* vpe;
	struct buffer** input_bufs;
	struct ControlData controlData;
	struct SensorData sensorData;

	DumpState dump_state;
	unsigned char dump_img_data[VPE_OUTPUT_IMG_SIZE];

	int msgq_id;
	bool bfull_screen;
	bool bstream_start;
	pthread_t threads[3];
};

/**
  * @brief  Alloc vpe input buffer and a new buffer object
  * @param  data: pointer to parameter of thr_data
  * @retval none
  */
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

/**
  * @brief  Free vpe input buffer and destroy a buffer object
  * @param  buffer: pointer to parameter of buffer object
				  n : count of buffer object
				  bmultiplanar : multipanar value of buffer object
  * @retval none
  */
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

/**
  * @brief  Draw operating time to overlay buffer.
  * @param  disp: pointer to parameter of struct display
				  time : operate time (ms)
  * @retval none
  */

  // characteristic : 연산에 소요된 시간을 이미지에 출력한다.
  // precondition : none
  // postcondition : 연산에 소요된 시간을 disp에 표시한다.

static void draw_operatingtime(struct display* disp, uint32_t time)
{
	FrameBuffer tmpFrame;
	unsigned char* pbuf[4];
	char strtime[128];

	memset(strtime, 0, sizeof(strtime));

	sprintf(strtime, "%03d(ms)", time);

	if (get_framebuf(disp->overlay_p_bo, pbuf) == 0) {
		tmpFrame.buf = pbuf[0];
		tmpFrame.format = draw_get_pixel_foramt(disp->overlay_p_bo->fourcc);//FORMAT_RGB888; //alloc_overlay_plane() -- FOURCC('R','G','2','4');
		tmpFrame.stride = disp->overlay_p_bo->pitches[0];//tmpFrame.width*3;

		drawString(&tmpFrame, strtime, TIME_TEXT_X, TIME_TEXT_Y, 0, TIME_TEXT_COLOR);
	}
}

/**
  * @brief  Handle houht transform with opencv api
  * @param  disp: pointer to parameter of struct display
				 cambuf: vpe output buffer that converted capture image
  * @retval none
  */


	//우리가 자체 제작할 영상처리 알고리즘
static void img_process(struct display* disp, struct buffer* cambuf)
{
	unsigned char srcbuf[VPE_OUTPUT_W * VPE_OUTPUT_H * 3];
	// 이미지를 나타내는 배열
	uint32_t optime;
	struct timeval st, et;

	unsigned char* cam_pbuf[4];
	if (get_framebuf(cambuf, cam_pbuf) == 0)
	{
		memcpy(srcbuf, cam_pbuf[0], VPE_OUTPUT_W * VPE_OUTPUT_H * 3);
		//scrbuf 에 입력영상을 복사한다.
		//cam_pbuf는 img_process_thread에서 매개변수로 전달한 영상과 동기화된다.
		gettimeofday(&st, NULL);

		/*******************************************************
		* 우리가 만든 알고리즘 함수를 넣는 부분.
		********************************************************/

		/*****pseudo code*****
		
		캘리브레이션func();

		stop = 우선정지검출func();
		if( stop ) stopFlag = 1;
		else
		{
			steer = 조향값검출func();
		}

		**********************
		*/

		OpenCV_hough_transform(srcbuf, VPE_OUTPUT_W, VPE_OUTPUT_H, cam_pbuf[0], VPE_OUTPUT_W, VPE_OUTPUT_H);



		gettimeofday(&et, NULL);
		optime = ((et.tv_sec - st.tv_sec) * 1000) + ((int)et.tv_usec / 1000 - (int)st.tv_usec / 1000);
		draw_operatingtime(disp, optime);
	}
}

/**
  * @brief  Camera capture, capture image covert by VPE and display after sobel edge
  * @param  arg: pointer to parameter of thr_data
  * @retval none
  */
  // characteristic : Thread 함수로 동작한다. capture된 이미지를 대상으로 허프 변환을 실시하고 변환 결과와 소요 시간을 이미지에 적용하여 출력한다.
  // precondition : 버퍼 이미지가 존재해야 한다.
  // postcondition : none
void* image_process_thread(void* arg)
{
	struct thr_data* data = (struct thr_data*)arg;
	struct v4l2* v4l2 = data->v4l2;
	struct vpe* vpe = data->vpe;
	struct buffer* capt;
	bool isFirst = true;
	int index;
	int count = 0;
	int i;

	v4l2_reqbufs(v4l2, NUMBUF);
	// 영상을 저장할 큐 버퍼 만큼의 메모리를 할당
	vpe_input_init(vpe);
	// vpe 입력을 초기화한다.
	allocate_input_buffers(data);
	// vpe input buffer를 할당해준다.
	if (vpe->dst.coplanar)
		vpe->disp->multiplanar = true;
	else
		vpe->disp->multiplanar = false;
	printf("disp multiplanar:%d \n", vpe->disp->multiplanar);
	// pass
	vpe_output_init(vpe);
	vpe_output_fullscreen(vpe, data->bfull_screen);
	// vpe 출력을 초기화하고 할당한다.
	for (i = 0; i < NUMBUF; i++)
		v4l2_qbuf(v4l2, vpe->input_buf_dmafd[i], i);
	for (i = 0; i < NUMBUF; i++)
		vpe_output_qbuf(vpe, i);
	// vpe 버퍼에 존재하는 영상을 vpe 가공 후에 큐에 저장한다.
	v4l2_streamon(v4l2);
	vpe_stream_on(vpe->fd, V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE);
	// 영상 캡쳐를 시작하고 vpe 하드웨어의 출력 stream을 on 상태로 한다.
	// 여기까지의 과정이 영상 입출력 버퍼 초기화, vpe 초기화 과정이다.

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
		// driver에서 application으로 소유권을 넘겨준다.


		/*******************************************************
		* 영상처리 코드를 진행되는 부분.
		* img_process에서 steer나 stop flag등 영상처리로 얻을 수 있는 정보를
		* thr_data의 멤버 변수로 저장하여 control_thread로 전달한다.
		********************************************************/
		
		//while(event)
		//{
		//	usleep(5 * 1000);
		//}
		img_process(vpe->disp, capt);
		// 우리가 짠 알고리즘.


		if (disp_post_vid_buffer(vpe->disp, capt, 0, 0, vpe->dst.width, vpe->dst.height))
		{
			ERROR("Post buffer failed");
			return NULL;
		}
		// 영상 출력 버퍼(disp)로 영상(capt)을 보내준다.
		update_overlay_disp(vpe->disp);
		// overlay된 영상(hough_transform된 이미지와 수행 시간)을 병합하여 출력한다.

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
		// application으로 이전되었던 소유권을 다시 driver로 돌려준다.
	}

	MSG("Ok!");
	return NULL;
}

/**
  * @brief  Hough transform the captured image dump and save to file
  * @param  arg: pointer to parameter of thr_data
  * @retval none
  */
  // characteristic : hough transform을 수행한 캡쳐된 이미지 덤프를 파일로 저장한다.
  // precondition : none
  // postcondition : File로 이미지가 저장된다.
//void* capture_dump_thread(void* arg)
//{
//	struct thr_data* data = (struct thr_data*)arg;
//	FILE* fp;
//	char file[50];
//	struct timeval timestamp;
//	struct tm* today;
//	DumpMsg dumpmsg;
//
//	while (1)
//	{
//		if (msgrcv(data->msgq_id, &dumpmsg, sizeof(DumpMsg) - sizeof(long), DUMP_MSGQ_MSG_TYPE, 0) >= 0)
//		{
//			switch (dumpmsg.state_msg)
//			{
//			case DUMP_CMD:
//				gettimeofday(&timestamp, NULL);
//				today = localtime(&timestamp.tv_sec);
//				sprintf(file, "dump_%04d%02d%02d_%02d%02d%02d.%s", today->tm_year + 1900, today->tm_mon + 1, today->tm_mday, today->tm_hour, today->tm_min, today->tm_sec, VPE_OUTPUT_FORMAT);
//				data->dump_state = DUMP_READY;
//				MSG("file name:%s", file);
//				break;
//
//			case DUMP_WRITE_TO_FILE:
//				if ((fp = fopen(file, "w+")) == NULL)
//				{
//					ERROR("Fail to fopen");
//				}
//				else
//				{
//					fwrite(data->dump_img_data, VPE_OUTPUT_IMG_SIZE, 1, fp);
//				}
//				fclose(fp);
//				data->dump_state = DUMP_DONE;
//				break;
//
//			default:
//				MSG("dump msg wrong (%d)", dumpmsg.state_msg);
//				break;
//			}
//		}
//	}
//
//	return NULL;
//}

/**
  * @brief  handling an input command
  * @param  arg: pointer to parameter of thr_data
  * @retval none
  */
  // characteristic : console에서 키 입력을 대기하는 함수이다.
  // 키 입력시:
  // 1. thread간 공유 데이터 수정 (dump_state = DUMP_CMD)
  // 2. msgsnd로 capture_dump_thread 호출 (DUMP_CMD)
  // 3. Dump 완료시 까지 대기: thread간 공유 데이터값 확읶(dump_state = DUMP_DONE)
  // precondition : none
  // postcondition : File로 이미지가 저장된다.

void* input_thread(void* arg)
{
	struct thr_data* data = (struct thr_data*)arg;

	char cmd_input[128];
	char cmd_ready = true;

	while (!data->bstream_start) {
		usleep(100 * 1000);
	}

	MSG("\n\nInput command:");
	MSG("\t dump  : display image(%s, %dx%d) dump", VPE_OUTPUT_FORMAT, VPE_OUTPUT_W, VPE_OUTPUT_H);
	MSG("\n");

	while (1)
	{
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
			else
			{
				printf("cmd_input:%s \n", cmd_input);
			}
			cmd_ready = true;
		}
	}

	return NULL;
}

/**
  * characteristic : 차체를 제어하는 스레드이다.
  *					input 스레드에서 받아온 공유데이터로 어떻게 제어할지 결정한다.
  *
  */
void* control_thread(void* arg)
{
	struct thr_data* data = (struct thr_data*)arg;
	CarControlInit();

	int i;
	while (1)
	{
		SteeringServoControl_Write(data->controlData.steerVal);
		DesireSpeed_Write(data->controlData.desireSpeedVal);

		for (i = 0; i < 2; i++)
		{
			Alarm_Write(ON);
			usleep(200000);
			Alarm_Write(OFF);
			usleep(200000);
		}
		usleep(1000000);
	}

	return NULL;
}

static struct thr_data* pexam_data = NULL;

/**
  * @brief  handling an SIGINT(CTRL+C) signal
  * @param  sig: signal type
  * @retval none
  */
void signal_handler(int sig)
{
	if (sig == SIGINT) {
		pthread_cancel(pexam_data->threads[0]);
		pthread_cancel(pexam_data->threads[1]);
		pthread_cancel(pexam_data->threads[2]);

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

int main(int argc, char** argv)
{
	struct v4l2* v4l2;
	struct vpe* vpe;
	struct thr_data tdata;
	int disp_argc = 3;
	char* disp_argv[] = { "dummy", "-s", "4:480x272", "\0" }; // 추후 변경 여부 확인 후 처리..
	int ret = 0;

	printf("-- 7_all_test Start --\n");

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

	ret = pthread_create(&tdata.threads[1], NULL, control_thread, &tdata);
	if (ret) {
		MSG("Failed creating capture dump thread");
	}
	pthread_detach(tdata.threads[1]);

	ret = pthread_create(&tdata.threads[2], NULL, input_thread, &tdata);
	if (ret) {
		MSG("Failed creating input thread");
	}
	pthread_detach(tdata.threads[2]);

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