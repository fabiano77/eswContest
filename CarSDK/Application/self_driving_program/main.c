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

//?•´?ƒ?„ë¥? ë°”ê¾¸? ¤ë©? ?´ë¶?ë¶„ë§Œ ë³?ê²½í•˜ë©? ?¨
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

#define BASIC_SPEED 55		// ?”„ë¡œê·¸?ž¨ ê¸°ë³¸ ì£¼í–‰ ?†?„,
#define BUZZER_PULSE 100000 // ê¸°ë³¸ ë¶???? ê¸¸ì´

// thr_data?˜ ? •?˜??? ê°ì¢… structure?“¤??? control_mission.h ?œ¼ë¡? ?˜®ê¹? 9/8(????¬)
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

static void draw_operatingtime(struct display *disp, uint32_t time, uint32_t itime, uint32_t mtime)
{
	FrameBuffer tmpFrame;
	unsigned char *pbuf[4];
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
static void img_process(struct display *disp, struct buffer *cambuf, struct thr_data *t_data, float *map1, float *map2)
{
	unsigned char srcbuf[VPE_OUTPUT_W * VPE_OUTPUT_H * 3];
	uint32_t optime;
	struct timeval st, et;
	unsigned char *cam_pbuf[4];
	bool delay_flag = false;
	bool checking_stopline = false;

	if (get_framebuf(cambuf, cam_pbuf) == 0)
	{
		memcpy(srcbuf, cam_pbuf[0], VPE_OUTPUT_W * VPE_OUTPUT_H * 3);
		gettimeofday(&st, NULL);

		/********************************************************/
		/*			?š°ë¦¬ê?? ë§Œë“  ?•Œê³ ë¦¬ì¦? ?•¨?ˆ˜ë¥? ?„£?Š” ë¶?ë¶?		*/
		/********************************************************/

		/* ?¼?¸ ?•„?„°ë§ì´?‚˜ canny ê²°ê³¼ ?™•?¸ */
		if (t_data->imgData.bdebug)
		{
			if (t_data->imgData.debugMode == 9)
				OpenCV_remap(srcbuf, VPE_OUTPUT_W, VPE_OUTPUT_H, srcbuf, map1, map2);
			debugFiltering(srcbuf, VPE_OUTPUT_W, VPE_OUTPUT_H, srcbuf, t_data->imgData.debugMode);
		}

		/* ë¯¸ì…˜ ì§„í–‰ì¤‘ì— ì²˜ë¦¬?•˜?Š” ?˜?ƒì²˜ë¦¬ */
		else if (t_data->imgData.bmission)
		{

			t_data->controlData.beforeSpeedVal = 0;

			/* ì¶”ì›”ì°¨ë¡œ?‹œ?— ?‚¬?š© */
			if (t_data->missionData.overtakingFlag &&
				t_data->missionData.overtakingData.updownCamera == CAMERA_UP)
			{
				usleep(100000);
				/*checkï¿??? ?ï¿½ï¿½?ï¿½ï¿½ camera up*/
				bool check_direction;
				check_direction = checkObstacle(srcbuf, VPE_OUTPUT_W, VPE_OUTPUT_H, srcbuf);
				/*?˜¤ë¥¸ìª½?¸ì§? ?™¼ìª½ì¸ì§? ë§? ë²? ?™•?¸*/
				if (check_direction == true)
				{ //true=>left
					t_data->missionData.overtakingData.leftFlag++;
				}
				else
				{ //false =>right
					t_data->missionData.overtakingData.rightFlag++;
				}

				/*3?šŒ ?Œ?‹¨ ?´?›„ ?™•?¸*/
				if ((t_data->missionData.overtakingData.rightFlag + t_data->missionData.overtakingData.leftFlag) >= 3)
				{
					/*?˜¤ë¥¸ìª½ flagê°? ?°ê²½ìš°*/
					if (t_data->missionData.overtakingData.rightFlag > t_data->missionData.overtakingData.leftFlag)
					{
						t_data->missionData.overtakingData.headingDirection = RIGHT;
					}
					/*?™¼ìª? flagê°? ?°ê²½ìš°*/
					else
					{
						t_data->missionData.overtakingData.headingDirection = LEFT;
					}
					/*?ƒ?™© ?ž¬ì§„ìž… ë§‰ê¸° ?œ„?•œ Camera Down*/
					t_data->missionData.overtakingData.updownCamera = CAMERA_DOWN;
				}
				//srcbufë¥? ?™œ?š©?•˜?—¬ capture?•œ ?˜?ƒ?„ ë³??™˜
			}

			/* ?‹ ?˜¸?“± ?™•?¸ */
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
							delay_flag = true;
							buzzer(2, BUZZER_PULSE, BUZZER_PULSE);
							t_data->missionData.signalLightData.state = DETECTION_FINISH;
							t_data->missionData.signalLightData.finalDirection = 1;
							t_data->imgData.bcheckSignalLight = false;
						}
						else if (t_data->missionData.signalLightData.Accumulation_greenVal <= -3)
						{
							delay_flag = true;
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

			/* ?”¼?‹ˆ?‹œ ?¼?¸ê³¼ì˜ ê±°ë¦¬ ì¸¡ì • */
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

			/*??‚  ?•Œ ?‚¬?š©*/
			if (t_data->missionData.finishData.checkFront == true)
			{
				t_data->imgData.topMode = 1; //?•ž?´ ?” ?ž˜ë³´ì´?Š” mode 1?‚¬?š©
				topview_transform(srcbuf, VPE_OUTPUT_W, VPE_OUTPUT_H, srcbuf, t_data->imgData.topMode);
				t_data->missionData.finishData.distEndLine = checkFront(srcbuf, VPE_OUTPUT_W, VPE_OUTPUT_H, srcbuf);
				/*ë¬´ì˜ë¯¸í•œ ê°’ì¸ ê²½ìš° ?•Œê³ ë¦¬ì¦˜ì— ë§žê²Œ steering ì§„í–‰*/
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
						//?´? „ ?†?„??? ?‹¬?¼ì¡Œì„ ?•Œë§? ?†?„ê°? ?¸ê°?.
						DesireSpeed_Write_uart(t_data->controlData.desireSpeedVal);
						t_data->controlData.beforeSpeedVal = t_data->controlData.desireSpeedVal;
					}
				}
				else if (t_data->missionData.finishData.distEndLine < 320)
				{ /*ê±°ë¦¬ê°? 40(360-40)?´?•˜ë¡? ?ƒì§??œ ê²½ìš° ?˜?ƒì²˜ë¦¬ ì¢…ë£Œ*/
					t_data->missionData.finishData.checkFront = false;
				}
			}
		}

		/* ê¸°ë³¸ ?ƒ?ƒœ?—?„œ ì²˜ë¦¬?˜?Š” ?˜?ƒì²˜ë¦¬ */
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
						//?´? „ ?†?„??? ?‹¬?¼ì¡Œì„ ?•Œë§? ?†?„ê°? ?¸ê°?.
						DesireSpeed_Write_uart(t_data->controlData.desireSpeedVal);
						t_data->controlData.beforeSpeedVal = t_data->controlData.desireSpeedVal;
					}
				}
			}
			else
			{
				t_data->controlData.beforeSpeedVal = 0;
			}

			/*checkWhiteLineFlagï¿??? True?ï¿½ï¿½ ê²½ìš°, RoundAbout */
		}

		/********************************************************/
		/*			?˜?ƒì²˜ë¦¬ ì¢…ë£Œ								*/
		/********************************************************/

		/* ?˜?ƒì²˜ë¦¬?›„ ?˜¤ë²„ë ˆ?´ë¡? ? •ë³? ?“±?“± ì¶œë ¥.*/
		if (checking_stopline)
		{
			checking_stopline = false;
			displayPrintStopLine(srcbuf, VPE_OUTPUT_W, VPE_OUTPUT_H);
		}
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

		/*		?˜„?ž¬ ?˜?ƒ .jpg?ŒŒ?¼ë¡? ????ž¥		*/
		if (t_data->imgData.dump_request)
		{
			opencv_imwrite(srcbuf);
			t_data->imgData.dump_request = false;
		}

		/*		?˜„?ž¬ ?˜?ƒ .avi?ŒŒ?¼ë¡? ?…¹?™”		*/
		if (t_data->imgData.bvideoRecord)
		{
			memcpy(t_data->img_data_buf, srcbuf, VPE_OUTPUT_IMG_SIZE);
		}

		/* ?‹ ?˜¸?“± ê²?ì¶œí™”ë©´ì„ ?œ ì§??•˜ê¸? ?œ„?•œ delay */
		if (delay_flag)
		{
			usleep(1700000); // 1700ms == 1.7s
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
void *image_process_thread(void *arg)
{
	struct thr_data *data = (struct thr_data *)arg;
	struct v4l2 *v4l2 = data->v4l2;
	struct vpe *vpe = data->vpe;
	struct buffer *capt;
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
		/* ?˜?ƒì²˜ë¦¬ ?‹œ?ž‘										*/
		/********************************************************/

		img_process(vpe->disp, capt, data, map1, map2);

		/********************************************************/
		/* ?˜?ƒì²˜ë¦¬ ì¢…ë£Œ										*/
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

void *video_record_thread(void *arg)
{
	struct thr_data *data = (struct thr_data *)arg;
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

	// stì²´í¬ë¥? ?´ê³³ì—?„œ ?•˜ê³?, etì²´í¬ë¥? ?‘ ë²ˆì§¸ while?— ?•˜?—¬ ë§? ?”„? ˆ?ž„ë§ˆë‹¤ delay_msë§Œí¼ ?ˆ„?‚°?‹œ?‚¤?–´
	// ?”„? ˆ?ž„?´ ì°ížˆ?Š” ?‹œ? ê³? ?˜?ƒ?—?„œ?˜ ?•´?‹¹ ?‹œ? ?„ ê·¼ì‚¬?•˜?„ë¡? ?•œ?‹¤. (0.1ì´? ? ˆ???ê°? ìº¡ì²˜ ????‹  ?˜?ƒ?˜ ? „ì²´ê¸¸?´?˜ ë¹„ìœ¨ë¡?)
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

		while (1) //?˜?ƒ ?…¹?™” ?‹±?¬ë¥? ë§žì¶°ì£¼ê¸° ?œ„?•œ delay
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
void *mission_thread(void *arg)
{
	struct thr_data *data = (struct thr_data *)arg;
	struct timeval time;
	time.tv_sec = 0;
	enum MissionState start = READY;
	enum MissionState flyover = DONE;
	enum MissionState priority = DONE;
	enum MissionState parking = NONE;
	enum MissionState roundabout = DONE;
	enum MissionState tunnel = READY;
	enum MissionState overtake = NONE;
	enum MissionState signalLight = NONE;
	enum MissionState finish = NONE;

	//int i = 0;
	int temp = 0;
	data->missionData.ms[0] = start;
	data->missionData.ms[1] = flyover;
	data->missionData.ms[2] = priority;
	data->missionData.ms[3] = parking;
	data->missionData.ms[4] = tunnel;
	data->missionData.ms[5] = roundabout;
	data->missionData.ms[6] = overtake;
	data->missionData.ms[7] = signalLight;
	data->missionData.ms[8] = finish;

	while (1)
	{
		data->missionData.loopTime = timeCheck(&time);

		if (start && start != DONE)
		{
			data->missionData.ms[0] = start;
			startFunc(data);
			start = DONE;
			//flyover = READY;
			data->missionData.ms[0] = start;
			//data->missionData.ms[1] = flyover;
		}

		if (flyover && flyover != DONE)
		{
			flyoverFunc(data);
			flyover = DONE;
		}

		if (priority && priority != DONE)
		{
			if (priorityFunc(data))
				priority = DONE;
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
				}
			}
		}

		if (tunnel && tunnel != DONE)
		{
			if (tunnelFunc(data))
			{
				tunnel = DONE;
				parking = READY;
			}
		}

		if (roundabout && roundabout != DONE)
		{
			//data->imgData.bcheckFrontWhite = true;	// ?ï¿½ï¿½ï¿??? ?ï¿½ï¿½ï¿????ï¿½ï¿½ ?ï¿½ï¿½?ï¿½ï¿½?ï¿½ï¿½ï¿??? ?ï¿½ï¿½ï¿??? ON
			if (roundaboutFunc(data))
				roundabout = DONE;
		}

		if (overtake && overtake != DONE)
		{
			int distance_1 = DistanceSensor_cm(1);
			data->imgData.bwhiteLine = true;
			if (distance_1 <= 30)
			{
				printf("\t distance_1 = %d \n", distance_1);
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
				DesireSpeed_Write_uart(0);
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
							/*
							 dahee's function
							*/
							Winker_Write(ALL_OFF);
							usleep(500000);
							if (DistanceSensor_cm(1) < 20)
							{
								sprintf(data->imgData.missionString, "Detect Error");
								DesireDistance(-50, thresDistance, 1100);
								data->missionData.overtakingData.headingDirection = LEFT;
							}
							else
							{
								state = SIDE_ON;
								sprintf(data->imgData.missionString, "Detect Side");
								DesireSpeed_Write_uart(BASIC_SPEED);
								usleep(500000);
							}
						}
						else if (data->missionData.overtakingData.headingDirection == LEFT &&
								 data->missionData.overtakingData.updownCamera == CAMERA_DOWN)
						{

							sprintf(data->imgData.missionString, "Left to go");
							Winker_Write(LEFT_ON);
							/*
							 dahee's function
							*/
							Winker_Write(ALL_OFF);
							usleep(500000);
							if (DistanceSensor_cm(1) < 20)
							{
								sprintf(data->imgData.missionString, "Detect Error");
								DesireDistance(-50, thresDistance, 1900);
								data->missionData.overtakingData.headingDirection = RIGHT;
							}
							else
							{
								state = SIDE_ON;
								sprintf(data->imgData.missionString, "Detect Side");
								DesireSpeed_Write_uart(BASIC_SPEED);
								usleep(500000);
							}
						}
						else
						{
						}

						break;

					case SIDE_ON:
						data->imgData.bmission = false;
						// right
						if (data->missionData.overtakingData.headingDirection == RIGHT)
						{
							int distance_5 = DistanceSensor_cm(5);
							int distance_6 = DistanceSensor_cm(6);

							if (distance_5 <= 30 || distance_6 <= 30)
							{
								obstacle = true;
							}
							else if (obstacle == true)
							{
								if (distance_5 > 30 && distance_6 > 30) // side-off condition
								{
									data->imgData.bmission = true; // Auto Steering off
									usleep(100000);
									DesireSpeed_Write_uart(0);
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
							int distance_3 = DistanceSensor_cm(3);
							int distance_2 = DistanceSensor_cm(2);
							if (distance_3 <= 30 || distance_2 <= 30)
							{
								obstacle = true;
							}
							else if (obstacle == true)
							{
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
						usleep(10000);
						data->imgData.bmission = true; //Auto Steering off
						usleep(10000);
						//right
						if (data->missionData.overtakingData.headingDirection == RIGHT) // return left
						{
							Winker_Write(LEFT_ON);
							/*
							 dahee's function
							*/
							Winker_Write(ALL_OFF);
						}
						//left
						else if (data->missionData.overtakingData.headingDirection == LEFT) //return right
						{
							Winker_Write(RIGHT_ON);
							/*
							 dahee's function
							*/
							Winker_Write(ALL_OFF);
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

		if (data->imgData.bvideoSave == false && data->imgData.bvideoRecord == true)
		{
			if (temp == 5)
			{
				if (DistanceSensor_cm(1) <= 5 && DistanceSensor_cm(4) <= 5)
				{
					data->imgData.bvideoSave = true;
					buzzer(3, 100000, 300000);
				}
				temp = 0;
			}
			else
			{
				temp++;
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

		if (data->imgData.bvideoSave == false && data->imgData.bvideoSave == true)
		{
			if (DistanceSensor_cm(1) < 15 && DistanceSensor_cm(4) < 15)
			{
				buzzer(1, 0, 1000000);
				data->imgData.bvideoRecord = false;
				data->imgData.bvideoSave = true;
			}
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

		printf("-- 6_camera_opencv_disp example End --\n");
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
	char *disp_argv[] = {"dummy", "-s", "4:480x272", "\0"}; // ì¶”í›„ ï¿???ï¿??? ?ï¿½ï¿½ï¿??? ?ï¿½ï¿½?ï¿½ï¿½ ?ï¿½ï¿½ ì²˜ë¦¬..
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
	// z-order, alpha, multiplied-alpha ?ï¿½ï¿½?ï¿½ï¿½ (overlayï¿??? ?ï¿½ï¿½?ï¿½ï¿½ plane ï¿??? ?ï¿½ï¿½?ï¿½ï¿½)

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
	// signal error

	pause();

	return ret;
}
