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

#define CAPTURE_IMG_W 1280
#define CAPTURE_IMG_H 720
#define CAPTURE_IMG_SIZE (CAPTURE_IMG_W * CAPTURE_IMG_H * 2) // YUYU : 16bpp
#define CAPTURE_IMG_FORMAT "uyvy"

//?ï¥?ÉÅ?èÑÎ•? Î∞îÍæ∏?†§Î©? ?ù¥Î∂?Î∂ÑÎßå Î≥?Í≤ΩÌïòÎ©? ?ê®
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

#define BASIC_SPEED 55		// ?îÑÎ°úÍ∑∏?û® Í∏∞Î≥∏ Î∞îÌ?¥ÏÜç?èÑ, autoSteer??? Ï£ºÎ°ú ?Ç¨?ö©.
#define BUZZER_PULSE 100000 // Í∏∞Î≥∏ Î∂???? Í∏∏Ïù¥

// Í∞ÅÏ¢Ö structure?äî control_mission.h ?óê ?ù¥?èô?ê®. 9/8(?ôî)

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
	pthread_t threads[4]; //ÔøΩÔøΩÔøΩÔøΩÔøΩÂ∞≥ÔøΩÔøΩ
};

static struct thr_data *ptr_data;
/******************** function ********************/
static void DesireDistance(int SettingSpeed, int SettingDistance, int SettingSteering);

static void SteeringServo_Write(signed short angle);

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
		/*			?ö∞Î¶¨Í?? ÎßåÎì† ?ïåÍ≥†Î¶¨Ï¶? ?ï®?àòÎ•? ?Ñ£?äî Î∂?Î∂?		*/
		/********************************************************/

		/* ?ùº?ù∏ ?ïÑ?Ñ∞ÎßÅÏù¥?Çò canny Í≤∞Í≥º ?ôï?ù∏ */
		if (t_data->imgData.bdebug)
		{
			if (t_data->imgData.debugMode == 9)
				OpenCV_remap(srcbuf, VPE_OUTPUT_W, VPE_OUTPUT_H, srcbuf, map1, map2);
			debugFiltering(srcbuf, VPE_OUTPUT_W, VPE_OUTPUT_H, srcbuf, t_data->imgData.debugMode);
		}

		/* ÎØ∏ÏÖò ÏßÑÌñâÏ§ëÏóê Ï≤òÎ¶¨?ïò?äî ?òÅ?ÉÅÏ≤òÎ¶¨ */
		else if (t_data->imgData.bmission)
		{
			/* ?ûê?ú®Ï£ºÌñâ?ùò ?Üç?èÑÏ°∞Ï†à?ùÑ ?úÑ?ïú Î≥??àòÏ≤òÎ¶¨ */
			t_data->controlData.beforeSpeedVal = 0;

			/* Ï∂îÏõîÏ∞®Î°ú?ãú?óê ?Ç¨?ö© */
			if (t_data->missionData.overtakingFlag &&
				t_data->missionData.overtakingData.updownCamera == CAMERA_UP)
			{
				usleep(100000);
				/*checkÎ•? ?úÑ?ïú camera up*/
				bool check_direction;
				check_direction = checkObstacle(srcbuf, VPE_OUTPUT_W, VPE_OUTPUT_H, srcbuf);
				/*?ò§Î•∏Ï™Ω?ù∏Ïß? ?ôºÏ™ΩÏù∏Ïß? Îß? Î≤? ?ôï?ù∏*/
				if (check_direction == true)
				{ //true=>left
					t_data->missionData.overtakingData.leftFlag++;
				}
				else
				{ //false =>right
					t_data->missionData.overtakingData.rightFlag++;
				}

				/*3?öå ?åê?ã® ?ù¥?õÑ ?ôï?ù∏*/
				if ((t_data->missionData.overtakingData.rightFlag + t_data->missionData.overtakingData.leftFlag) >= 3)
				{
					/*?ò§Î•∏Ï™Ω flagÍ∞? ?Å∞Í≤ΩÏö∞*/
					if (t_data->missionData.overtakingData.rightFlag > t_data->missionData.overtakingData.leftFlag)
					{
						t_data->missionData.overtakingData.headingDirection = RIGHT;
					}
					/*?ôºÏ™? flagÍ∞? ?Å∞Í≤ΩÏö∞*/
					else
					{
						t_data->missionData.overtakingData.headingDirection = LEFT;
					}
					/*?ÉÅ?ô© ?û¨ÏßÑÏûÖ ÎßâÍ∏∞ ?úÑ?ïú Camera Down*/
					t_data->missionData.overtakingData.updownCamera = CAMERA_DOWN;
				}
				//srcbufÎ•? ?ôú?ö©?ïò?ó¨ capture?ïú ?òÅ?ÉÅ?ùÑ Î≥??ôò
			}

			/* ?ã†?ò∏?ì± ?ôï?ù∏ */
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

			/* ?îº?ãà?ãú ?ùº?ù∏Í≥ºÏùò Í±∞Î¶¨ Ï∏°Ï†ï */
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

			/*?Åù?Ç† ?ïå ?Ç¨?ö©*/
			if (t_data->missionData.finishData.checkFront == true)
			{
				t_data->imgData.topMode = 1; //?ïû?ù¥ ?çî ?ûòÎ≥¥Ïù¥?äî mode 1?Ç¨?ö©
				topview_transform(srcbuf, VPE_OUTPUT_W, VPE_OUTPUT_H, srcbuf, t_data->imgData.topMode);
				t_data->missionData.finishData.distEndLine = checkFront(srcbuf, VPE_OUTPUT_W, VPE_OUTPUT_H, srcbuf);
				/*Î¨¥ÏùòÎØ∏Ìïú Í∞íÏù∏ Í≤ΩÏö∞ ?ïåÍ≥†Î¶¨Ï¶òÏóê ÎßûÍ≤å steering ÏßÑÌñâ*/
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
						//?ù¥?†Ñ ?Üç?èÑ??? ?ã¨?ùºÏ°åÏùÑ ?ïåÎß? ?Üç?èÑÍ∞? ?ù∏Í∞?.
						DesireSpeed_Write_uart(t_data->controlData.desireSpeedVal);
						t_data->controlData.beforeSpeedVal = t_data->controlData.desireSpeedVal;
					}
				}
				else if (t_data->missionData.finishData.distEndLine < 320)
				{ /*Í±∞Î¶¨Í∞? 40(360-40)?ù¥?ïòÎ°? ?ÉêÏß??êú Í≤ΩÏö∞ ?òÅ?ÉÅÏ≤òÎ¶¨ Ï¢ÖÎ£å*/
					t_data->missionData.finishData.checkFront = false;
				}
			}
		}

		/* Í∏∞Î≥∏ ?ÉÅ?Éú?óê?Ñú Ï≤òÎ¶¨?êò?äî ?òÅ?ÉÅÏ≤òÎ¶¨ */
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
						//?ù¥?†Ñ ?Üç?èÑ??? ?ã¨?ùºÏ°åÏùÑ ?ïåÎß? ?Üç?èÑÍ∞? ?ù∏Í∞?.
						DesireSpeed_Write_uart(t_data->controlData.desireSpeedVal);
						t_data->controlData.beforeSpeedVal = t_data->controlData.desireSpeedVal;
					}
				}
			}
			else
			{
				/* ?ûê?ú®Ï£ºÌñâ?ùò ?Üç?èÑÏ°∞Ï†à?ùÑ ?úÑ?ïú Î≥??àòÏ≤òÎ¶¨ */
				t_data->controlData.beforeSpeedVal = 0;
			}

			/*checkWhiteLineFlagÍ∞? True?ù∏ Í≤ΩÏö∞, RoundAbout */
		}

		/********************************************************/
		/*			?òÅ?ÉÅÏ≤òÎ¶¨ Ï¢ÖÎ£å								*/
		/********************************************************/

		/* ?òÅ?ÉÅÏ≤òÎ¶¨?õÑ ?ò§Î≤ÑÎ†à?ù¥Î°? ?†ïÎ≥? ?ì±?ì± Ï∂úÎ†•.*/
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

		/*		?òÑ?û¨ ?òÅ?ÉÅ .bmp?åå?ùºÎ°? ????û•		*/
		if (t_data->imgData.dump_request)
		{
			opencv_imwrite(srcbuf);
			t_data->imgData.dump_request = false;
		}

		/*		?òÑ?û¨ ?òÅ?ÉÅ .avi?åå?ùºÎ°? ?Öπ?ôî		*/
		if (t_data->imgData.bvideoRecord)
		{
			memcpy(t_data->img_data_buf, srcbuf, VPE_OUTPUT_IMG_SIZE);
		}

		/* ?ã†?ò∏?ì± Í≤?Ï∂úÌôîÎ©¥ÏùÑ ?ú†Ïß??ïòÍ∏? ?úÑ?ïú delay */
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
		/* ?òÅ?ÉÅÏ≤òÎ¶¨ ?ãú?ûë										*/
		/********************************************************/

		img_process(vpe->disp, capt, data, map1, map2);

		/********************************************************/
		/* ?òÅ?ÉÅÏ≤òÎ¶¨ Ï¢ÖÎ£å										*/
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

	// stÏ≤¥ÌÅ¨Î•? ?ù¥Í≥≥Ïóê?Ñú ?ïòÍ≥?, etÏ≤¥ÌÅ¨Î•? ?ëê Î≤àÏß∏ while?óê ?ïò?ó¨ Îß? ?îÑ?†à?ûÑÎßàÎã§ delay_msÎßåÌÅº ?àÑ?Ç∞?ãú?Ç§?ñ¥
	// ?îÑ?†à?ûÑ?ù¥ Ï∞çÌûà?äî ?ãú?†êÍ≥? ?òÅ?ÉÅ?óê?Ñú?ùò ?ï¥?ãπ ?ãú?†ê?ùÑ Í∑ºÏÇ¨?ïò?èÑÎ°? ?ïú?ã§. (0.1Ï¥? ?†à???Í∞? Ï∫°Ï≤ò ????ã† ?òÅ?ÉÅ?ùò ?†ÑÏ≤¥Í∏∏?ù¥?ùò ÎπÑÏú®Î°?)
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

		while (1) //?òÅ?ÉÅ ?Öπ?ôî ?ã±?Å¨Î•? ÎßûÏ∂∞Ï£ºÍ∏∞ ?úÑ?ïú delay
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

	//Í∞? ÎØ∏ÏÖò?ù¥ ?àò?ñâ?êòÍ≥†ÎÇòÎ©? detectÎ•? ?ïòÏß? ?ïä?èÑÎ°? Î≥??àò?Ñ§?†ï.

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
			//?ò§Î•∏Ï™Ω Í±∞Î¶¨?Ñº?ÑúÍ∞? ?ïà?û°?ûàÎ©? ?ÉàÏ∂úÌïò?äî Í≤ÉÏúºÎ°? ?òà?ÉÅ.
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
			//imgProcess?óê?Ñú ?ö∞?Ñ†?†ïÏß??ëúÏß??åê Ï≤¥ÌÅ¨ ?ôú?Ñ±?ôî
			data->imgData.bcheckPriority = true;

			if (data->missionData.frame_priority >= 2) //?ö∞?Ñ†?†ïÏß??ëúÏß??åê 2?îÑ?†à?ûÑ Í≤?Ï∂?.
			{
				data->imgData.bskip = true;
				DesireSpeed_Write(0);
				Winker_Write(ALL_ON);
				while (data->imgData.bcheckPriority)
				{
					if (data->missionData.frame_priority == 0) //?ö∞?Ñ†?†ïÏß??ëúÏß??åê ?Ç¨?ùºÏß?Î©?
					{
						data->imgData.bcheckPriority = false; //imgProcess?óê?Ñú ?ö∞?Ñ†?†ïÏß??ëúÏß??åê Ï≤¥ÌÅ¨ ÎπÑÌôú?Ñ±?ôî
					}
					usleep(100000);
				}
				usleep(1000000); //1Ï¥? ???Í∏?
				Winker_Write(ALL_OFF);
				DesireSpeed_Write(BASIC_SPEED);
				priority = DONE;
				data->imgData.bskip = false;
			}
		}

		if (parking && parking != DONE)
		{
			if (1) //(data->controlData.steerVal <= 1600 && data->controlData.steerVal >= 1400) || parking == REMAIN)
			{
				if (DistanceSensor_cm(2) <= 28) //Ï≤òÏùå Î≤ΩÏù¥ Í∞êÏ???êò?óà?ùÑ Í≤ΩÏö∞
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
					int escape_distance = 0;

					enum ParkingState state = FIRST_WALL;
					enum HorizontalStep step_h = FIRST_BACKWARD;
					enum VerticalStep step_v = FIRST_BACKWARD_V;

					while (state && wrong_detection) // state == ENDÍ∞? ?ïÑ?ãå?ù¥?ÉÅ Î£®ÌîÑ ÏßÑÌñâ
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
							Ï£ºÏ∞® ?è≠?óê ????ïú Í±∞Î¶¨Î•? Ï∏°Ï†ï?ïòÍ∏? ?úÑ?ï¥ Í±∞Î¶¨ Ï∏°Ï†ï ?ãú?ûë
							*/

							/*
							?ö∞?Ñ† ?†ïÏß? ?ëúÏß??åê?ùÑ Í∞êÏ???ïòÎ©? Ï£ºÏ∞®?óê?Ñú ?ÉàÏ∂úÎêò?èÑÎ°? Ï°∞Ïπò
							*/
							if (data->missionData.frame_priority)
							{
								wrong_detection = 0;
								break;
							}
							// Encoder Ï∏°Ï†ï
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
								// Ï£ºÏ∞® Í≥µÍ∞Ñ Ï∏°Ï†ï?ù¥ ?ïà?êò?äî Í≤ΩÏö∞, ?ûòÎ™ªÎêú Ï£ºÏ∞® Î∂ÑÍ∏∞ ÏßÑÏûÖ?úºÎ°? ?åê?ã®?ïòÍ≥? ?ÉàÏ∂?.
							}
							if (data->missionData.parkingData.frontRight == true)
							{

								/*
								Í±∞Î¶¨ Ï∏°Ï†ï Ï¢ÖÎ£å -> Ï∏°Ï†ï Í±∞Î¶¨Î•? Î≥??àò?óê ?ã¥?äî?ã§.
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
								// ?ëêÎ≤? Ïß? Î≤ΩÏóê Ï∞®Îüâ ?ö∞Ï∏? ?õÑÎ∞? ?Ñº?ÑúÍ∞? Í±∏Î¶∞ ?ÉÅ?Éú?ù¥?ã§. -> ?àòÏß? ?òê?äî ?àò?èâ Ï£ºÏ∞® ÏßÑÌñâ.
							}
							break;

						case PARKING_START:
							sprintf(data->imgData.missionString, "Parking Start");
							/*
							?àòÏß? Î∞? ?àò?èâ Ï£ºÏ∞® Íµ¨Î¨∏ Ï∂îÍ??.
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
										// ?öå?†Ñ Í∞? ?àò?†ï Î∂?Î∂?
										usleep(80000);
										DesireSpeed_Write(-60);
										//usleep(200000);
										while (1)
										{
											if (DistanceSensor_cm(3) <= 13 && DistanceSensor_cm(5) <= 13)
											{
												DesireSpeed_Write(0);
												//usleep(20000);
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
										DesireSpeed_Write(-35);
										//usleep(50000);
										// Î∞îÌ?¥Î?? ?ùº?ûêÎ°? ÎßûÏ∂∞Ï£ºÍ≥† ?õÑÏßÑÌïú?ã§.
										if (DistanceSensor_cm(2) <= 11)
										{
											DesireSpeed_Write(0);
											//usleep(50000);
											step_v = RIGHT_FRONT_V;
											break;
										}
										else if (DistanceSensor_cm(6) <= 11)
										{
											DesireSpeed_Write(0);
											//usleep(50000);
											step_v = LEFT_FRONT_V;
											break;
										}
										if (DistanceSensor_cm(5) <= 4)
										{
											// ?ñ∏?çî?ä§?ã∞?ñ¥ ?ÉÅ?ô©
											printf("under steer\n");
											DesireSpeed_Write(0);
											//usleep(50000);
											step_v = UNDER_STEER_V;
											break;
										}
										else if (DistanceSensor_cm(3) <= 4)
										{
											// ?ò§Î≤ÑÏä§?ã∞?ñ¥ ?ÉÅ?ô©
											printf("over steer\n");
											DesireSpeed_Write(0);
											//usleep(50000);
											step_v = OVER_STEER_V;
											break;
										}
										break;

									case UNDER_STEER_V:
										sprintf(data->imgData.missionString, "UNDER_STEER");
										DesireDistance(60, 100, 1300);
										//usleep(30000);
										DesireDistance(60, 100, 1500);
										//usleep(30000);
										DesireDistance(60, 200, 1750);
										//usleep(30000);
										step_v = SECOND_BACKWARD_V;
										break;

									case OVER_STEER_V:
										sprintf(data->imgData.missionString, "OVER_STEER");
										DesireDistance(60, 100, 1700);
										//usleep(30000);
										DesireDistance(60, 100, 1500);
										//usleep(30000);
										DesireDistance(60, 200, 1250);
										//usleep(30000);
										step_v = SECOND_BACKWARD_V;
										break;

									case RIGHT_FRONT_V:
										sprintf(data->imgData.missionString, "RIGHT_FRONT_V");
										int right_difference = DistanceSensor_cm(2) - DistanceSensor_cm(3);
										if (abs(right_difference) < 3)
										{
											DesireSpeed_Write(0);
											//usleep(50000);
											step_v = FIRST_FORWARD_V;
											break;
										}
										DesireDistance(60, 100, 1500 - (right_difference * 80));
										//usleep(200000);
										DesireDistance(-30, 400, 1500);
										//usleep(200000);
										break;

									case LEFT_FRONT_V:
										sprintf(data->imgData.missionString, "RIGHT_FRONT_V");
										int left_difference = DistanceSensor_cm(6) - DistanceSensor_cm(5);
										if (abs(left_difference) < 3)
										{
											DesireSpeed_Write(0);
											//usleep(50000);
											step_v = FIRST_FORWARD_V;
											break;
										}
										DesireDistance(60, 100, 1500 + (left_difference * 80));
										//usleep(200000);
										DesireDistance(-30, 400, 1500);
										//usleep(200000);
										break;

									case FIRST_FORWARD_V:
										sprintf(data->imgData.missionString, "FIRST_FORWARD_V");
										DesireDistance(-30, 400, 1500);
										//usleep(1000000);
										step_v = SECOND_FORWARD_V;
										Winker_Write(ALL_ON);
										buzzer(1, 0, 300000);
										Winker_Write(ALL_OFF);
										break;

									case SECOND_FORWARD_V:
										sprintf(data->imgData.missionString, "SECOND_FORWARD_V");
										DesireSpeed_Write(60);
										//usleep(50000);
										if (DistanceSensor_cm(2) >= 20 && DistanceSensor_cm(6) >= 20)
										{
											//DesireSpeed_Write(0);
											//usleep(50000);
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
								DesireDistance(60, 230, 1500);
								// Ï£ºÏ∞® Í∞? ?àò?†ï Î∂?Î∂?
								while (data->missionData.parkingData.horizontalFlag)
								{
									data->missionData.loopTime = timeCheck(&time);
									switch (step_h)
									{
									case FIRST_BACKWARD:
										sprintf(data->imgData.missionString, "FIRST_BACKWARD");
										DesireDistance(-60, 800, 1050);
										//usleep(200000);
										DesireDistance(-60, 400, 1500);
										//usleep(200000);
										SteeringServo_Write(2000);
										usleep(50000);
										DesireSpeed_Write(-26);

										while (1)
										{
											if (DistanceSensor_cm(4) <= 10)
											{
												DesireSpeed_Write(0);
												break;
											}
											usleep(5000);
										}
										SteeringServo_Write(1250);
										usleep(80000);
										DesireSpeed_Write(40);
										//usleep(5000);
										while (1)
										{
											if ((abs(DistanceSensor_cm(2) - DistanceSensor_cm(3)) <= 3) || DistanceSensor_cm(1) <= 4)
											{
												DesireSpeed_Write(0);
												step_h = SECOND_BACKWARD;
												break;
											}
											usleep(5000);
										}

										break;

									case SECOND_BACKWARD:
										sprintf(data->imgData.missionString, "SECOND_BACKWARD");
										//sprintf(data->imgData.missionString, "d1 = %d, d2 = %d, d3 = %d", DistanceSensor_cm(1), DistanceSensor_cm(2), DistanceSensor_cm(3));
										int difference = DistanceSensor_cm(2) - DistanceSensor_cm(3);
										if (difference < -2)
										{
											//sprintf(data->imgData.missionString, "d1 = %d, d2 = %d, d3 = %d", DistanceSensor_cm(1), DistanceSensor_cm(2), DistanceSensor_cm(3));
											DesireDistance(-30, 400, 1300);
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
											DesireDistance(-30, 400, 1700);
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
											escape_distance = (DistanceSensor_cm(2) + DistanceSensor_cm(3)) / 2;
											//usleep(50000);
											SteeringServo_Write(1500);
											//usleep(1000000);
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

										if (DistanceSensor_cm(4) <= 6)
										{
											step_h = ESCAPE;
											break;
										}
										DesireSpeed_Write(-27);

										while (1)
										{
											if (DistanceSensor_cm(4) <= 7)
											{
												DesireSpeed_Write(0);
												usleep(5000);
												//if (escape_distance <= 7)
												step_h = ESCAPE;
												//else
												//step_h = ESCAPE_3;
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
										//DesireDistance(60, 700, 1300);
										DesireDistance(60, 600, 1100);
										data->missionData.parkingData.horizontalFlag = 0;
										break;

									default:
										break;
									}
									usleep(5000);
								}
							}
							DesireSpeed_Write(35);
							state = DONE_P;

							gettimeofday(&et_p, NULL);
							float parkingTime = (((et_p.tv_sec - st_p.tv_sec) * 1000) + ((int)et_p.tv_usec / 1000 - (int)st_p.tv_usec / 1000)) / 1000.0;
							printf("parking time : %f\n", parkingTime);

							if (parking == REMAIN)
								parking = DONE;
							else if (parking == READY)
								parking = REMAIN;

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
					}
					if (parking == DONE)
					{
						printf("Second Parking is Done!\n");
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
				//DesireSpeed_Write(0);

				frontLightOnOff(data->controlData.lightFlag, false);

				//buzzer(1, 0, 500000);
				//usleep(100000);

				//DesireDistance(-40, 400, 1500);
				//usleep(100000);

				printf("Tunnel OUT\n");
				tunnel = DONE;
				data->imgData.bmission = false;
				data->imgData.bprintString = false;
				data->missionData.btunnel = false;
			}
		}

		if (roundabout && roundabout != DONE)
		{
			//data->imgData.bcheckFrontWhite = true;	// ?†ÑÎ∞? ?†ïÏß??Ñ† ?òÅ?ÉÅ?úºÎ°? ?ÉêÏß? ON

			if (StopLine(4)) //|| data->missionData.finish_distance != -1)	//?òÅ?ÉÅ?úºÎ°? ?†ïÏß??Ñ†?ùò Í±∞Î¶¨Í∞? Ï∏°Ï†ï?êòÎ©?
			{
				//onlyDistance(BASIC_SPEED, (data->missionData.finish_distance / 26.0) * 500);	//?†ïÏß??Ñ†ÍπåÏ?? Í∞??Ñú stop
				//data->missionData.finish_distance = -1;

				/* Í∏∞Ï°¥?ùò roundaboutÎ∂ÑÍ∏∞ ÏßÑÏûÖ */
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
						//DesireDistance(speed, 600, 1500, &(data->controlData)); //?ïû ?Ñº?Ñú Î∞õÏïÑ?ò§Î©¥ÏÑú ?ùº?†ïÍ±∞Î¶¨ Í∞??äî ?ï®?àò Ï∂îÍ??.
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
						if (data->controlData.steerVal - 1500 < 30) // steerVal 1500 ?ù¥?ÉÅ?úºÎ°? ?ú†Ïß??êò?ã§Í∞? ÏßÅÏßÑ Íµ¨Í∞Ñ?ù¥ ?Çò?ò¨ ?ïå
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
			/* Î∂ÑÍ∏∞ÏßÑÏûÖ Î™ÖÎ†π Ïß??ãú */
			if (data->controlData.steerVal <= 1600 &&
				data->controlData.steerVal >= 1400)
			{
				if (DistanceSensor_cm(1) < 30) //?†ÑÎ∞? ?û•?ï†Î¨? Í∞êÏ?? //Ï£ºÏ∞® ?ÉÅ?ô©?ù¥ ?ïÑ?ãê?ïå, Î∂ÑÍ∏∞ÏßÑÏûÖ Í∞??ä•
				{
					data->imgData.btopview = false; //topview off
					data->imgData.bmission = true;	//?òÅ?ÉÅÏ≤òÎ¶¨ X
					//data->imgData.bwhiteLine = true; // ?ù∞?Éâ ÏßÅÏÑ† O
					data->imgData.bprintString = true;
					sprintf(data->imgData.missionString, "overtake");
					printf("overtake \n");
					enum OvertakeState state = FRONT_DETECT;
					data->missionData.overtakingData.headingDirection = STOP;
					data->missionData.overtakingFlag = true;
					data->imgData.bwhiteLine = true;
					bool obstacle = false;
					int thresDistance = 450;
					/*Ï∞®Îüâ ?†ïÏß?*/
					DesireSpeed_Write(0);

					while (state)
					{
						data->missionData.loopTime = timeCheck(&time);
						switch (state)
						{
						case FRONT_DETECT:
							/* ?û•?ï†Î¨? Ï¢åÏö∞?åê?ã®?ùÑ ?úÑ?ïú Ïπ¥Î©î?ùº Í∞ÅÎèÑÏ°∞Ï†à */
							sprintf(data->imgData.missionString, "Front Detect");
							if (data->missionData.overtakingData.headingDirection == STOP)
							{
								data->controlData.cameraY = 1610;
								CameraYServoControl_Write(data->controlData.cameraY);
								data->missionData.overtakingData.updownCamera = CAMERA_UP;
							}
							/* ?û•?ï†Î¨? Ï¢åÏö∞ ?åê?ã® Î∞? ÎπÑÏñ¥?ûà?äî Ï∞®ÏÑ†?úºÎ°? ?†ÑÏßÑÌïò?†§?äî ÏΩîÎìú*/
							while (data->missionData.overtakingData.headingDirection == STOP)
							{
								data->missionData.loopTime = timeCheck(&time);
								usleep(50000);
							}
							/*?åê?ã® Î∞õÏúºÎ©? Camera ?õê?ûò ?úÑÏπòÎ°ú ?èåÎ¶?*/
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
							/*?åê?ã® ?ù¥?õÑ ?ï¥?ãπ Î∞©Ìñ• ?†ÑÏß?*/
							if (data->missionData.overtakingData.headingDirection == RIGHT &&
								data->missionData.overtakingData.updownCamera == CAMERA_DOWN)
							{
								sprintf(data->imgData.missionString, "Right to go");
								/*Ï∂úÎ∞ú*/
								Winker_Write(RIGHT_ON);
								DesireDistance(50, thresDistance, 1100);
								Winker_Write(ALL_OFF);
								/* ?Ñº?Ñú ?ò§Î•? ?ù∏?ãù Î∞©Ï??*/
								usleep(500000);
								/*thresDistance?ù¥?ÉÅ Í∞??Ñú ?†ÑÎ∞? Í±∞Î¶¨ ?û¨?ôï?ù∏*/
								if (DistanceSensor_cm(1) < 20)
								{
									sprintf(data->imgData.missionString, "Detect Error");
									/*?†ïÏß?, ?õÑÏß? Î∞? Î∞©Ìñ• ?†Ñ?ôò*/
									DesireDistance(-50, thresDistance, 1100);
									/*?†ïÏß? Î∞? Î∞©Ìñ• ?†Ñ?ôò Î™ÖÎ†π*/
									data->missionData.overtakingData.headingDirection = LEFT;
								}
								else
								{ /*?†ÑÎ∞? ÎØ∏ÌÉêÏß?*/
									state = SIDE_ON;
									sprintf(data->imgData.missionString, "Detect Side");
									/*?†ÑÏßÑÌïò?äî ?èô?ïà ?†ÑÎ∞? ?Ñº?ÑúÍ∞? 20 ?ù¥?ÉÅ Î©??ñ¥Ïß?Î©? SIDE_ON?úºÎ°? ÏßÑÌñâ*/
									DesireSpeed_Write(BASIC_SPEED);
								}
							}
							else if (data->missionData.overtakingData.headingDirection == LEFT &&
									 data->missionData.overtakingData.updownCamera == CAMERA_DOWN)
							{

								sprintf(data->imgData.missionString, "Left to go");
								/*Ï∂úÎ∞ú*/
								Winker_Write(LEFT_ON);
								DesireDistance(50, thresDistance, 1900);
								Winker_Write(ALL_OFF);
								/* ?Ñº?Ñú ?ò§Î•? ?ù∏?ãù Î∞©Ï??*/
								usleep(500000);
								/*thresDistance?ù¥?ÉÅ Í∞??Ñú ?†ÑÎ∞? Í±∞Î¶¨ ?û¨?ôï?ù∏*/
								if (DistanceSensor_cm(1) < 20)
								{
									/*?†ïÏß?, ?õÑÏß? Î∞? Î∞©Ìñ• ?†Ñ?ôò*/
									sprintf(data->imgData.missionString, "Detect Error");
									DesireDistance(-50, thresDistance, 1900);
									/*?†ïÏß? ?õÑ Î∞©Ìñ• ?†Ñ?ôò Î™ÖÎ†π*/
									data->missionData.overtakingData.headingDirection = RIGHT;
								}
								else
								{
									/*?†ÑÏßÑÌïò?äî ?èô?ïà ?†ÑÎ∞? ?Ñº?ÑúÍ∞? 20 ?ù¥?ÉÅ Î©??ñ¥Ïß?Î©? SIDE_ON?úºÎ°? ÏßÑÌñâ*/
									state = SIDE_ON;
									sprintf(data->imgData.missionString, "Detect Side");
									DesireSpeed_Write(BASIC_SPEED);
								}
							}
							else
							{ /*STOP?ù¥ ?ú†Ïß??êò?äî Í≤ΩÏö∞ Î©àÏ∂§*/
							}

							break;

						case SIDE_ON:
							/*Auto Steering ?èô?ûë*/
							data->imgData.bmission = false;
							/* ?òÑ?û¨ ?û•?ï†Î¨ºÏù¥ ?ñ¥?îî?ûà?äê?Éê?óê ?î∞?ùº side ?Ñº?Ñú(2,3 or 4,5)Î°? Í∞êÏ???ïò?äî ÏΩîÎìú*/
							//right
							if (data->missionData.overtakingData.headingDirection == RIGHT)
							{
								/*?û•?ï†Î¨? ?ÜµÍ≥? ?ôï?ù∏*/
								if (DistanceSensor_cm(5) < 30 || DistanceSensor_cm(6) < 30)
								{
									obstacle = true;
								}
								else if (obstacle == true)
								{
									/*?û•?ï†Î¨? ?ÜµÍ≥?*/
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
								/*?û•?ï†Î¨? ?ÜµÍ≥? ?ôï?ù∏*/
								if (DistanceSensor_cm(3) < 30 || DistanceSensor_cm(2) < 30)
								{
									obstacle = true;
								}
								else if (obstacle == true)
								{
									/*?û•?ï†Î¨? ?ÜµÍ≥?*/
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
							/*?õê?ûò Ï∞®ÏÑ†?úºÎ°? Î≥µÍ???ïò?äî ÏΩîÎìú*/
							usleep(10000);
							data->imgData.bmission = true; //Auto Steering off
							//right
							if (data->missionData.overtakingData.headingDirection == RIGHT)
							{
								/*Î≥µÍ?? Ï¢åÌöå?†Ñ Î∞©Ìñ• ?Ñ§?†ï Î∞? ?†ÑÏß?*/
								Winker_Write(LEFT_ON);
								DesireDistance(50, thresDistance + 100, 1900);
								Winker_Write(ALL_OFF);
							}
							//left
							else if (data->missionData.overtakingData.headingDirection == LEFT)
							{
								/*Î≥µÍ?? ?ö∞?öå?†Ñ Î∞©Ìñ• ?Ñ§?†ï*/
								Winker_Write(RIGHT_ON);
								DesireDistance(50, thresDistance + 100, 1100);
								Winker_Write(ALL_OFF);
							}
							/*?ïåÍ≥†Î¶¨Ï¶? ?†ÑÏß?*/
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
						usleep(50000); // 1,500 ms -> 50ms Î°? Î≥?Í≤?, 09/01 AM 00:50 -KDH
					}
					signalLight = READY;
					data->imgData.bmission = false;
					data->imgData.bprintString = false;
				}
			}
		}

		if (signalLight && signalLight != DONE)
		{
			data->imgData.bcheckFrontWhite = true; // ?†ÑÎ∞? ?†ïÏß??Ñ† ?òÅ?ÉÅ?úºÎ°? ?ÉêÏß? ON

			if (/*StopLine(5) ||*/ data->missionData.finish_distance != -1) //?òÅ?ÉÅ?úºÎ°? ?†ïÏß??Ñ†?ùò Í±∞Î¶¨Í∞? Ï∏°Ï†ï?êòÎ©?
			{
				onlyDistance(BASIC_SPEED, (data->missionData.finish_distance / 26.0) * 500); //?†ïÏß??Ñ†ÍπåÏ?? Í∞??Ñú stop
				data->missionData.finish_distance = -1;

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
					usleep(200000); //?òÅ?ÉÅÏ≤òÎ¶¨?óê?Ñú ?ùº?†®?ùò Í≥ºÏ†ï?ù¥ ?Åù?Ç† ?ïå ÍπåÏ?? Í∏∞Îã§Î¶∞Îã§.

				data->imgData.bprintTire = true;
				sprintf(data->imgData.missionString, "Distance control");
				DesireSpeed_Write(BASIC_SPEED);

				bool once_back = false;
				while (1) // ?ã†?ò∏?ì± Íµ¨Ï°∞Î¨ºÍ≥º?ùò Í±∞Î¶¨Î•? 23,24cm Î°? ÎßûÏ∂îÍ∏? ?úÑ?ï¥ ?†ÑÏß?
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
			// data->missionData.finishData.checkFront = false; /*ÎπÑÌôú?Ñ±?ôî*/
			// if (1)											 /*?Ö∏????Éâ Í∞?Î°? ÏßÅÏÑ†?ù¥ ?ùº?†ï?ù¥?ïòÎ°? ?ñ®?ñ¥Ïß?Î©? ?ûÖ?†•*/
			// {												 //Encoder ?Ç¨?ö©?ï¥?Ñú ?ùº?†ï ÏßÅÏßÑ?ïòÎ©? Ï¢ÖÎ£å?ïòÍ≤? ?Ñ§?†ï
			// 	//?Åù?ÇòÍ≥? ?Çê?ÜåÎ¶?
			// 	/*?ù¥ÎØ? ?ù¥?èô ?ÉÅ?Éú*/
			// 	data->missionData.finishData.distEndLine = -1000;
			// 	data->imgData.bmission = true;
			// 	data->imgData.bprintString = true;
			// 	/*box filtering*/
			// 	data->missionData.finishData.checkFront = true;/*?†ÑÎ∞? ?Ö∏????ùº?ù∏ ?ÉêÏß? ?ôú?Ñ±?ôî*/
			// 	/*encoding?ùÑ ?ù¥?ö©?ïú ?†ÑÏß?*/
			// 	//data->missionData.finishData.encodingStart = false;
			// 	/*check front signal waiting*/
			// 	while (data->missionData.finishData.checkFront == true || data->missionData.finishData.distEndLine == -1000)
			// 	{
			// 		usleep(500000);
			// 		/*checkFront Í∞? falseÍ∞? ?êò?ñ¥ Ï¢ÖÎ£å ?êêÍ±∞ÎÇò distEndlineÍ∞íÏù¥ Î¨¥ÏùòÎØ∏ÌïòÏß? ?ïä?ùÑÍ≤ΩÏö∞ Ï¢ÖÎ£å*/
			// 		if (data->missionData.finishData.checkFront == false || data->missionData.finishData.distEndLine > 320)
			// 		{
			// 			sprintf(data->imgData.missionString, "End Check Front");
			// 			break; /*?ïû?óê ?ÉêÏß??ãú Ï¢ÖÎ£å*/
			// 		}
			// 		sprintf(data->imgData.missionString, "Check Front");
			// 	}
			// 	/*?çî?ù¥?ÉÅ ?ôï?ù∏?ïòÏß? ?ïä?èÑÎ°? Ï¢ÖÎ£å(double check)*/
			// 	data->missionData.finishData.checkFront = false;
			// 	DesireSpeed_Write(0);
			// 	/*?ù¥?èô ?õÑ Ï¢ÖÎ£å*/
			// 	Winker_Write(ALL_ON);
			// 	usleep(1000000);
			// 	Winker_Write(ALL_OFF);
			// 	/*Î∞ëÏóê ?ù∞?Éâ?ù¥ ?ïò?Çò?ùº?èÑ ?ÉêÏß??êò?äîÏß? ?ôï?ù∏ ?õÑ ?ûà?ã§Î©? ?†ïÏß?*/
			// 	///Ï∂îÍ???ïÑ?öî/////
			// }

			if (1) //Î¨¥Ï°∞Í±? ÏßÑÏûÖ
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
				DesireDistance(40, 500 * (rest_distance / 26.0), 1500); // encoder = 500 -> 26cmÎ°? Ï∏°Ï†ï

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
	char *disp_argv[] = {"dummy", "-s", "4:480x272", "\0"}; // Ï∂îÌõÑ Î≥?Í≤? ?ó¨Î∂? ?ôï?ù∏ ?õÑ Ï≤òÎ¶¨..
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
	// vpe Íµ¨Ï°∞Ï≤? ?Éù?Ñ± Î∞? Ï¥àÍ∏∞?ôî
	// vpe input (v4l cameradata)
	vpe->src.width = CAPTURE_IMG_W;
	vpe->src.height = CAPTURE_IMG_H;
	describeFormat(CAPTURE_IMG_FORMAT, &vpe->src);
	// ?ûÖ?†• ?ù¥ÎØ∏Ï???óê ????ïú ?åå?ùºÎØ∏ÌÑ∞(?Ç¨?ù¥Ï¶? Î∞? ?è¨Îß?)Î•? vpe ?ûÖ?†• ?ù¥ÎØ∏Ï?? Î©§Î≤Ñ?óê ?ï†?ãπ?ïú?ã§.

	// vpe output (disp data)
	vpe->dst.width = VPE_OUTPUT_W;
	vpe->dst.height = VPE_OUTPUT_H;
	describeFormat(VPE_OUTPUT_FORMAT, &vpe->dst);
	// Ï∂úÎ†• ?ù¥ÎØ∏Ï???óê ????ïú ?åå?ùºÎØ∏ÌÑ∞(?Ç¨?ù¥Ï¶? Î∞? ?è¨Îß?)Î•? vpe Ï∂úÎ†• ?ù¥ÎØ∏Ï?? Î©§Î≤Ñ?óê ?ï†?ãπ?ïú?ã§.

	vpe->disp = disp_open(disp_argc, disp_argv);
	if (!vpe->disp)
	{
		ERROR("disp open error!");
		vpe_close(vpe);
		return 1;
	}
	// ?òÅ?ÉÅ Ï∂úÎ†•?ùÑ ?úÑ?ï¥ vpe?ùò display Î©§Î≤Ñ Íµ¨Ï°∞Ï≤? Ï¥àÍ∏∞?ôî

	set_z_order(vpe->disp, vpe->disp->overlay_p.id);
	set_global_alpha(vpe->disp, vpe->disp->overlay_p.id);
	set_pre_multiplied_alpha(vpe->disp, vpe->disp->overlay_p.id);
	alloc_overlay_plane(vpe->disp, OVERLAY_DISP_FORCC, 0, 0, OVERLAY_DISP_W, OVERLAY_DISP_H);
	// z-order, alpha, multiplied-alpha ?Ñ§?†ï (overlayÎ•? ?úÑ?ïú plane Í∞? ?Ñ§?†ï)

	//vpe->deint = 0;
	vpe->translen = 1;

	MSG("Input(Camera) = %d x %d (%.4s)\nOutput(LCD) = %d x %d (%.4s)",
		vpe->src.width, vpe->src.height, (char *)&vpe->src.fourcc,
		vpe->dst.width, vpe->dst.height, (char *)&vpe->dst.fourcc);
	// ?ûÖÏ∂úÎ†• ?ù¥ÎØ∏Ï???ùò ?Å¨Í∏? Î∞? ?è¨Îß∑Ï†ïÎ≥? Ï∂úÎ†•
	// ?ûÖ?†• ?ù¥ÎØ∏Ï?? : 1280x720, Format = UYUV422
	// Ï∂úÎ†• ?ù¥ÎØ∏Ï?? : 320x180. Format = BGR24

	if (vpe->src.height < 0 || vpe->src.width < 0 || vpe->src.fourcc < 0 ||
		vpe->dst.height < 0 || vpe->dst.width < 0 || vpe->dst.fourcc < 0)
	{
		ERROR("Invalid parameters\n");
	}

	v4l2 = v4l2_open(vpe->src.fourcc, vpe->src.width, vpe->src.height);
	// ?ù¥ÎØ∏Ï?? Ï∫°Ï≥êÎ•? ?úÑ?ï¥ vpe Íµ¨Ï°∞Ï≤¥Î?? Î∞îÌÉï?úºÎ°? v412 Íµ¨Ï°∞Ï≤? Ï¥àÍ∏∞?ôî

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
	// signal error Í≤?Ï∂?

	pause();

	return ret;
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
	ptr_data->controlData.steerVal = angle; //?ò§Î≤ÑÎ†à?ù¥ ?ó∞?èô

	SteeringServo_Write_uart(angle);
}