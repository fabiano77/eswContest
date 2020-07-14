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

#define NBUF 6

#define SCREEN_W            480
#define SCREEN_H            272

#define VPE_IMG_FORMAT      "rgba32"
#define VPE_IMG_SIZE        (SCREEN_W*SCREEN_H*4)   // rgba32

#define SCREEN_DUMP_FORMAT  "bgr24"
#define SCREEN_DUMP_SIZE    (SCREEN_W*SCREEN_H*3)   // bgr24

#define CAPTURE_IMG_W       1280
#define CAPTURE_IMG_H       720
#define CAPTURE_IMG_SIZE    (CAPTURE_IMG_W*CAPTURE_IMG_H*2) // YUYU : 16bpp
#define CAPTURE_IMG_FORMAT  "uyvy"

#define OVERLAY_DISP_W          SCREEN_W
#define OVERLAY_DISP_H          SCREEN_H
#define OVERLAY_DISP_FORCC      FOURCC('A','R','2','4')     // argb32
#define OVERLAY_DISP_SIZE        (OVERLAY_DISP_W*OVERLAY_DISP_H*4)   // argb32

#define DUMP_MSGQ_KEY           1020
#define DUMP_MSGQ_MSG_TYPE      0x02

typedef enum {
    DUMP_NONE,
    DUMP_CMD,
    DUMP_READY,
    DUMP_WRITE_TO_FILE,
    DUMP_DONE
}DumpState;

typedef struct _DumpMsg{
    long type;
    int  state_msg;
}DumpMsg;

struct thr_data {
    struct display *disp;
    struct v4l2 *v4l2;

    unsigned char* screen_fb;
    int            screen_size;

    DumpState dump_state;
    unsigned char dump_img_data[CAPTURE_IMG_SIZE];
    unsigned char dump_screen_data[SCREEN_DUMP_SIZE];
    unsigned char dump_overlay_data[OVERLAY_DISP_SIZE];
    bool dump_screen;

    int msgq_id;
    bool bfull_screen;
    bool bstream_start;
    pthread_t threads[3];
};


static uint32_t getfourccformat (char *format)
{
    if (strcmp (format, "rgb24") == 0) {
        return FOURCC('R','G','2','4');
    } else if (strcmp (format, "bgr24") == 0) {
        return FOURCC('B','G','2','4');
    } else if (strcmp (format, "argb32") == 0) {
        return FOURCC('A','R','2','4');
    } else if (strcmp (format, "abgr32") == 0) {
        return FOURCC('R','A','2','4');
    } else if (strcmp (format, "yuyv") == 0) {
        return FOURCC('Y','U','Y','V');
    } else if (strcmp (format, "uyvy") == 0) {
        return FOURCC('U','Y','V','Y');
    } else if (strcmp (format, "nv12") == 0) {
        return FOURCC('N','V','1','2');
    } else {
        MSG("not yet support format..");
    }

    return 0;
}

/**
  * @brief  Capture and display of camera
  * @param  arg: pointer to parameter of thr_data
  * @retval none
  */
void * capture_thread(void *arg)
{
    struct thr_data *data = (struct thr_data *)arg;
    struct display *disp = data->disp;
    struct v4l2 *v4l2 = data->v4l2;
    struct buffer **buffers, *capt;
    int ret, i;

    buffers = disp_get_vid_buffers(disp, NBUF, getfourccformat(CAPTURE_IMG_FORMAT), CAPTURE_IMG_W, CAPTURE_IMG_H);
    if (!buffers) {
        return NULL;
    }

    ret = v4l2_reqbufs(v4l2, buffers, NBUF);
    if (ret) {
        return NULL;
    }

    for (i = 0; i < NBUF; i++) {
        v4l2_qbuf(v4l2, buffers[i]);
    }

    ret = v4l2_streamon(v4l2);
    if (ret) {
        return NULL;
    }

    MSG("streaming started...");
    data->bstream_start = true;

    while(true) {
        capt = v4l2_dqbuf(v4l2);
        if(data->bfull_screen)
            capt->noScale = false;
        else
            capt->noScale = true;
        ret = disp_post_vid_buffer(disp, capt, 0, 0, CAPTURE_IMG_W, CAPTURE_IMG_H);
        if (ret) {
            ERROR("Post buffer failed");
            return NULL;
        }
        if(data->dump_state == DUMP_READY) {
            DumpMsg dumpmsg;
            unsigned char* pbuf[4];

            if(get_framebuf(capt, pbuf) == 0) {
                switch(capt->fourcc) {
                    case FOURCC('U','Y','V','Y'):
                    case FOURCC('Y','U','Y','V'):
                        memcpy(data->dump_img_data, pbuf[0], CAPTURE_IMG_SIZE);
                        break;
                    default :
                        MSG("DUMP.. not yet support format : %.4s\n", (char*)&capt->fourcc);
                        break;
                }
            } else {
                MSG("dump capture buf fail !");
            }

            dumpmsg.type = DUMP_MSGQ_MSG_TYPE;
            dumpmsg.state_msg = DUMP_WRITE_TO_FILE;
            data->dump_state = DUMP_WRITE_TO_FILE;
            if (-1 == msgsnd(data->msgq_id, &dumpmsg, sizeof(DumpMsg)-sizeof(long), 0)) {
                MSG("state:%d, msg send fail\n", dumpmsg.state_msg);
            }
        }

        v4l2_qbuf(v4l2, capt);
    }
    v4l2_streamoff(v4l2);

    return NULL;
}

/**
  * @brief  Make screen dump image. using vpe and opencv
  * @param  arg: pointer to parameter of thr_data
  * @retval none
  */
static int makescreendata(struct thr_data *data)
{
    struct vpe *vpe;
    struct buffer *input_bufs;
    struct buffer *output_bufs;
    unsigned char *input_img;
    unsigned char *output_img;
    unsigned char* pbuf[4];

    //scale & convert with vpe
    // open vpe
    vpe = vpe_open();
    if(!vpe) {
        return -1;
    }

    vpe->disp = data->disp;
    // vpe input (v4l cameradata)
    vpe->src.width  = CAPTURE_IMG_W;
    vpe->src.height = CAPTURE_IMG_H;
    describeFormat(CAPTURE_IMG_FORMAT, &vpe->src);

    // vpe output (disp data)
    vpe->dst.width  = SCREEN_W;
    vpe->dst.height = SCREEN_H;
    describeFormat(VPE_IMG_FORMAT, &vpe->dst);

    input_bufs = alloc_buffer(vpe->disp, vpe->src.fourcc, vpe->src.width, vpe->src.height, false);
    output_bufs = alloc_buffer(vpe->disp, vpe->dst.fourcc, vpe->dst.width, vpe->dst.height, false);

    if (!input_bufs || !output_bufs)
        ERROR("allocating vpe input/output buffer failed\n");

    vpe->input_buf_dmafd[0] = omap_bo_dmabuf(input_bufs->bo[0]);
    input_bufs->fd[0] = vpe->input_buf_dmafd[0];

    vpe->output_buf_dmafd[0] = omap_bo_dmabuf(output_bufs->bo[0]);
    output_bufs->fd[0] = vpe->output_buf_dmafd[0];

    input_img = (unsigned char*)omap_bo_map(input_bufs->bo[0]);
    memcpy(input_img, data->dump_img_data, CAPTURE_IMG_SIZE);
    output_img = (unsigned char*)omap_bo_map(output_bufs->bo[0]);

    vpe_input_init(vpe);
    vpe_output_init(vpe);

    vpe_output_qbuf(vpe, 0);
    vpe_stream_on(vpe->fd, V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE);

    vpe_input_qbuf(vpe, 0);
    vpe_stream_on(vpe->fd, V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE);

    vpe_output_dqbuf(vpe);
    vpe_input_dqbuf(vpe);

    vpe_stream_off(vpe->fd, V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE);
    vpe_stream_off(vpe->fd, V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE);

    get_framebuf(data->disp->overlay_p_bo, pbuf);

    memcpy(data->dump_overlay_data, pbuf[0], OVERLAY_DISP_SIZE);

    // src1 : output_img : SCREEN_W*SCREEN_H camera image. 
    // src2 : overlay(draw line, text, rect) image
    // dst : merge image
    OpenCV_merge_image(output_img, data->dump_overlay_data, data->dump_screen_data, SCREEN_W, SCREEN_H);

    if (input_bufs) {
        close(input_bufs->fd[0]);
        omap_bo_del(input_bufs->bo[0]);
        if(vpe->src.coplanar){
            close(input_bufs->fd[1]);
            omap_bo_del(input_bufs->bo[1]);
        }
    }

    if (output_bufs) {
        close(output_bufs->fd[0]);
        omap_bo_del(output_bufs->bo[0]);
        if(vpe->dst.coplanar){
            close(output_bufs->fd[1]);
            omap_bo_del(output_bufs->bo[1]);
        }
    }

    vpe_close(vpe);

    return 0;
}

/**
  * @brief  Capture image dump / Screen image dump and save to file
  * @param  arg: pointer to parameter of thr_data
  * @retval none
  */
void * capture_dump_thread(void *arg)
{
    struct thr_data *data = (struct thr_data *)arg;
    FILE *fp;
    char filename[50];
    struct timeval timestamp;
    struct tm *today;
    DumpMsg dumpmsg;

    while(1) {
        if(msgrcv(data->msgq_id, &dumpmsg, sizeof(DumpMsg)-sizeof(long), DUMP_MSGQ_MSG_TYPE, 0) >= 0) {
            switch(dumpmsg.state_msg) {
                case DUMP_CMD :
                    gettimeofday(&timestamp, NULL);
                    today = localtime(&timestamp.tv_sec);

                    memset(filename, 0, sizeof(filename));
                    if(data->dump_screen)
                        sprintf(filename, "dump_%04d%02d%02d_%02d%02d%02d.%s", today->tm_year+1900, today->tm_mon+1, today->tm_mday, today->tm_hour, today->tm_min, today->tm_sec, SCREEN_DUMP_FORMAT);
                    else
                        sprintf(filename, "dump_%04d%02d%02d_%02d%02d%02d.%s", today->tm_year+1900, today->tm_mon+1, today->tm_mday, today->tm_hour, today->tm_min, today->tm_sec, CAPTURE_IMG_FORMAT);

                    data->dump_state = DUMP_READY;
                    MSG("file name:%s", filename);
                    break;

                case DUMP_WRITE_TO_FILE :
                    if((fp = fopen(filename, "w+")) == NULL){
                        ERROR("Fail to fopen");
                    } else {
                        if(data->dump_screen) {
                            makescreendata(data);
                            fwrite(data->dump_screen_data, SCREEN_DUMP_SIZE, 1, fp);
                        }
                        else
                            fwrite(data->dump_img_data, CAPTURE_IMG_SIZE, 1, fp);
                    }
                    fclose(fp);
                    data->dump_state = DUMP_DONE;
                    break;

                default :
                    MSG("dump msg wrong (%d)", dumpmsg.state_msg);
                    break;
            }
        }
    }

    return NULL;
}

/**
  * @brief  handling an input command
  * @param  arg: pointer to parameter of thr_data
  * @retval none
  */
void * input_thread(void *arg)
{
    struct thr_data *data = (struct thr_data *)arg;
    struct display *disp = data->disp;

    char cmd_input[256];
    char cmd_ready = true;

    while(!data->bstream_start) {
        usleep(100*1000);
    }

    MSG("\n\nInput command:");
    MSG("\t dump");
    MSG("\t\t - display image(%s, %dx%d) dump",CAPTURE_IMG_FORMAT,CAPTURE_IMG_W,CAPTURE_IMG_H);
    MSG("\t sdump");
    MSG("\t\t - Screen (%s, %dx%d) dump",SCREEN_DUMP_FORMAT,SCREEN_W,SCREEN_H);
    MSG("\t drawP:x,y:color");
    MSG("\t\t - draw point, ex) drawP:10,10:0xffff0000");
    MSG("\t drawL:x1,y1:x2,y2:color");
    MSG("\t\t - draw line, ex) drawL:4,4:100,2:0xff00ff00");
    MSG("\t drawR:x,y:w,h:color");
    MSG("\t\t - draw Rect, ex) drawR:12,12:20,40:0xff0000ff");
    MSG("\t drawT:x,y:color:string");
    MSG("\t\t - draw text, ex) drawT:100,100:0xffffffff:Draw test");
    MSG("\t drawC");
    MSG("\t\t - draw disp clear");
    MSG("\n");

    while(1)
    {
        if(cmd_ready == true) {
            /*standby to input command */
            cmd_ready = StandbyInput(cmd_input);     //define in cmd.cpp
        } else {
            if(0 == strncmp(cmd_input,"dump",4)) {
                DumpMsg dumpmsg;
                dumpmsg.type = DUMP_MSGQ_MSG_TYPE;
                dumpmsg.state_msg = DUMP_CMD;
                data->dump_state = DUMP_CMD;

                if (-1 == msgsnd(data->msgq_id, &dumpmsg, sizeof(DumpMsg)-sizeof(long), 0)) {
                    printf("dump cmd msg send fail\n");
                } else {
                    printf("dump cmd msg send\n");
                }
                while(data->dump_state != DUMP_DONE) {
                    usleep(5*1000);
                }
                data->dump_state = DUMP_NONE;
                printf("dump done!\n");
            } else if(0 == strncmp(cmd_input,"sdump",5)) {
                DumpMsg dumpmsg;

                data->dump_screen = true;
                dumpmsg.type = DUMP_MSGQ_MSG_TYPE;
                dumpmsg.state_msg = DUMP_CMD;
                data->dump_state = DUMP_CMD;

                if (-1 == msgsnd(data->msgq_id, &dumpmsg, sizeof(DumpMsg)-sizeof(long), 0)) {
                    printf("sdump cmd msg send fail\n");
                } else {
                    printf("sdump cmd msg send\n");
                }
                while(data->dump_state != DUMP_DONE) {
                    usleep(5*1000);
                }
                data->dump_state = DUMP_NONE;
                data->dump_screen = false;
                printf("sdump done!\n");
            } else if(0 == strncmp(cmd_input,"draw",4)) {
                FrameBuffer tmpFrame;
                unsigned char* pbuf[4];
                char drawcmd[12];
                char drawtxt[24];
                int type = 0;
                uint32_t x1, y1, x2, y2, w, h, color;
                memset(&tmpFrame, 0, sizeof(FrameBuffer));

                if(0 == strncmp(cmd_input,"drawP",5)) {
                    sscanf(cmd_input, "%5s:%d,%d:0x%08x", drawcmd, &x1, &y1,&color);
                    printf("drawcmd:%s x1:%d,y1:%d, color:0x%x\n", drawcmd, x1, y1, color);
                    type = 0; //draw pointer
                } else if(0 == strncmp(cmd_input,"drawL",5)) {
                    sscanf(cmd_input, "%5s:%d,%d:%d,%d:0x%08x", drawcmd, &x1, &y1,&x2,&y2,&color);
                    printf("drawcmd:%s x1:%d,y1:%d,x2:%d,y2:%d,color:0x%x\n", drawcmd, x1, y1, x2, y2, color);
                    type = 1; //draw line
                } else if(0 == strncmp(cmd_input,"drawR",5)) {
                    sscanf(cmd_input, "%5s:%d,%d:%d,%d:0x%08x", drawcmd, &x1, &y1,&w,&h,&color);
                    printf("drawcmd:%s x:%d,y:%d,w:%d,h:%d,color:0x%x\n", drawcmd, x1, y1, w, h, color);
                    type = 2; //draw rect
                } else if(0 == strncmp(cmd_input,"drawT",5)) {
                    sscanf(cmd_input, "%5s:%d,%d:0x%08x:%[^\n]", drawcmd, &x1, &y1,&color, drawtxt);
                    printf("drawcmd:%s x:%d,y:%d,s:%s,color:0x%x\n", drawcmd, x1, y1, drawtxt, color);
                    type = 3; //draw string
                } else if(0 == strncmp(cmd_input,"drawC",5)) {
                    type = 4; // clear
                } else {
                    cmd_ready = true;
                    continue;
                }

                if(get_framebuf(disp->overlay_p_bo, pbuf) == 0) {
                    tmpFrame.buf = pbuf[0];
                    tmpFrame.format = draw_get_pixel_foramt(disp->overlay_p_bo->fourcc);//FORMAT_RGB888; //alloc_overlay_plane() -- FOURCC('R','G','2','4');
                    tmpFrame.stride = disp->overlay_p_bo->pitches[0];//tmpFrame.width*3;

                    switch(type) {
                        case 0: // draw pointer
                            drawPixel(&tmpFrame, x1, y1, color);
                            break;
                        case 1: // draw line
                            drawLine(&tmpFrame, x1, y1, x2, y2, color);
                            break;
                        case 2: // draw rect
                            drawRect(&tmpFrame, x1, y1, w, h, color);
                            break;
                        case 3: // draw string
                            drawString(&tmpFrame, drawtxt, x1, y1, 0, color);
                            break;

                        case 4 : // draw clear
                            memset(tmpFrame.buf, 0, disp->overlay_p.yres*tmpFrame.stride);
                            break;

                        default :
                            break;
                    }
                }

                update_overlay_disp(disp);

            } else {
                printf("cmd_input:%s \n", cmd_input);
            }
            cmd_ready = true;
        }
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
    MSG("signal_hanlde\n");

    if(sig == SIGINT) {
        pthread_cancel(pexam_data->threads[0]);
        pthread_cancel(pexam_data->threads[1]);
        pthread_cancel(pexam_data->threads[2]);
        
        msgctl(pexam_data->msgq_id, IPC_RMID, 0);
        
        v4l2_streamoff(pexam_data->v4l2);
        disp_free_buffers(pexam_data->disp, NBUF);
        free_overlay_plane(pexam_data->disp);
        
        disp_close(pexam_data->disp);
        v4l2_close(pexam_data->v4l2);
        
        MSG("-- 3_camera_overlay_draw_disp example End --\n");
    }
}


int main(int argc, char **argv)
{
    struct display *disp;
    struct v4l2 *v4l2;
    struct thr_data tdata;
    int disp_argc = 3;
    char* disp_argv[] = {"dummy", "-s", "4:480x272", "\0"};
    int ret = 0;
    uint32_t fourcc;
    printf("-- 3_camera_overlay_draw_disp example Start --\n");

    tdata.dump_state = DUMP_NONE;
    memset(tdata.dump_img_data, 0, sizeof(tdata.dump_img_data));

    disp = disp_open(disp_argc, disp_argv);
    if (!disp) {
        ERROR("disp open error!");
        return 1;
    }

    set_z_order(disp, disp->overlay_p.id);
    set_global_alpha(disp, disp->overlay_p.id);
    set_pre_multiplied_alpha(disp, disp->overlay_p.id);
    alloc_overlay_plane(disp, OVERLAY_DISP_FORCC, 0, 0, OVERLAY_DISP_W, OVERLAY_DISP_H);

    fourcc = getfourccformat(CAPTURE_IMG_FORMAT);
    v4l2 = v4l2_open(fourcc, CAPTURE_IMG_W, CAPTURE_IMG_H);
    if (!v4l2) {
        ERROR("v4l2 open error!");
        disp_close(disp);
        return 1;
    }

    MSG ("Input(Camera) = %d x %d (%.4s)\nOutput(LCD) = %d x %d (%.4s)",
        CAPTURE_IMG_W, CAPTURE_IMG_H, (char*)&fourcc,
        CAPTURE_IMG_W, CAPTURE_IMG_H, (char*)&fourcc);

    tdata.disp = disp;
    tdata.v4l2 = v4l2;
    tdata.bfull_screen = true;
    tdata.bstream_start = false;
    tdata.dump_screen = false;

    if(-1 == (tdata.msgq_id = msgget((key_t)DUMP_MSGQ_KEY, IPC_CREAT | 0666))) {
        fprintf(stderr, "%s msg create fail!!!\n", __func__);
        return -1;
    }

    pexam_data = &tdata;

    ret = pthread_create(&tdata.threads[0], NULL, capture_thread, &tdata);
    if(ret) {
        MSG("Failed creating capture thread");
    }
    pthread_detach(tdata.threads[0]);

    ret = pthread_create(&tdata.threads[1], NULL, capture_dump_thread, &tdata);
    if(ret) {
        MSG("Failed creating capture dump thread");
    }
    pthread_detach(tdata.threads[1]);

    ret = pthread_create(&tdata.threads[2], NULL, input_thread, &tdata);
    if(ret) {
        MSG("Failed creating input thread");
    }
    pthread_detach(tdata.threads[2]);


    /* register signal handler for <CTRL>+C in order to clean up */
    if(signal(SIGINT, signal_handler) == SIG_ERR) {
        MSG("could not register signal handler");
        closelog();
        exit(EXIT_FAILURE);
    }

    pause();

    return ret;
}
