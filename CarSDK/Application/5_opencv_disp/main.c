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
#include "input_cmd.h"
#include "drawing.h"
#include "exam_cv.h"
#include "jpeg_utils.h"

#define DISP_OUTPUT_IMG_W       640
#define DISP_OUTPUT_IMG_H       480
#define DISP_OUTPUT_IMG_SIZE    (DISP_OUTPUT_IMG_W*DISP_OUTPUT_IMG_H*3) // RGB : 24bpp - bgr..
#define DISP_OUTPUT_IMG_FORMAT  "rgb24"
#define DISP_OUTPUT_BUF_NUM     1

#define OPENCV_MSGQ_KEY         1020
#define OPENCV_MSGQ_MSG_TYPE    0x02

#define IMAGE_SAMPLE_1          "./images/lena_640x480.jpg"
#define IMAGE_SAMPLE_2          "./images/sunglass.jpg"

#define OVERLAY_DISP_FORCC      FOURCC('A','R','2','4')
#define OVERLAY_DISP_W          480
#define OVERLAY_DISP_H          272

#define TIME_TEXT_X             385 //320
#define TIME_TEXT_Y             260 //240
#define TIME_TEXT_COLOR         0xffffffff //while

#define DUMP_MSGQ_KEY           1020
#define DUMP_MSGQ_MSG_TYPE      0x02

typedef enum {
    OPENCV_MODE_1,  // Default
    OPENCV_MODE_2,  // Face Detection
    OPENCV_MODE_3,  // Binding Images
    OPENCV_MODE_4   // Canny Edge Detection
}OpenCVMode;

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
    struct buffer **output_bufs;

    DumpState dump_state;
    unsigned char dump_img_data[DISP_OUTPUT_IMG_SIZE];
    OpenCVMode opencv_mode;

    int msgq_id;
    bool bfull_screen;
    pthread_t threads[2];
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
  * @brief  Alloc ouput buffer and a new buffer object. It is used as a frame buffer to display
  * @param  data: pointer to parameter of thr_data
  * @retval none
  */
static int allocate_output_buffers(struct thr_data *data)
{
    int i;
    struct display *disp = data->disp;

    data->output_bufs = disp_get_vid_buffers(disp, DISP_OUTPUT_BUF_NUM, getfourccformat(DISP_OUTPUT_IMG_FORMAT), DISP_OUTPUT_IMG_W, DISP_OUTPUT_IMG_H);
    if (!data->output_bufs)
        ERROR("allocating shared buffer failed\n");

    for (i = 0; i < DISP_OUTPUT_BUF_NUM; i++) {
        data->output_bufs[i]->fd[0] = omap_bo_dmabuf(data->output_bufs[i]->bo[0]);
    }
    return 0;
}

/**
  * @brief  Free ouput buffer and destroy a buffer object
  * @param  buffer: pointer to parameter of buffer object
                  n : count of buffer object
                  bmultiplanar : multipanar value of buffer object
  * @retval none
  */
static void free_output_buffers(struct buffer **buffer, uint32_t n, bool bmultiplanar)
{
    uint32_t i;
    for (i = 0; i < n; i++) {
        if (buffer[i]) {
            close(buffer[i]->fd[0]);
            omap_bo_del(buffer[i]->bo[0]);
            if(bmultiplanar){
                close(buffer[i]->fd[1]);
                omap_bo_del(buffer[i]->bo[1]);
            }
        }
    }
    free(buffer);
}

static struct thr_data* pexam_data = NULL;

/**
  * @brief  handling an SIGINT(CTRL+C) signal
  * @param  sig: signal type
  * @retval none
  */
void signal_handler(int sig)
{
    if(sig == SIGINT) {
        pthread_cancel(pexam_data->threads[0]);
        pthread_cancel(pexam_data->threads[1]);
        
        msgctl(pexam_data->msgq_id, IPC_RMID, 0);
        
        free_output_buffers(pexam_data->output_bufs, DISP_OUTPUT_BUF_NUM, false);
        disp_close(pexam_data->disp);

        printf("-- 5_opencv_disp example End --\n");
    }
}

/**
  * @brief  Draw operating time to overlay buffer.
  * @param  disp: pointer to parameter of struct display
                  time : operate time (ms)
  * @retval none
  */
static void draw_operatingtime(struct display *disp, uint32_t time)
{
    FrameBuffer tmpFrame;
    unsigned char* pbuf[4];
    char strtime[128];

    memset(strtime, 0, sizeof(strtime));

    sprintf(strtime, "%04d(ms)", time);

    if(get_framebuf(disp->overlay_p_bo, pbuf) == 0) {
        tmpFrame.buf = pbuf[0];
        tmpFrame.format = draw_get_pixel_foramt(disp->overlay_p_bo->fourcc);//FORMAT_RGB888; //alloc_overlay_plane() -- FOURCC('R','G','2','4');
        tmpFrame.stride = disp->overlay_p_bo->pitches[0];//tmpFrame.width*3;

        drawString(&tmpFrame, strtime, TIME_TEXT_X, TIME_TEXT_Y, 0, TIME_TEXT_COLOR);
    }
}

/**
  * @brief  Update opencv display
  * @param  data: pointer to parameter of thr_data
  * @retval none
  */
static void cv_disp_update(struct thr_data* data)
{
    struct display *disp = data->disp;
    struct buffer *dispbuf;

    dispbuf = data->output_bufs[0];
    if(data->bfull_screen)
        dispbuf->noScale = false;
    else
        dispbuf->noScale = true;

    if (disp_post_vid_buffer(disp, dispbuf, 0, 0, DISP_OUTPUT_IMG_W, DISP_OUTPUT_IMG_H)) {
        ERROR("Post buffer failed");
        return;
    }

    update_overlay_disp(disp);
}

/**
  * @brief  Save dump image data to jpeg file.
  * @param  buf: pointer to parameter of dump image data
                  w : image width
                  h : image height
  * @retval none
  */
static void cv_savetojpeg(unsigned char* buf, int w, int h)
{
    struct timeval timestamp;
    struct tm *today;
    char jpgname[256];

    gettimeofday(&timestamp, NULL);
    today = localtime(&timestamp.tv_sec);
    sprintf(jpgname, "%04d%02d%02d_%02d%02d%02d.jpg", today->tm_year+1900, today->tm_mon+1, today->tm_mday, today->tm_hour, today->tm_min, today->tm_sec);

    OpenCV_Bgr2RgbConvert(buf, w, h, buf);
    compress_rgb24_to_jpeg(buf, w, h, 80, jpgname);
}

/**
  * @brief  Example handling with opencv api
  * @param  data: pointer to parameter of thr_data
                  filename1 : sample image file path
                  filename2 : sample2 image file path
                  mode : example mode
  * @retval none
  */
static void cv_exam(struct thr_data* data, char* filename1, char* filename2, OpenCVMode mode)
{
    struct buffer *dispbuf = data->output_bufs[0];
    unsigned char img[DISP_OUTPUT_IMG_SIZE];
    unsigned char* pbuf[4];
    unsigned char* fbuf;

    uint32_t optime = 0;
    struct timeval st;
    struct timeval et;
    
    memset(img, 0, sizeof(DISP_OUTPUT_IMG_SIZE));

    //printf("filename1:%s \n", filename1);

    gettimeofday(&st, NULL);
    if(mode == OPENCV_MODE_2)
    {
        OpenCV_face_detection(filename1, img, DISP_OUTPUT_IMG_W, DISP_OUTPUT_IMG_H);
    }
    else if(mode == OPENCV_MODE_3)
    {
        OpenCV_binding_image(filename1, filename2, img, DISP_OUTPUT_IMG_W, DISP_OUTPUT_IMG_H);
    } 
    else if(mode == OPENCV_MODE_4)
    {
        OpenCV_canny_edge_image(filename1, img, DISP_OUTPUT_IMG_W, DISP_OUTPUT_IMG_H);
    }
    else
    {
        OpenCV_load_file(filename1, img, DISP_OUTPUT_IMG_W, DISP_OUTPUT_IMG_H);
    }
    gettimeofday(&et, NULL);
    optime = ((et.tv_sec - st.tv_sec)*1000)+ ((int)et.tv_usec/1000 - (int)st.tv_usec/1000);
    printf("(mode:%d) operation time : %04d(ms)\n", mode, optime);
    draw_operatingtime(data->disp, optime);
    
    if(data->dump_state == DUMP_READY)
    {       
        DumpMsg dumpmsg;
        memcpy(data->dump_img_data, img, DISP_OUTPUT_IMG_SIZE);
        dumpmsg.type = DUMP_MSGQ_MSG_TYPE;
        dumpmsg.state_msg = DUMP_WRITE_TO_FILE;
        data->dump_state = DUMP_WRITE_TO_FILE;
            
        if (-1 == msgsnd(data->msgq_id, &dumpmsg, sizeof(DumpMsg)-sizeof(long), 0)) {
           MSG("state:%d, msg send fail\n", dumpmsg.state_msg);
        }
    } 
    else 
    {
        get_framebuf(dispbuf, pbuf);
        fbuf = pbuf[0];

        if(!fbuf) {
            MSG("dispbuf ptr is NULL");
            return;
        }
        
        memcpy(fbuf, img, DISP_OUTPUT_IMG_SIZE);

        cv_disp_update(data);
    }
}

/**
  * @brief  handling an input command
  * @param  arg: pointer to parameter of thr_data
  * @retval none
  */
void * input_thread(void *arg)
{
    struct thr_data *data = (struct thr_data *)arg;
    char cmd_input[128];
    char cmd_ready = true;
    char cmd_opencv = false;
    
    MSG("\n\nInput command:");
    MSG("\t   1  : load image");
    MSG("\t   2  : face detection");
    MSG("\t   3  : binding image");
    MSG("\t   4  : edge detection");
    MSG("\t save : display image(%dx%d) save to jpeg", DISP_OUTPUT_IMG_W,DISP_OUTPUT_IMG_H);
    MSG("\n");

    while(1)
    {
        if(cmd_ready == true) {
            /*standby to input command */
            cmd_ready = StandbyInput(cmd_input);     //define in cmd.cpp
        } else {
            if(0 == strncmp(cmd_input,"save",4)) {
                DumpMsg dumpmsg;
                dumpmsg.type = DUMP_MSGQ_MSG_TYPE;
                dumpmsg.state_msg = DUMP_CMD;
                data->dump_state = DUMP_CMD;
                if (-1 == msgsnd(data->msgq_id, &dumpmsg, sizeof(DumpMsg)-sizeof(long), 0)) {
                    printf("dump cmd msg send fail\n");
                }

                while(data->dump_state != DUMP_DONE) {
                    usleep(100);
                }
                data->dump_state = DUMP_NONE;
                cmd_opencv = false;
                MSG("image dump done");
            } else if(0 == strncmp(cmd_input,"1",4)) {
                data->opencv_mode = OPENCV_MODE_1;
                cmd_opencv = true;
                MSG("load image!");
            } else if(0 == strncmp(cmd_input,"2",4)) {
                data->opencv_mode = OPENCV_MODE_2;
                cmd_opencv = true;
                MSG("face detection!");
            } else if(0 == strncmp(cmd_input,"3",4)) {
                data->opencv_mode = OPENCV_MODE_3;
                cmd_opencv = true;
                MSG("binding images!");
            } else if(0 == strncmp(cmd_input,"4",4)) {
                data->opencv_mode = OPENCV_MODE_4;
                cmd_opencv = true;
                MSG("canny edge detection image!");
            } else {
                cmd_opencv = false;
                printf("cmd_input:%s \n", cmd_input);
            }

            if(cmd_opencv) 
            {
                cv_exam(data, IMAGE_SAMPLE_1, IMAGE_SAMPLE_2, data->opencv_mode);
                cmd_opencv = false;
            }

            cmd_ready = true;
        }
    }

    return NULL;
}

/**
  * @brief  diplay image dump and save to file
  * @param  arg: pointer to parameter of thr_data
  * @retval none
  */
void * capture_dump_thread(void *arg)
{
    struct thr_data *data = (struct thr_data *)arg;
    DumpMsg dumpmsg;

    while(1) {
        if(msgrcv(data->msgq_id, &dumpmsg, sizeof(DumpMsg)-sizeof(long), DUMP_MSGQ_MSG_TYPE, 0) >= 0) {
            switch(dumpmsg.state_msg) {
                case DUMP_CMD :
                    data->dump_state = DUMP_READY;
                    cv_exam(data, IMAGE_SAMPLE_1, IMAGE_SAMPLE_2, data->opencv_mode);
                    break;

                case DUMP_WRITE_TO_FILE :
                    cv_savetojpeg(data->dump_img_data, DISP_OUTPUT_IMG_W, DISP_OUTPUT_IMG_H);
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

int main(int argc, char **argv)
{
    struct display *disp;
    struct thr_data tdata;
    struct buffer *dispbuf;

    int disp_argc = 3;
    char* disp_argv[] = {"dummy", "-s", "4:480x272", "\0"}; // 추후 변경 여부 확인 후 처리..
    int ret = 0;
    unsigned char* pbuf[4];
    unsigned char* fbuf;

    printf("-- 5_opencv_disp example Start --\n");

    disp = disp_open(disp_argc, disp_argv);
    if (!disp) {
        ERROR("disp open error!");
        return 1;
    }
    disp->multiplanar = false;

    set_z_order(disp, disp->overlay_p.id);
    set_global_alpha(disp, disp->overlay_p.id);
    set_pre_multiplied_alpha(disp, disp->overlay_p.id);
    alloc_overlay_plane(disp, OVERLAY_DISP_FORCC, 0, 0, OVERLAY_DISP_W, OVERLAY_DISP_H);

    tdata.disp = disp;
    tdata.bfull_screen = true;

    allocate_output_buffers(&tdata);
    dispbuf = tdata.output_bufs[0];

    MSG ("nOutput(LCD) = %d x %d (%.4s)",
        dispbuf->width, dispbuf->height, (char*)&dispbuf->fourcc);

    if (dispbuf->height < 0 || dispbuf->width < 0 || dispbuf->fourcc < 0) {
        ERROR("Invalid parameters\n");
    }
   
    if(-1 == (tdata.msgq_id = msgget((key_t)DUMP_MSGQ_KEY, IPC_CREAT | 0666))) {
        fprintf(stderr, "%s msg create fail!!!\n", __func__);
        return -1;
    }
    
    pexam_data = &tdata;

    get_framebuf(dispbuf, pbuf);
    fbuf = pbuf[0];
    memset(fbuf, 0, DISP_OUTPUT_IMG_SIZE);
    cv_disp_update(&tdata);

    cv_exam(&tdata, IMAGE_SAMPLE_1, IMAGE_SAMPLE_2, OPENCV_MODE_1);

    ret = pthread_create(&tdata.threads[0], NULL, input_thread, &tdata);
    if(ret) {
        MSG("Failed creating input thread");
    }
    pthread_detach(tdata.threads[0]);

    ret = pthread_create(&tdata.threads[1], NULL, capture_dump_thread, &tdata);
    if(ret) {
        MSG("Failed creating capture dump thread");
    }
    pthread_detach(tdata.threads[1]);
    
    /* register signal handler for <CTRL>+C in order to clean up */
    if(signal(SIGINT, signal_handler) == SIG_ERR) {
        MSG("could not register signal handler");
        closelog();
        exit(EXIT_FAILURE);
    }

    pause();

    return ret;
}
