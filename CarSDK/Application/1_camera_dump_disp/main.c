z #include<signal.h>
#include <pthread.h> //Thread를 사용하기 위한 header file
#include <sys/ipc.h>
#include <sys/msg.h>
#include <termios.h>
#include <sys/time.h>
#include <errno.h>
#include <syslog.h>

#include "util.h"

#include "display-kms.h"
#include "v4l2.h"
#include "input_cmd.h"

#define NBUF 6 // v4l camera buf count

#define CAPTURE_IMG_W 1280                                   // camera resolution ,Width
#define CAPTURE_IMG_H 720                                    // camera resolution ,Height
#define CAPTURE_IMG_SIZE (CAPTURE_IMG_W * CAPTURE_IMG_H * 2) // YUYU : 16bpp
#define CAPTURE_IMG_FORMAT "uyvy"                            // camera resolution

#define DUMP_MSGQ_KEY 1020
#define DUMP_MSGQ_MSG_TYPE 0x02

    //사실상 Thread간의 통신 담당
    //Dump state가 존재하고, Dump 명령이 존재(명령이랄게 없는게 state수정하면 특정 state만 각 thread가 인지)
    //평상시에 키 입력 대기 상태(캡처 대기)
    // Dump key가 수신되면
    // Dump state를 CMD로 Input Thread에서 변환된다. 이후 Input_Thread는 DUMP_CMD를 DumpThread로 송신
    // Dump_Thread는 DUMP_CMD 수신하면 DUMP_READ로 state가 변환
    // DUMP_STATE가 DUMP_READ 임을 확인한 Capture Thread는 Dump cmd(Write to file 명령)을 DumpThread로 송신
    // dump_img_data에 이미지를 저장, 이후 dump thread가 write_to_file명령을 수신하면,
    //data file로 출력하고, Dump done으로 state를 변환한다.
    //done은 dump thread에서 state를 none으로 변환한다음 정지
    typedef enum {
        DUMP_NONE,          //Dump 명령 대기
        DUMP_CMD,           // thread간 공유데이터 수정--> capture_dump_thread가 수신하면, 파일명을 생성
        DUMP_READY,         // thread간 공유데이터 값 확인, msgsnd로 capture_dump_thread 호출
        DUMP_WRITE_TO_FILE, // date file로 출력 in Dump Thread
        DUMP_DONE           //Dump 명령 처리 완료(after write to file명령) -->바로 none으로 회귀
    } DumpState;

typedef struct _DumpMsg
{
    long type;
    int state_msg;
} DumpMsg;

struct thr_data //Thread간 전부 공유하는 data 및 state정보가 들어 있음
{
    struct display *disp;
    struct v4l2 *v4l2;

    DumpState dump_state;                          //state를 변환하면 모두가 공유함
    unsigned char dump_img_data[CAPTURE_IMG_SIZE]; // dump image size

    int msgq_id;
    bool bfull_screen;  // true : 480x272 disp 화면에 맞게 scale 그렇지 않을 경우 false.
    bool bstream_start; // camera stream start 여부
    pthread_t threads[3];
};

static uint32_t getfourccformat(char *format)
{
    if (strcmp(format, "rgb24") == 0)
    {
        return FOURCC('R', 'G', 'B', '3');
    }
    else if (strcmp(format, "bgr24") == 0)
    {
        return FOURCC('B', 'G', 'R', '3');
    }
    else if (strcmp(format, "argb32") == 0)
    {
        return FOURCC('A', 'R', '2', '4');
    }
    else if (strcmp(format, "abgr32") == 0)
    {
        return FOURCC('R', 'A', '2', '4');
    }
    else if (strcmp(format, "yuyv") == 0)
    {
        return FOURCC('Y', 'U', 'Y', 'V');
    }
    else if (strcmp(format, "uyvy") == 0)
    {
        return FOURCC('U', 'Y', 'V', 'Y');
    }
    else if (strcmp(format, "nv12") == 0)
    {
        return FOURCC('N', 'V', '1', '2');
    }
    else
    {
        MSG("not yet support format..");
    }

    return 0;
}

/**
  * @brief  Capture and display of camera
  * @param  arg: pointer to parameter of thr_data
  * @retval none
  */
void *capture_thread(void *arg)
{
    struct thr_data *data = (struct thr_data *)arg;
    struct display *disp = data->disp;
    struct v4l2 *v4l2 = data->v4l2;
    struct buffer **buffers, *capt;
    int ret, i;

    // display buffer alloc
    buffers = disp_get_vid_buffers(disp, NBUF, getfourccformat(CAPTURE_IMG_FORMAT), CAPTURE_IMG_W, CAPTURE_IMG_H);
    if (!buffers)
    {
        return NULL;
    }

    ret = v4l2_reqbufs(v4l2, buffers, NBUF);
    if (ret)
    {
        return NULL;
    }

    for (i = 0; i < NBUF; i++)
    {
        v4l2_qbuf(v4l2, buffers[i]);
    }

    ret = v4l2_streamon(v4l2);
    if (ret)
    {
        return NULL;
    }

    MSG("streaming started...");
    data->bstream_start = true;

    while (true)
    {
        capt = v4l2_dqbuf(v4l2);
        if (data->bfull_screen)
            capt->noScale = false;
        else
            capt->noScale = true;

        ret = disp_post_vid_buffer(disp, capt,
                                   0, 0, CAPTURE_IMG_W, CAPTURE_IMG_H);
        if (ret)
        {
            ERROR("Post buffer failed");
            return NULL;
        }
        if (data->dump_state == DUMP_READY)
        {
            DumpMsg dumpmsg;
            unsigned char *pbuf[4];

            // get framebuffer ptr
            if (get_framebuf(capt, pbuf) == 0)
            {
                switch (capt->fourcc)
                {
                case FOURCC('U', 'Y', 'V', 'Y'):
                case FOURCC('Y', 'U', 'Y', 'V'):
                    memcpy(data->dump_img_data, pbuf[0], CAPTURE_IMG_SIZE);
                    break;
                default:
                    MSG("DUMP.. not yet support format : %.4s\n", (char *)&capt->fourcc);
                    break;
                }
            }
            else
            {
                MSG("dump capture buf fail !");
            }

            dumpmsg.type = DUMP_MSGQ_MSG_TYPE;
            dumpmsg.state_msg = DUMP_WRITE_TO_FILE;
            data->dump_state = DUMP_WRITE_TO_FILE;
            if (-1 == msgsnd(data->msgq_id, &dumpmsg, sizeof(DumpMsg) - sizeof(long), 0))
            {
                MSG("state:%d, msg send fail\n", dumpmsg.state_msg);
            }
        }

        v4l2_qbuf(v4l2, capt);
    }
    v4l2_streamoff(v4l2);

    MSG("Ok!");
    return NULL;
}

/**
  * @brief  Capture image dump and save to file
  * @param  arg: pointer to parameter of thr_data
  * @retval none
  */
void *capture_dump_thread(void *arg)
{
    struct thr_data *data = (struct thr_data *)arg;
    FILE *fp;
    char file[50];
    struct timeval timestamp;
    struct tm *today;
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
                sprintf(file, "dump_%04d%02d%02d_%02d%02d%02d.%s", today->tm_year + 1900, today->tm_mon + 1, today->tm_mday, today->tm_hour, today->tm_min, today->tm_sec, CAPTURE_IMG_FORMAT);
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
                    fwrite(data->dump_img_data, CAPTURE_IMG_SIZE, 1, fp);
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

/**
  * @brief  handling an input command
  * @param  arg: pointer to parameter of thr_data
  * @retval none
  */
void *input_thread(void *arg)
{
    struct thr_data *data = (struct thr_data *)arg;

    char cmd_input[128];
    char cmd_ready = true;

    while (!data->bstream_start)
    {
        usleep(100 * 1000);
    }

    MSG("\n\nInput command:");
    MSG("\t dump  : display image(%s, %dx%d) dump", CAPTURE_IMG_FORMAT, CAPTURE_IMG_W, CAPTURE_IMG_H);
    MSG("\n");

    while (1)
    {
        if (cmd_ready == true) //평상시의 입력 대기
        {
            /*standby to input command */
            cmd_ready = StandbyInput(cmd_input); //define in cmd.cpp
        }
        else //입력이 들어온 경우
        {

            if (0 == strncmp(cmd_input, "dump", 4)) //덤프키 수신후
            {
                DumpMsg dumpmsg;
                dumpmsg.type = DUMP_MSGQ_MSG_TYPE;
                dumpmsg.state_msg = DUMP_CMD;
                data->dump_state = DUMP_CMD;

                if (-1 == msgsnd(data->msgq_id, &dumpmsg, sizeof(DumpMsg) - sizeof(long), 0))
                {
                    printf("dump cmd msg send fail\n");
                }
                else
                {
                    printf("dump cmd msg send\n");
                }

                while (data->dump_state != DUMP_DONE) //dump 상황이 끝날 때 까지, 유휴 상태 보존
                {
                    usleep(5 * 1000);
                }
                data->dump_state = DUMP_NONE; //끝나고 입력 대기 상태인 none으로 회귀
                printf("dump done!\n");
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

static struct thr_data *pexam_data = NULL;

/**
  * @brief  handling an SIGINT(CTRL+C) signal
  * @param  sig: signal type
  * @retval none
  */
void signal_handler(int sig)
{
    MSG("signal_hanlde\n");

    if (sig == SIGINT)
    {
        pthread_cancel(pexam_data->threads[0]);
        pthread_cancel(pexam_data->threads[1]);
        pthread_cancel(pexam_data->threads[2]);

        msgctl(pexam_data->msgq_id, IPC_RMID, 0);

        v4l2_streamoff(pexam_data->v4l2);
        disp_free_buffers(pexam_data->disp, NBUF);

        disp_close(pexam_data->disp);
        v4l2_close(pexam_data->v4l2);

        MSG("-- 1_camera_dump_disp example End --\n");
    }
}

int main(int argc, char **argv)
{
    struct display *disp;
    struct v4l2 *v4l2;
    struct thr_data tdata;
    int disp_argc = 3;
    char *disp_argv[] = {"dummy", "-s", "4:480x272", "\0"};
    int ret = 0;
    uint32_t fourcc;
    printf("-- 1_camera_dump_disp example Start --\n");

    tdata.dump_state = DUMP_NONE;
    memset(tdata.dump_img_data, 0, sizeof(tdata.dump_img_data));

    disp = disp_open(disp_argc, disp_argv);
    if (!disp)
    {
        ERROR("disp open error!");
        return 1;
    }

    fourcc = getfourccformat(CAPTURE_IMG_FORMAT);
    v4l2 = v4l2_open(fourcc, CAPTURE_IMG_W, CAPTURE_IMG_H);
    if (!v4l2)
    {
        ERROR("v4l2 open error!");
        disp_close(disp);
        return 1;
    }

    MSG("Input(Camera) = %d x %d (%.4s)\nOutput(LCD) = %d x %d (%.4s)",
        CAPTURE_IMG_W, CAPTURE_IMG_H, (char *)&fourcc,
        CAPTURE_IMG_H, CAPTURE_IMG_H, (char *)&fourcc);

    tdata.disp = disp;
    tdata.v4l2 = v4l2;
    tdata.bfull_screen = true;
    tdata.bstream_start = false;

    // create a msgq
    if (-1 == (tdata.msgq_id = msgget((key_t)DUMP_MSGQ_KEY, IPC_CREAT | 0666)))
    {
        fprintf(stderr, "%s msg create fail!!!\n", __func__);
        return -1;
    }

    pexam_data = &tdata;

    ret = pthread_create(&tdata.threads[0], NULL, capture_thread, &tdata);
    if (ret)
    {
        MSG("Failed creating capture thread");
    }
    pthread_detach(tdata.threads[0]);

    ret = pthread_create(&tdata.threads[1], NULL, capture_dump_thread, &tdata);
    if (ret)
    {
        MSG("Failed creating capture dump thread");
    }
    pthread_detach(tdata.threads[1]);

    ret = pthread_create(&tdata.threads[2], NULL, input_thread, &tdata);
    if (ret)
    {
        MSG("Failed creating input thread");
    }
    pthread_detach(tdata.threads[2]);

    /* register signal handler for <CTRL>+C in order to clean up */
    if (signal(SIGINT, signal_handler) == SIG_ERR)
    {
        MSG("could not register signal handler");
        closelog();
        exit(EXIT_FAILURE);
    }

    pause();

    return ret;
}
