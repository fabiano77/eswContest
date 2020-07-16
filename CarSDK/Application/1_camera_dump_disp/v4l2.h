#ifndef V4L2_H_
#define V4L2_H_

#include "util.h"
#include <linux/videodev2.h>

struct v4l2
{
    int fd;
    int nbufs;
    struct v4l2_buffer *v4l2bufs; //display에 capture image출력하는 buffer
    struct buffer **bufs;         //
};

/* Print v4l2 related help */
void v4l2_usage(void);

/* Open v4l2  */
struct v4l2 *v4l2_open(uint32_t fourcc, uint32_t width, uint32_t height);
//v412의 포맷과 이미지 크기 지정
void v4l2_close(struct v4l2 *v4l2);
//

/* Share the buffers w/ v4l2 via dmabuf */
int v4l2_reqbufs(struct v4l2 *v4l2, struct buffer **bufs, uint32_t n);
// 버퍼 형태를 요청 및 버퍼 갯수 요청하고, 구성함

int v4l2_streamon(struct v4l2 *v4l2);
int v4l2_streamoff(struct v4l2 *v4l2);

/* Queue a buffer to the camera */
int v4l2_qbuf(struct v4l2 *v4l2, struct buffer *buf);
// 버퍼에 buf내용을 저장 1버퍼를 enqueue

/* Dequeue buffer from camera */
struct buffer *v4l2_dqbuf(struct v4l2 *v4l2);
//FIFO를 따라 버퍼 내용을 리턴 1버퍼씩 dequeue
#endif
