#include "h264.h"
#include <pthread.h>

#define BUFFER 2 * 1024 * 1024
typedef struct Arg {
    NALU_CallBack P_pCallBack;
    void *p_param;
    char *path;
} Arg;
static inline int startCode3(char *buf)
{
    if (buf[0] == 0 && buf[1] == 0 && buf[2] == 1)
        return 1;
    else
        return 0;
}
static inline int startCode4(char *buf)
{
    if (buf[0] == 0 && buf[1] == 0 && buf[2] == 0 && buf[3] == 1)
        return 1;
    else
        return 0;
}
static char *findNextStartCode(char *buf, int len)
{
    int i;

    if (len < 3)
        return NULL;

    for (i = 0; i < len - 3; ++i) {
        if (startCode3(buf) || startCode4(buf))
            return buf;

        ++buf;
    }

    if (startCode3(buf))
        return buf;

    return NULL;
}
static int getFrameFromH264File(int fd, char *frame, int size)
{
    int rSize, frameSize;
    char *nextStartCode;

    if (fd < 0)
        return fd;
    rSize = read(fd, frame, size);
    if (!startCode3(frame) && !startCode4(frame))
        return -1;
    nextStartCode = findNextStartCode(frame + 3, rSize - 3);
    if (!nextStartCode) {
        lseek(fd, 0, SEEK_SET);
        frameSize = rSize;
    } else {
        frameSize = (nextStartCode - frame);
        lseek(fd, frameSize - rSize, SEEK_CUR);
    }

    return frameSize;
}
static void *thread_func(void *arg)
{
    Arg *p_arg = (Arg *)arg;
    printf("p_arg->path:%s\n", p_arg->path);
    int fd = open(p_arg->path, O_RDONLY);
    if (fd < 0) {
        printf("failed to open %s\n", p_arg->path);
        return NULL;
    }
    char *frame = (char *)malloc(BUFFER);
    uint32_t frameSize;
    int fps = 25;
    int startCode;
    while (1) {
        frameSize = getFrameFromH264File(fd, frame, BUFFER);
        if (frameSize < 0) {
            printf("read err\n");
            continue;
        }

        if (startCode3(frame))
            startCode = 3;
        else
            startCode = 4;

        // frameSize -= startCode;
        p_arg->P_pCallBack(frame, frameSize, p_arg->p_param);
        usleep(1000 * 1000 / fps);
    }
    free(frame);
    close(fd);
    free(p_arg->path);
    free(p_arg);
    return NULL;
}

int NALUInit(char *filename, NALU_CallBack call_func, void *param)
{
    pthread_t tid;
    Arg *arg = (Arg *)malloc(sizeof(Arg));
    arg->P_pCallBack = call_func;
    arg->p_param = param;
    arg->path = strdup(filename);
    pthread_create(&tid, NULL, thread_func, arg);
    return 0;
}
