#ifndef _H264_H_
#define _H264_H_
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
typedef void (*NALU_CallBack)(char *buf, int len, void *param);

int NALUInit(char *filename, NALU_CallBack call_func, void *param);

#endif