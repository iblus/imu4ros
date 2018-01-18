#ifndef __IMU_H__
#define __IMU_H__
#include <stdint.h>
extern void stopSystem(void);
extern int initSystem(const char *naviCom, const char *imuCom);

#pragma pack(push, 1)
typedef struct {
    uint8_t data[5];
} IMU;
typedef struct{
  uint8_t data[5];
} NAVI;
#pragma pack(pop)
#endif