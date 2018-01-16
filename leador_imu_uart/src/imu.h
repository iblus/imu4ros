#ifndef __IMU_H__
#define __IMU_H__
#include <stdint.h>
extern int stopIMU(void);
extern int initIMU(const char*);

#pragma pack(push, 1)
typedef struct{
  uint8_t data[72];
} IMU;

#pragma pack(pop)
#endif