#ifndef __IMU_H__
#define __IMU_H__
#include <stdint.h>
#include <ros/ros.h>
extern void stopSystem(void);
extern int initSystem(const char *,const char *, ros::NodeHandle&);

#pragma pack(push, 1)
typedef struct {
    uint8_t head; //head of IMU , 0xAA
    uint8_t data[41];
    uint8_t check;//check bit of IMU,Byte 2-42 XOR
    uint8_t tail; //tail of IMU, 0xAC
} IMU;
typedef struct {
    uint8_t head[3];  //head of NAVI, 0xBD,0xDB,0x0B
    uint8_t data[54];
    uint8_t check;  //check bit of IMU,Byte 1-57 XOR
} NAVI;
#pragma pack(pop)
#endif