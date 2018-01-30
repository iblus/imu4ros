#ifndef __IMU_H__
#define __IMU_H__
#include <stdint.h>
#include <ros/ros.h>
extern void stopSystem(void);
extern int initSystem(const char *, const char *, ros::NodeHandle &);

#pragma pack(push, 1)
typedef struct
{
    uint8_t head; //head of IMU , 0xAA
    uint8_t data[41];
    uint8_t check; //check bit of IMU,Byte 2-42 XOR
    uint8_t tail;  //tail of IMU, 0xAC
} IMU;
typedef struct
{
    uint8_t head[3]; //head of NAVI, 0xBD,0xDB,0x0B
    uint8_t data[54];
    uint8_t check; //check bit of IMU,Byte 1-57 XOR
} NAVI;
typedef struct
{
    uint8_t head[3];

    int16_t hengGunJiao;
    int16_t fuYangJiao;
    int16_t fangWeiJiao;

    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
    int16_t accel_x;
    int16_t accel_y;
    int16_t accer_z;

    int32_t jingDu;
    int32_t weiDu;
    int32_t gaoDu;

    int16_t bei_sudu;
    int16_t dong_sudo;
    int16_t di_sudo;

    uint8_t status;

    uint8_t reserved[6];
    int16_t data1;
    int16_t data2;
    int16_t data3;

    uint32_t time;
    uint8_t type;

    uint8_t check;
} NAVI_D;
typedef struct
{
    uint8_t head;
    uint8_t gps_week[2];
    uint8_t gps_second[8];

    float gyro_x;
    float gyro_y;
    float gyro_z;
    float accel_x;
    float accel_y;
    float accel_z;

    int16_t temp;

    uint8_t reserved[4];
    uint8_t id;
    uint8_t check;
    uint8_t tail;
} IMU_D;
#pragma pack(pop)
#endif