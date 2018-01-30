#include "imu.h"
#include <stdio.h>
#include <stdlib.h>
#include <malloc.h>
#include <string.h>

// XOR check
// return 0--success 1--errer
static unsigned char checkXor(unsigned char *data, int len)
{
    unsigned int checksum = 0;
    int i;
    unsigned char ret = 0;
    for (i = 0; i < len; i++)
    {
        checksum = checksum ^ data[i];
    }
    ret = (unsigned char)(checksum & 0xFF);
    return ret;
}
static unsigned char Navi_checkXor(NAVI data)
{
    unsigned char *p = (unsigned char *)&data;
    unsigned char ret = checkXor(p, 57);
    return ret;
}
static unsigned char Imu_checkXor(IMU data)
{
    unsigned char *p = (unsigned char *)&data;
    unsigned char ret = checkXor((p + 1), 41);
    return ret;
}

int save2file(const char* file, void* data, int len)
{
    FILE* fp = NULL;
    fp = fopen(file, "wb");
    if(fp==NULL)
    {
        printf("open fail\n");
        return 1;
    }
    int ret = fwrite(data,1, len, fp);
    if(ret<0)
    {
        printf("write fail\n");
        fclose(fp);
        return 1;
    }
    fflush(fp);
    fclose(fp);
    return 0;
}

int main(void)
{
    NAVI navi;
    IMU imu;
    memset(&navi, 0, sizeof(NAVI));
    memset(&imu, 0, sizeof(IMU));

    navi.head[0] = 0xBD;
    navi.head[1] = 0xDB;
    navi.head[2] = 0x0B;
    navi.check = Navi_checkXor(navi);

    imu.head = 0xAA;
    imu.tail =0xAC;
    imu.data[2] =0x55;
    imu.check = Imu_checkXor(imu);

    save2file("navi.data", &navi, sizeof(NAVI));
    save2file("imu.data", &imu, sizeof(IMU));

    printf("over\n\n");
    return 0;
}

