#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termio.h>
#include <unistd.h>

#include <dirent.h>
#include <signal.h>
#include <time.h>

#include "imu.h"
#include "loop_queue.h"
#include "uart.h"
#include <leador_msgs/ImuMsg.h>
#include <leador_msgs/NaviMsg.h>
#include <malloc.h>
#include <ros/ros.h>
#include <sys/ioctl.h>

#define LOCK(x)                                                                  \
    do                                                                           \
    {                                                                            \
        int lockRet;                                                             \
        if ((lockRet = pthread_mutex_lock(&(x))) != 0)                           \
            printf("%s, --%d, lockRet = %d\n", __FUNCTION__, __LINE__, lockRet); \
    } while (0)

#define UNLOCK(x)                                                                \
    do                                                                           \
    {                                                                            \
        int lockRet;                                                             \
        if ((lockRet = pthread_mutex_unlock(&(x))) != 0)                         \
            printf("%s, --%d, lockRet = %d\n", __FUNCTION__, __LINE__, lockRet); \
    } while (0)
//==============================================================================
typedef int (*TTY_FUN)(void *);

typedef struct _tty_opt
{
    char *ttyPath;
    int ttyFp;
    int baud;

    char *savePath;
    FILE *saveFp;

    char *name;
    LOOP_QUEUE ttyQueue;
    pthread_mutex_t queueLock;
    pthread_t id[2];

    TTY_FUN fun;

    int timeOut;
    int isOut;
    int perLen;

    ros::Publisher pub;
} TTYOPT;

static int IsRun = 1;
static TTYOPT daoHang;
static TTYOPT imuOpt;

static void *tty_receive_pthread(void *arg)
{
    int i = 0;
    int selectRet, UartReadLen = 0;
    int ttyFp = 0;
    fd_set _readFdsr;
    struct timeval _tv;
    _tv.tv_sec = 0;
    _tv.tv_usec = 1000;

    TTYOPT *ttyOpt = (TTYOPT *)arg;
    ttyFp = ttyOpt->ttyFp;
    LOOP_QUEUE *CommandQueue = &(ttyOpt->ttyQueue);
    char UartReadContent[12500];
    memset(UartReadContent, 0, sizeof(UartReadContent));
    printf("goto tty_receive_pthread :%s \n", ttyOpt->name);

    while (IsRun)
    {
        FD_ZERO(&_readFdsr);
        FD_SET(ttyFp, &_readFdsr);

        selectRet = select(ttyFp + 1, &_readFdsr, NULL, NULL, &_tv);
        if (selectRet > 0)
        {
            if (FD_ISSET(ttyFp, &_readFdsr))
            {
                UartReadLen = read(ttyFp, UartReadContent, sizeof(UartReadContent));
// printf("\n tty:%s recive len:: %d\n",ttyOpt->ttyPath, UartReadLen);
#if 0
                for(i = 0; i < UartReadLen; i++)
                {
                    printf("0x%x ", UartReadContent[i]);
                    if(i % 9 == 8)
                        printf("\n");
                }
                printf("\n");
#endif

                if ((UartReadLen > 0) &&
                        !loop_queue_is_full(
                            CommandQueue)) // this fd has data ,so can recv it
                {
                    LOCK(ttyOpt->queueLock);
                    loop_queue_in(CommandQueue, UartReadContent, UartReadLen);
                    UNLOCK(ttyOpt->queueLock);
                }
                else
                {
                    printf("tty:%s receive fail:%d %d\n", ttyOpt->ttyPath, UartReadLen,
                           errno);
                }
            }
        }
        else
        {
        }
        usleep(10); // pthread delay, reduce CPU occupancy rate
    }

    return (void *)0;
}
#define PROCESS_SIZE 100
static void *tty_process_pthread(void *arg)
{
    TTYOPT *ttyOpt = (TTYOPT *)arg;

    while (IsRun)
    {
        ttyOpt->fun(ttyOpt);
        usleep(100); // pthread delay, reduce CPU occupancy rate
    }

    return (void *)0;
}

static int init_tty(TTYOPT *ttyOpt)
{
    int ttyFp = 0;
    FILE *saveFp = NULL;

    pthread_t tty_receive_tid;
    pthread_t tty_process_tid;

    ttyFp = uart_init(ttyOpt->ttyPath, ttyOpt->baud, 8, 0, 1);

    if (ttyFp <= 0)
    {
        printf("open tty:%s fail\n", ttyOpt->ttyPath);
        return -1;
    }
    ttyOpt->ttyFp = ttyFp;
    if (ttyOpt->savePath != NULL)
    {
        saveFp = fopen(ttyOpt->savePath, "wb");
        if (saveFp == NULL)
        {
            printf("create save file:%s fail!\n", ttyOpt->savePath);
            return -1;
        }
    }
    ttyOpt->saveFp = saveFp;

    initialize_loop_queue(&(ttyOpt->ttyQueue));
    if (pthread_mutex_init(&(ttyOpt->queueLock), NULL) != 0)
    {
        printf("%s--%d, pthread mutex init failed!\n", __FILE__, __LINE__);
        return -1;
    }

    if (pthread_create(&(ttyOpt->id[0]), NULL, tty_receive_pthread,
                       (void *)ttyOpt))
    {
        printf("%s--%d, tty_receive_pthread create error!\n", __FILE__, __LINE__);
        return -1;
    }
    if (pthread_create(&(ttyOpt->id[1]), NULL, tty_process_pthread,
                       (void *)ttyOpt))
    {
        printf("%s--%d, tty_receive_pthread create error!\n", __FILE__, __LINE__);
        return -1;
    }

    return 0;
}

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
// deal with navigation's data
static int dealNavi(void *arg)
{
    TTYOPT *ttyOpt = (TTYOPT *)arg;

    int FramLen = ttyOpt->perLen;
    FILE *saveFp = ttyOpt->saveFp;
    LOOP_QUEUE *CommandQueue = &(ttyOpt->ttyQueue);
    char buf[PROCESS_SIZE];
    unsigned char head[3];
    leador_msgs::NaviMsg msg_navi;
    NAVI bufNavi;

    LOCK(ttyOpt->queueLock);
    if (loop_queue_avaliable_items_count(CommandQueue) >= FramLen)
    {

        loop_queue_out_preview(CommandQueue, (char *)head, sizeof(head));

#if 0 // just for test
        printf("Navi %x %x %x\n", head[0], head[1], head[2]);
        memcpy(&(msg_navi.data), head, FramLen);
        ttyOpt->pub.publish(msg_navi);
#endif

        if ((head[0] == 0xBD) && (head[1] == 0xDB) && (head[2] == 0x0B))
        {
            loop_queue_out(CommandQueue, (char *)(&bufNavi), FramLen);

            // send msg
            if (bufNavi.check != Navi_checkXor(bufNavi))
            {
                printf("Navi check error!\n");
                initialize_loop_queue(CommandQueue);
                UNLOCK(ttyOpt->queueLock);
                return -1;
            }
            UNLOCK(ttyOpt->queueLock);

            memcpy(&(msg_navi.data), &bufNavi, FramLen);
            ttyOpt->pub.publish(msg_navi);
            printf("Send %s Data...\n", ttyOpt->name);

            //===
            if (saveFp != NULL)
            {
                fwrite(&bufNavi, sizeof(bufNavi), 1, saveFp);
                fflush(saveFp);
                // printf("save tty data to file:%s\n", ttyOpt->savePath);
            }
        }
        else
        {
            loop_queue_out(CommandQueue, (char *)head, sizeof(char));
            UNLOCK(ttyOpt->queueLock);
        }
        ttyOpt->isOut = 1;
        ttyOpt->timeOut = 0;
    }
    else
    {
        ttyOpt->timeOut++;
        if (ttyOpt->timeOut >= 200)
        {
            ttyOpt->isOut = 0;
        }
    }
    UNLOCK(ttyOpt->queueLock);
    return 0;
}

// deal with IMU's data
static int dealIMU(void *arg)
{
    TTYOPT *ttyOpt = (TTYOPT *)arg;

    int FramLen = ttyOpt->perLen;
    FILE *saveFp = ttyOpt->saveFp;
    LOOP_QUEUE *CommandQueue = &(ttyOpt->ttyQueue);
    unsigned char head[sizeof(IMU)];
    leador_msgs::ImuMsg msg_imu;
    IMU bufImu;

    LOCK(ttyOpt->queueLock);
    if (loop_queue_avaliable_items_count(CommandQueue) >= FramLen)
    {

        loop_queue_out_preview(CommandQueue, (char *)head, sizeof(head));

#if 0 // just for test
        printf("Imu %x %x %x\n", head[0], head[1], head[2]);
        memcpy(&(msg_imu.data), head, FramLen);
        ttyOpt->pub.publish(msg_imu);
#endif

        if ((head[0] == 0xAA) && (head[sizeof(head) - 1] == 0xAC))
        {
            loop_queue_out(CommandQueue, (char *)(&bufImu), FramLen);

            // send msg
            if (bufImu.check != Imu_checkXor(bufImu))
            {
                printf("IMU check error!\n");
                initialize_loop_queue(CommandQueue);
                UNLOCK(ttyOpt->queueLock);
                return -1;
            }
            UNLOCK(ttyOpt->queueLock);

            memcpy(&(msg_imu.data), &bufImu, FramLen);
            ttyOpt->pub.publish(msg_imu);
            printf("Send %s Data...\n", ttyOpt->name);

            //===
            if (saveFp != NULL)
            {
                fwrite(&bufImu, sizeof(bufImu), 1, saveFp);
                fflush(saveFp);
                // printf("save tty data to file:%s\n", ttyOpt->savePath);
            }
        }
        else
        {
            loop_queue_out(CommandQueue, (char *)head, sizeof(char));
            UNLOCK(ttyOpt->queueLock);
        }
        ttyOpt->isOut = 1;
        ttyOpt->timeOut = 0;
    }
    else
    {
        ttyOpt->timeOut++;
        if (ttyOpt->timeOut >= 200)
        {
            ttyOpt->isOut = 0;
        }
    }
    UNLOCK(ttyOpt->queueLock);
    return 0;
}
// init navigation publish
static int initNavi(const char *com, ros::NodeHandle &node)
{
    IsRun = 1;

    if (com != NULL)
    {
        daoHang.ttyPath = (char *)malloc(50);
        memcpy(daoHang.ttyPath, com, 50);
    }
    else
    {
    }
    printf("the com:%s\n", daoHang.ttyPath);

    daoHang.baud = 115200;
    daoHang.savePath = "./nav.bin";
    daoHang.name = "navigation";
    daoHang.timeOut = 0;
    daoHang.fun = dealNavi;
    daoHang.perLen = sizeof(NAVI);
    //==================

    daoHang.pub = node.advertise<leador_msgs::NaviMsg>("navi_leador/data", 2);
    //==================

    if (init_tty(&daoHang) != 0)
    {
        printf("init tty:%s fail \n", daoHang.name);
        return 1;
    }
    return 0;
}
// init IMU publish
static int initIMU(const char *com, ros::NodeHandle &node)
{
    IsRun = 1;

    if (com != NULL)
    {
        imuOpt.ttyPath = (char *)malloc(50);
        memcpy(imuOpt.ttyPath, com, 50);
    }
    else
    {
    }
    printf("the com:%s\n", imuOpt.ttyPath);

    imuOpt.baud = 115200;
    imuOpt.savePath = "./imu.bin";
    imuOpt.name = "imu";
    imuOpt.timeOut = 0;
    imuOpt.fun = dealIMU;
    imuOpt.perLen = sizeof(IMU);
    //==================
    imuOpt.pub = node.advertise<leador_msgs::ImuMsg>("imu_leador/data", 2);
    //==================

    if (init_tty(&imuOpt) != 0)
    {
        printf("init tty:%s fail \n", imuOpt.name);
        return 1;
    }
    return 0;
}

static int stopPublish(TTYOPT *_ttyOpt)
{
    IsRun = 0;

    if (pthread_join(_ttyOpt->id[0], NULL))
    {
        perror("pthread_join err");
    }
    if (pthread_join(_ttyOpt->id[1], NULL))
    {
        perror("pthread_join err");
        // exit(EXIT_FAILURE);
    }
    printf("exit pthread %s\n", _ttyOpt->name);
    return 0;
}

void stopSystem(void)
{
    stopPublish(&imuOpt);
    stopPublish(&daoHang);
    return;
}

int initSystem(const char *naviCom, const char *imuCom, ros::NodeHandle &node)
{
    if (initNavi(naviCom, node))
    {
        printf("init Navi fail\n");
        return 1;
    }
    if (initIMU(imuCom, node))
    {
        printf("init IMU fail\n");
        return 1;
    }
    return 0;
}
