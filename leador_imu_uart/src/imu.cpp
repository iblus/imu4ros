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

#define LOCK(x)                                                                \
  do {                                                                         \
    int lockRet;                                                               \
    if ((lockRet = pthread_mutex_lock(&(x))) != 0)                             \
      printf("%s, --%d, lockRet = %d\n", __FUNCTION__, __LINE__, lockRet);     \
  } while (0)

#define UNLOCK(x)                                                              \
  do {                                                                         \
    int lockRet;                                                               \
    if ((lockRet = pthread_mutex_unlock(&(x))) != 0)                           \
      printf("%s, --%d, lockRet = %d\n", __FUNCTION__, __LINE__, lockRet);     \
  } while (0)
//==============================================================================
typedef int (*TTY_FUN)(void *);

typedef struct _tty_opt {
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
ros::NodeHandle node;

static void *tty_receive_pthread(void *arg) {
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

    while (IsRun) {
        FD_ZERO(&_readFdsr);
        FD_SET(ttyFp, &_readFdsr);

        selectRet = select(ttyFp + 1, &_readFdsr, NULL, NULL, &_tv);
        if (selectRet > 0) {
            if (FD_ISSET(ttyFp, &_readFdsr)) {
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

                } else {
                    printf("tty:%s receive fail:%d %d\n", ttyOpt->ttyPath, UartReadLen,
                           errno);
                }
            }
        } else {
        }
        usleep(10); // pthread delay, reduce CPU occupancy rate
    }

    return (void *)0;
}
#define PROCESS_SIZE 100
static void *tty_process_pthread(void *arg) {
    TTYOPT *ttyOpt = (TTYOPT *)arg;

    while (IsRun) {
        ttyOpt->fun(ttyOpt);
        usleep(100); // pthread delay, reduce CPU occupancy rate
    }

    return (void *)0;
}

static int init_tty(TTYOPT *ttyOpt) {
    int ttyFp = 0;
    FILE *saveFp = NULL;

    pthread_t tty_receive_tid;
    pthread_t tty_process_tid;

    ttyFp = uart_init(ttyOpt->ttyPath, ttyOpt->baud, 8, 0, 1);

    if (ttyFp <= 0) {
        printf("open tty:%s fail\n", ttyOpt->ttyPath);
        return -1;
    }
    ttyOpt->ttyFp = ttyFp;
    if (ttyOpt->savePath != NULL) {
        saveFp = fopen(ttyOpt->savePath, "wb");
        if (saveFp == NULL) {
            printf("create save file:%s fail!\n", ttyOpt->savePath);
            return -1;
        }
    }
    ttyOpt->saveFp = saveFp;

    initialize_loop_queue(&(ttyOpt->ttyQueue));
    if (pthread_mutex_init(&(ttyOpt->queueLock), NULL) != 0) {
        printf("%s--%d, pthread mutex init failed!\n", __FILE__, __LINE__);
        return -1;
    }

    if (pthread_create(&(ttyOpt->id[0]), NULL, tty_receive_pthread,
                       (void *)ttyOpt)) {
        printf("%s--%d, tty_receive_pthread create error!\n", __FILE__, __LINE__);
        return -1;
    }
    if (pthread_create(&(ttyOpt->id[1]), NULL, tty_process_pthread,
                       (void *)ttyOpt)) {
        printf("%s--%d, tty_receive_pthread create error!\n", __FILE__, __LINE__);
        return -1;
    }

    return 0;
}

// deal with navigation's data
static int dealNavi(void *arg) {
    TTYOPT *ttyOpt = (TTYOPT *)arg;

    int FramLen = ttyOpt->perLen;
    FILE *saveFp = ttyOpt->saveFp;
    LOOP_QUEUE *CommandQueue = &(ttyOpt->ttyQueue);
    char buf[PROCESS_SIZE];
    unsigned char head[2];
    leador_msgs::NaviMsg msg_navi;
    NAVI bufNavi;

    LOCK(ttyOpt->queueLock);
    if (loop_queue_avaliable_items_count(CommandQueue) >= FramLen) {

        loop_queue_out_preview(CommandQueue, (char*)head, sizeof(head));
        if(head[0]==0x55)
        {
            loop_queue_out(CommandQueue, buf, FramLen);
            UNLOCK(ttyOpt->queueLock);
            // send msg
            memcpy(&bufNavi, buf, sizeof(NAVI));
            memcpy(&(msg_navi.data), &bufNavi, sizeof(NAVI));
            ttyOpt->pub.publish(msg_navi);
            printf("Send %s Data...\n", ttyOpt->name);

            //===
            if (saveFp != NULL) {
                fwrite(buf, sizeof(buf), 1, saveFp);
                fflush(saveFp);
                // printf("save tty data to file:%s\n", ttyOpt->savePath);
            }

        }
        ttyOpt->isOut = 1;
        ttyOpt->timeOut = 0;
    } else {
        ttyOpt->timeOut++;
        if (ttyOpt->timeOut >= 200) {
            ttyOpt->isOut = 0;
        }
    }
    UNLOCK(ttyOpt->queueLock);
}

// deal with IMU's data
static int dealIMU(void *arg) {
    TTYOPT *ttyOpt = (TTYOPT *)arg;

    int FramLen = ttyOpt->perLen;
    FILE *saveFp = ttyOpt->saveFp;
    LOOP_QUEUE *CommandQueue = &(ttyOpt->ttyQueue);
    char buf[PROCESS_SIZE];
    unsigned char head[2];
    leador_msgs::ImuMsg msg_imu;
    IMU bufImu;

    LOCK(ttyOpt->queueLock);
    if (loop_queue_avaliable_items_count(CommandQueue) >= FramLen) {

        loop_queue_out_preview(CommandQueue, (char*)head, sizeof(head));
        if(head[0]==0x55)
        {
            loop_queue_out(CommandQueue, buf, FramLen);
            UNLOCK(ttyOpt->queueLock);
            // send msg
            memcpy(&bufImu, buf, sizeof(IMU));
            memcpy(&(msg_imu.data), &bufImu, sizeof(IMU));
            ttyOpt->pub.publish(msg_imu);
            printf("Send %s Data...\n", ttyOpt->name);

            //===
            if (saveFp != NULL) {
                fwrite(buf, sizeof(buf), 1, saveFp);
                fflush(saveFp);
                // printf("save tty data to file:%s\n", ttyOpt->savePath);
            }

        }
        ttyOpt->isOut = 1;
        ttyOpt->timeOut = 0;
    } else {
        ttyOpt->timeOut++;
        if (ttyOpt->timeOut >= 200) {
            ttyOpt->isOut = 0;
        }
    }
    UNLOCK(ttyOpt->queueLock);
}
// init navigation publish
static int initNavi(const char *com)
{
    IsRun = 1;

    if (com != NULL) {
        daoHang.ttyPath = (char *)malloc(50);
        memcpy(daoHang.ttyPath, com, 50);
    } else {
    }
    printf("the com:%s\n", daoHang.ttyPath);

    daoHang.baud = 9600;
    daoHang.savePath = "./nav.bin";
    daoHang.name = "navigation";
    daoHang.timeOut = 0;
    daoHang.fun = dealNavi;
    daoHang.perLen = sizeof(IMU);
    //==================

    daoHang.pub = node.advertise<leador_msgs::NaviMsg>("navi/data", 2);
    //==================

    if (init_tty(&daoHang) != 0) {
        printf("init tty:%s fail \n", daoHang.name);
        return 1;
    }
    return 0;
}
// init IMU publish
static int initIMU(const char *com) {
    IsRun = 1;

    if (com != NULL) {
        imuOpt.ttyPath = (char *)malloc(50);
        memcpy(imuOpt.ttyPath, com, 50);
    } else {
    }
    printf("the com:%s\n", imuOpt.ttyPath);

    imuOpt.baud = 9600;
    imuOpt.savePath = "./imu.bin";
    imuOpt.name = "imu";
    imuOpt.timeOut = 0;
    imuOpt.fun = dealIMU;
    imuOpt.perLen = sizeof(IMU);
    //==================
    imuOpt.pub = node.advertise<leador_msgs::ImuMsg>("imu/data", 2);
    //==================

    if (init_tty(&imuOpt) != 0) {
        printf("init tty:%s fail \n", imuOpt.name);
        return 1;
    }
    return 0;
}

static int stopPublish(TTYOPT *_ttyOpt) {
    IsRun = 0;

    if (pthread_join(_ttyOpt->id[0], NULL)) {
        perror("pthread_join err");
    }
    if (pthread_join(_ttyOpt->id[1], NULL)) {
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
int initSystem(const char *naviCom, const char *imuCom)
{
    if(initNavi(naviCom))
    {
        printf("init Navi fail\n");
        return 1;
    }
    if(initIMU(imuCom))
    {
        printf("init IMU fail\n");
        return 1;
    }
    return 0;
}