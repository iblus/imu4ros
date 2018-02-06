#include "ros/ros.h"
#include <leador_msgs/ImuMsg.h>
#include <leador_msgs/NaviMsg.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

#include "imu.h"

//get system local time
static void getTime(char*str, int len)
{
	time_t timep;
	struct tm *p_lt;
	time(&timep);
	p_lt = localtime(&timep);

	memset(str,0,len);

	sprintf(str, "%d-%02d-%02d-%02d-%02d-%02d",
			(1900+p_lt->tm_year), p_lt->tm_mon, p_lt->tm_mday,
			p_lt->tm_hour, p_lt->tm_min, p_lt->tm_sec);
	return;
}
static void msgToImu(const leador_msgs::ImuMsg &msg, IMU_D *imu)
{
    int i=0;
    uint8_t *data = (uint8_t*)imu;
    memset(data, 0, sizeof(IMU_D));
    for(i=0; i<sizeof(IMU_D); i++)
    {
        data[i] = msg.data[i];
    }
    return;
}

static void msgToNavi(const leador_msgs::NaviMsg &msg, NAVI_D *navi)
{
    int i=0;
    uint8_t *data = (uint8_t*)navi;
    memset(data, 0, sizeof(NAVI_D));
    for(i=0; i<sizeof(NAVI_D); i++)
    {
        data[i] = msg.data[i];
    }
    return;
}

static int Recive_imu_cun = 0;
static  FILE *SaveImuFp = NULL;
void imuCallback(const leador_msgs::ImuMsg &msg)
{
    IMU_D imu;
    msgToImu(msg, &imu);
    
    //save data to file
    if(SaveImuFp !=NULL)
    {
        fwrite(&imu, sizeof(imu), 1, SaveImuFp);
        fflush(SaveImuFp);
    }
    
    Recive_imu_cun++;
    if(Recive_imu_cun%125 !=0)
    return;
    printf("imu:%d\n", Recive_imu_cun);

    //==========================
    printf("imu.gyro_x =%.5f y = %.5f z = %0.5f\n",imu.gyro_x,imu.gyro_y,imu.gyro_z);
    printf("imu.accel_x=%.5f y=%.5f z=%.5f\n", imu.accel_x,imu.accel_y, imu.accel_y);

    //==========================
    uint8_t* p = (uint8_t*)&imu;
    for(int i=0; i<sizeof(IMU_D); i++)
    {
        printf("0x%x ",*p++);
    }
    printf("\n");
}

static int Recive_navi_cun =0;
static  FILE *SaveNaviFp = NULL;
void naviCallback(const leador_msgs::NaviMsg &msg)
{
    NAVI_D navi;
    msgToNavi(msg, &navi);

    // save data to file
    if(SaveNaviFp !=NULL)
    {
        fwrite(&navi, sizeof(navi), 1, SaveNaviFp);
        fflush(SaveNaviFp);
    }

    Recive_navi_cun ++;
    if(Recive_navi_cun%125 !=0)
    return;
    printf("navi:%d\n",Recive_navi_cun);
    //==========================
    printf("navi.hengGunJiao=%d\n",navi.hengGunJiao);
    printf("navi.fuYangJiao=%d\n",navi.fuYangJiao);
    printf("navi.fangWeiJiao=%d\n",navi.fangWeiJiao);

    printf("navi.gyro_x=%d\n",navi.gyro_x);
    printf("navi.gyro_y=%d\n",navi.gyro_y);
    printf("navi.gyro_z=%d\n",navi.gyro_z);
    printf("navi.accel_x=%d\n",navi.accel_x);
    printf("navi.accel_y=%d\n",navi.accel_y);
    printf("navi.accer_z=%d\n",navi.accer_z);

    printf("navi.jingDu=%d\n",navi.jingDu);
    printf("navi.weiDu=%d\n",navi.weiDu);
    printf("navi.gaoDu=%d\n",navi.gaoDu);
    printf("navi.bei_sudu=%d\n",navi.bei_sudu);
    printf("navi.dong_sudo=%d\n",navi.dong_sudo);
    printf("navi.di_sudo=%d\n",navi.di_sudo);

    printf("navi.status=0x%x\n",navi.status);

    printf("navi.time=%d\n",navi.time);
    printf("navi.type=0x%x\n",navi.type);
    //==========================
    uint8_t* p = (uint8_t*)&navi;
    for(int i=0; i<sizeof(NAVI_D); i++)
    {
        printf("0x%x ",*p++);
    }
    printf("\n");
}

int main(int argc, char **argv)
{
    /**
     * The ros::init() function needs to see argc and argv so that it can perform
     * any ROS arguments and name remapping that were provided at the command line.
     * For programmatic remappings you can use a different version of init() which takes
     * remappings directly, but for most command-line programs, passing argc and argv is
     * the easiest way to do it.  The third argument to init() is the name of the node.
     *
     * You must call one of the versions of ros::init() before using any other
     * part of the ROS system.
     */
    ros::init(argc, argv, "leador_listener");

    /**
     * NodeHandle is the main access point to communications with the ROS system.
     * The first NodeHandle constructed will fully initialize this node, and the last
     * NodeHandle destructed will close down the node.
     */
    ros::NodeHandle n;

//============================
    char strTime[64];
	getTime(strTime,sizeof(strTime));
    char pathBuf[64];
    memset(pathBuf, 0, sizeof(pathBuf));
    sprintf(pathBuf, "%s.imu",strTime);

    SaveImuFp = fopen(pathBuf, "wb");
    if (SaveImuFp == NULL)
    {
        printf("create Imu file:%s fail!\n", pathBuf);
    }
    char pathBuf2[64];
    memset(pathBuf2, 0, sizeof(pathBuf2));
    sprintf(pathBuf2, "%s.navi",strTime);
    SaveNaviFp = fopen(pathBuf2, "wb");
    if (SaveNaviFp == NULL)
    {
        printf("create Navi file:%s fail!\n", pathBuf2);
    }
	printf("\n save navi to %s save imu to %s\n", pathBuf2, pathBuf);
//=============================
    /**
     * The subscribe() call is how you tell ROS that you want to receive messages
     * on a given topic.  This invokes a call to the ROS
     * master node, which keeps a registry of who is publishing and who
     * is subscribing.  Messages are passed to a callback function, here
     * called chatterCallback.  subscribe() returns a Subscriber object that you
     * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
     * object go out of scope, this callback will automatically be unsubscribed from
     * this topic.
     *
     * The second parameter to the subscribe() function is the size of the message
     * queue.  If messages are arriving faster than they are being processed, this
     * is the number of messages that will be buffered up before beginning to throw
     * away the oldest ones.
     */
    ros::Subscriber sub_imu = n.subscribe("imu_leador/data", 5, imuCallback);
    ros::Subscriber sub_navi = n.subscribe("navi_leador/data", 5, naviCallback);

    /**
     * ros::spin() will enter a loop, pumping callbacks.  With this version, all
     * callbacks will be called from within this thread (the main one).  ros::spin()
     * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
     */
    ros::spin();

    return 0;
}
