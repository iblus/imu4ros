/*
 * Copyright (C) 2008, Morgan Quigley and Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

// %Tag(FULLTEXT)%
#include "ros/ros.h"
#include <leador_msgs/NaviMsg.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "imu.h"

static int Recive_cun = 0;
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

void naviCallback(const leador_msgs::NaviMsg &msg)
{
    NAVI_D navi;
    msgToNavi(msg, &navi);
    Recive_cun ++;
    printf("navi:%d\n",Recive_cun);
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
    ros::init(argc, argv, "leador_listener");
    ros::NodeHandle n;

    ros::Subscriber sub_navi = n.subscribe("navi_leador/data", 5, naviCallback);

    ros::spin();
    return 0;
}
