/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015-2017, Dataspeed Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Dataspeed Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

// ROS
#include "imu.h"
#include <ros/ros.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "leador_imu");

    ros::NodeHandle priv_nh("~");

    if (argc < 3) {
        ROS_INFO("No com to use!!!");
        ros::WallDuration(1.0).sleep();
    }
    std::string naviInterface = argv[1];
    if (naviInterface.length()) {
        ROS_INFO("Use %s for recive data from imu", naviInterface.c_str());
    } else {
        ROS_FATAL("Failed to open COM");
        ros::WallDuration(1.0).sleep();
    }
    std::string imuInterface = argv[2];
    if (imuInterface.length()) {
        ROS_INFO("Use %s for recive data from imu", imuInterface.c_str());
    } else {
        ROS_FATAL("Failed to open COM");
        ros::WallDuration(1.0).sleep();
    }

    //========================
    ros::Rate loop_rate(0.1);
    ros::NodeHandle node;
    int count = 0;
    if (!initSystem(naviInterface.c_str(), imuInterface.c_str(), node)) {
        // Loop until shutdown
        while (ros::ok()) {

            ROS_INFO("run ..%d", count);
            // Handle callbacks
            ros::spinOnce();
            loop_rate.sleep();
            count++;
        }

    } else {
        ROS_FATAL("Failed to open COM");
        ros::WallDuration(1.0).sleep();
    }
    return 0;
}
