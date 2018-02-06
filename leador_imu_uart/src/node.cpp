#include <stdlib.h>
#include <stdio.h>
// ROS
#include "imu.h"
#include <ros/ros.h>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "leador_imu");

    ros::NodeHandle priv_nh("~");

    if (argc < 2)
    {
        ROS_INFO("No com to use!!!");
        ros::WallDuration(1.0).sleep();
    }
    std::string naviInterface = argv[1];
    if (naviInterface.length())
    {
        ROS_INFO("Use %s for recive data from imu", naviInterface.c_str());
    }
    else
    {
        ROS_FATAL("Failed to open COM");
        ros::WallDuration(1.0).sleep();
    }
    std::string imuInterface = "";
    if(argc > 2)
    {
        imuInterface = argv[2];
        if (imuInterface.length())
        {
            ROS_INFO("Use %s for recive data from imu", imuInterface.c_str());
        }
        else
        {
            ROS_FATAL("Failed to open COM");
            ros::WallDuration(1.0).sleep();
        }
        printf("get Navi and IMU\n");
    }
    else
    {
        printf("only get Navi \n");
    }

    //========================
    ros::Rate loop_rate(0.1);
    ros::NodeHandle node;
    int count = 0;
    if (!initSystem(naviInterface.c_str(), imuInterface.c_str(), node))
    {
        // Loop until shutdown
        while (ros::ok())
        {

            ROS_INFO("run ..%d", count);
            // Handle callbacks
            ros::spinOnce();
            loop_rate.sleep();
            count++;
        }
    }
    else
    {
        ROS_FATAL("Failed to open COM");
        ros::WallDuration(1.0).sleep();
    }
    return 0;
}
