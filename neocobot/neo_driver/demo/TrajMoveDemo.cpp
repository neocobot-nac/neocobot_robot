#include <ros/ros.h>
#include <string>

#include "neo_msgs/EndPose.h"
#include "neo_msgs/MoveJ.h"

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "TrajMoveDemo");
    ros::NodeHandle handle;
    ros::ServiceClient client = handle.serviceClient<neo_msgs::MoveJ>("neo_moveJ");

    neo_msgs::EndPose pose1;
    pose1.x = 356.65;
    pose1.y = -9.77;
    pose1.z = 641.07;
    pose1.Rx = 172.34;
    pose1.Ry = -7.21;
    pose1.Rz = 5.60;

    neo_msgs::EndPose pose2;
    pose2.x = 338.24;
    pose2.y = -69.16;
    pose2.z = 655.30;
    pose2.Rx = -131.09;
    pose2.Ry = 6.46;
    pose2.Rz = -14.15;

    neo_msgs::EndPose pose3;
    pose3.x = 182.02;
    pose3.y = 9.03;
    pose3.z = 719.68;
    pose3.Rx = 124.85;
    pose3.Ry = 0.26;
    pose3.Rz = 14.91;

    neo_msgs::EndPose pose4;
    pose4.x = 502.92;
    pose4.y = -19.87;
    pose4.z = 221.47;
    pose4.Rx = 178.65;
    pose4.Ry = -70.72;
    pose4.Rz = 0.89;

    neo_msgs::MoveJ moveJ_msg;

    moveJ_msg.request.points.push_back(pose1);
    moveJ_msg.request.points.push_back(pose2);
    moveJ_msg.request.points.push_back(pose3);
    moveJ_msg.request.points.push_back(pose4);

    moveJ_msg.request.velocity = 10.0;
    moveJ_msg.request.acceleration = 50.0;
    moveJ_msg.request.interval = 0.04;
    moveJ_msg.request.close = 1;

    if(client.call(moveJ_msg))
    {
        ROS_INFO("call succeed");
    }
    else
    {
        ROS_INFO("call failed");
    }
    return 0;
}