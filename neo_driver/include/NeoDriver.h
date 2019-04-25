#ifndef _NEO_DRIVER_H
#define _NEO_DRIVER_H

#include <ros/ros.h>
#include <industrial_msgs/RobotStatus.h>
#include <sensor_msgs/JointState.h>
#include <control_msgs/FollowJointTrajectory.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

#include "NeoServiceInterface.h"

class NeoDriver
{
private:
    NeoServiceInterface NeoSI;

    size_t ids_size;
    int joint_ids;

private:
    ros::NodeHandle handle;

public:
    NeoDriver() {};
    ~NeoDriver() {};

    bool Connect(const char* ip, short int port, const char* name, bool sim);
    bool Disconnect();
};

#endif
