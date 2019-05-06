#ifndef _NEODRIVER_H
#define _NEODRIVER_H

#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/JointState.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <industrial_msgs/RobotStatus.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

#include <string>

#include "NeoServiceInterface.h"

#define PI 3.141592653
#define DegreeToRadian(degree) ((degree/180.0)*PI)
#define RadianToDegree(radian) ((radian/PI)*180.0)

using std::string;

class NeoDriver
{
private:
    NeoServiceInterface NeoSI;

    string joint_names[MAX_JOINTS] = { "first_joint", "second_joint", "third_joint", "fourth_joint", "fifth_joint",
                                       "sixth_joint", "seventh_joint", "eighth_joint", "ninth_joint", "tenth_joint"};
    size_t ids_size;
    int joint_ids[MAX_JOINTS];

    RobotState robot_state;
    double joints_record[MAX_JOINTS];
    double velocity_record[MAX_JOINTS];

private:
    ros::NodeHandle _handle;
    ros::Publisher JointStatePublisher;
    ros::Publisher RobotStatusPublisher;

public:
    NeoDriver() {};
    ~NeoDriver() {};

    bool Connect(const char* ip, short int port, const char* name, bool sim);
    bool Disconnect();
    void SetupROSFunction();
    void UpdateData();
};

#endif
