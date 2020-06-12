#ifndef _NEODRIVER_H
#define _NEODRIVER_H

#include <ros/ros.h>

#include <sensor_msgs/JointState.h>

#include <control_msgs/FollowJointTrajectoryAction.h>

#include <industrial_msgs/RobotStatus.h>

#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

#include "neo_msgs/MoveJoints.h"
#include "neo_msgs/MoveEndpos.h"

#include "interfaces.h"

#include <string>
using std::string;

#define PI 3.141592653
#define DegreeToRadian(degree) ((degree/180.0)*PI)
#define RadianToDegree(radian) ((radian/PI)*180.0)

class NeoDriver
{
private:
    Interfaces interfaces;

    string joint_names[MAX_JOINTS] = { "first_joint", "second_joint", "third_joint", "fourth_joint", "fifth_joint",
                                       "sixth_joint", "seventh_joint" };
    size_t joint_size;
    int joint_ids[MAX_JOINTS];

    // RobotState robot_state;
    double joints_record[MAX_JOINTS];
    double velocity_record[MAX_JOINTS];

private:
    ros::NodeHandle _handle;

    ros::Timer publish_timer;

    ros::Publisher JointStatePublisher;
    ros::Publisher RobotStatusPublisher;
    
    ros::ServiceServer MoveJointsServer;
    ros::ServiceServer MoveEndposServer;

public:
    NeoDriver() {};
    ~NeoDriver() {};

    bool Connect(const char* ip, short int port, const char* serial, unsigned int timeout, int transport, const char* name, unsigned int mode);
    bool Disconnect();
    void SetupROS();
    void UpdateData(const ros::TimerEvent& e);

private:
    bool executeMoveJoints(neo_msgs::MoveJoints::Request &req, neo_msgs::MoveJoints::Response &res);
    bool executeMoveEndpos(neo_msgs::MoveEndpos::Request &req, neo_msgs::MoveEndpos::Response &res);
};

#endif










