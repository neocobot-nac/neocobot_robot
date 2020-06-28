#ifndef _NEODRIVER_H
#define _NEODRIVER_H

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <industrial_msgs/RobotStatus.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

#include "rosUtils.h"
#include "interfaces.h"
#include "neo_msgs/GetJointIds.h"

#include "neo_msgs/MoveJoints.h"
#include "neo_msgs/MoveEndpos.h"
#include "neo_msgs/MoveJ.h"

#include "neo_msgs/Stop.h"

#include <string>
using std::string;

class NeoDriver
{
private:
    Interfaces interfaces;

    string joint_names[MAX_JOINTS] = { "first_joint", "second_joint", "third_joint", "fourth_joint", "fifth_joint", "sixth_joint", "seventh_joint" };

    size_t joint_size;
    int joint_ids[MAX_JOINTS];

    industrial_msgs::RobotStatus robot_status;

    double joints_record[MAX_JOINTS];
    double velocity_record[MAX_JOINTS];

public:
    ros::NodeHandle handle;

    ros::Timer update_timer;

    ros::Publisher joint_state_publisher;
    ros::Publisher robot_status_publisher;
    
    ros::ServiceServer get_joint_ids_server;

    ros::ServiceServer move_joints_server;
    ros::ServiceServer move_endpos_server;
    ros::ServiceServer movej_server;

    ros::ServiceServer stop_server;

public:
    NeoDriver() {};
    ~NeoDriver() {};

    bool Connect(const char* ip, short int port, const char* serial, unsigned int timeout, int transport, const char* name, unsigned int mode);
    bool Disconnect();
    void SetupROS();
    void UpdateData(const ros::TimerEvent& e) ;

private:
    bool executeGetJointIds(neo_msgs::GetJointIds::Request &req, neo_msgs::GetJointIds::Response &res);

    bool executeMoveJoints(neo_msgs::MoveJoints::Request &req, neo_msgs::MoveJoints::Response &res);
    bool executeMoveEndpos(neo_msgs::MoveEndpos::Request &req, neo_msgs::MoveEndpos::Response &res);
    bool executeMoveJ(neo_msgs::MoveJ::Request &req, neo_msgs::MoveJ::Response &res);

    bool executeStop(neo_msgs::Stop::Request &req, neo_msgs::Stop::Response &res);
};

#endif










