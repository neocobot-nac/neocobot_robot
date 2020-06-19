#ifndef _NEODRIVER_H
#define _NEODRIVER_H

#include <ros/ros.h>

#include <actionlib/server/action_server.h>

#include <control_msgs/JointTolerance.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

#include <sensor_msgs/JointState.h>
#include <industrial_msgs/RobotStatus.h>

#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

#include "neo_msgs/MoveJoints.h"
#include "neo_msgs/MoveEndpos.h"

#include "interfaces.h"

#include <string>
using std::string;

#define DEFAULT_GOAL_THRESHOLD 0.01

#define PI 3.141592653
#define DegreeToRadian(degree) ((degree/180.0)*PI)
#define RadianToDegree(radian) ((radian/PI)*180.0)

class NeoDriver
{
private:
    Interfaces interfaces;

    string joint_names[MAX_JOINTS] = { "first_joint", "second_joint", "third_joint", "fourth_joint", "fifth_joint", "sixth_joint", "seventh_joint" };

    string s_ip;
    string s_port;
    string s_serial;
    string s_timeout;
    string s_transport;
    string s_name;
    string s_mode;

    size_t joint_size;
    int joint_ids[MAX_JOINTS];

    industrial_msgs::RobotStatus robot_status;

    double joints_record[MAX_JOINTS];
    double velocity_record[MAX_JOINTS];

    trajectory_msgs::JointTrajectory trajectory_cache;
    double goal_threshold = DEFAULT_GOAL_THRESHOLD;

public:
    typedef actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction> FollowJointTrajectoryActionServer;

    ros::NodeHandle _handle;

    ros::Timer update_timer;

    ros::Publisher joint_state_publisher;
    ros::Publisher robot_status_publisher;
    ros::Publisher joint_path_command_publisher;
    
    ros::ServiceServer move_joints_server;
    ros::ServiceServer move_endpos_server;

    FollowJointTrajectoryActionServer follow_joint_trajectory_action_server;

public:
    NeoDriver(std::string action_name);
    ~NeoDriver();

    bool Connect(string ip, string port, string serial, string timeout, string transport, string name, string mode);
    bool Disconnect();
    void SetupROS();
    void UpdateData(const ros::TimerEvent& e);

    void actionGoal(FollowJointTrajectoryActionServer::GoalHandle goal_handle);
    void actionCancel(FollowJointTrajectoryActionServer::GoalHandle goal_handle);
    void controllerStateFeedback(const control_msgs::FollowJointTrajectoryFeedbackConstPtr &msg);
    bool withinGoalConstraints(const control_msgs::FollowJointTrajectoryFeedbackConstPtr &msg, const trajectory_msgs::JointTrajectory &trajectory);

private:
    bool executeMoveJoints(neo_msgs::MoveJoints::Request &req, neo_msgs::MoveJoints::Response &res);
    bool executeMoveEndpos(neo_msgs::MoveEndpos::Request &req, neo_msgs::MoveEndpos::Response &res);
};

#endif










