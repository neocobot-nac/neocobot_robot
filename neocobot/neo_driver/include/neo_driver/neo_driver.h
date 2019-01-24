#ifndef NEO_DRIVER_H_
#define NEO_DRIVER_H_

#include <ros/ros.h>
#include <string>
#include <stdlib.h>
#include "robot.h"

#include "neo_msgs/JointAngles.h"
#include "neo_msgs/JointVelocity.h"
#include "neo_msgs/JointCurrent.h"
#include "neo_msgs/EndPose.h"

#include "neo_msgs/Calibrate.h"
#include "neo_msgs/Forward.h"
#include "neo_msgs/GetInput.h"
#include "neo_msgs/GetMotorIds.h"
#include "neo_msgs/Hold.h"
#include "neo_msgs/Inverse.h"
#include "neo_msgs/MoveToAngles.h"
#include "neo_msgs/MoveToPose.h"
#include "neo_msgs/Recover.h"
#include "neo_msgs/Release.h"
#include "neo_msgs/Reset.h"
#include "neo_msgs/SetEOATAction.h"
#include "neo_msgs/SetOutput.h"
#include "neo_msgs/Stop.h"
#include "neo_msgs/MoveJ.h"
#include "neo_msgs/MoveL.h"
#include "neo_msgs/MoveP.h"

class NeoDriver {
private:
    Robot robot;

    size_t  ids_size;
    int     motor_ids[MAX_MOTORS];

    bool _isConnected = false;
    bool _isSetup = false;

    ros::NodeHandle handle;
    ros::Timer publish_timer;
    ros::Publisher angles_publisher;
    ros::Publisher current_publisher;
    ros::Publisher velocity_publisher;
    ros::Publisher pose_publisher;

    ros::ServiceServer MoveToAngles_server;
    ros::ServiceServer MoveToPose_server;
    ros::ServiceServer MoveJ_server;
    ros::ServiceServer MoveL_server;
    ros::ServiceServer MoveP_server;
    ros::ServiceServer Release_server;
    ros::ServiceServer Hold_server;
    ros::ServiceServer Stop_server;
    ros::ServiceServer Recover_server;
    ros::ServiceServer Calibrate_server;
    ros::ServiceServer Reset_server;
    ros::ServiceServer GetMotorIds_server;
    ros::ServiceServer Forward_server;
    ros::ServiceServer Inverse_server;
    ros::ServiceServer GetInput_server;
    ros::ServiceServer SetOutput_server;
    ros::ServiceServer SetEOATAction_server;

public:
	NeoDriver(std::string host, int port, std::string name, bool debug);
    ~NeoDriver();

    void timerCallback(const ros::TimerEvent& e);

    bool executeMoveToAngles(neo_msgs::MoveToAngles::Request &req, neo_msgs::MoveToAngles::Response &res);
    bool executeMoveToPose(neo_msgs::MoveToPose::Request &req, neo_msgs::MoveToPose::Response &res);
    bool executeMoveJ(neo_msgs::MoveJ::Request &req, neo_msgs::MoveJ::Response &res);
    bool executeMoveL(neo_msgs::MoveL::Request &req, neo_msgs::MoveL::Response &res);
    bool executeMoveP(neo_msgs::MoveP::Request &req, neo_msgs::MoveP::Response &res);
    bool executeRelease(neo_msgs::Release::Request &req, neo_msgs::Release::Response &res);
    bool executeHold(neo_msgs::Hold::Request &req, neo_msgs::Hold::Response &res);
    bool executeStop(neo_msgs::Stop::Request &req, neo_msgs::Stop::Response &res);
    bool executeRecover(neo_msgs::Recover::Request &req, neo_msgs::Recover::Response &res);
    bool executeCalibrate(neo_msgs::Calibrate::Request &req, neo_msgs::Calibrate::Response &res);
    bool executeReset(neo_msgs::Reset::Request &req, neo_msgs::Reset::Response &res);
    bool executeGetMotorIds(neo_msgs::GetMotorIds::Request &req, neo_msgs::GetMotorIds::Response &res);
    bool executeForward(neo_msgs::Forward::Request &req, neo_msgs::Forward::Response &res);
    bool executeInverse(neo_msgs::Inverse::Request &req, neo_msgs::Inverse::Response &res);
    bool executeGetInput(neo_msgs::GetInput::Request &req, neo_msgs::GetInput::Response &res);
    bool executeSetOutput(neo_msgs::SetOutput::Request &req, neo_msgs::SetOutput::Response &res);
    bool executeSetEOATAction(neo_msgs::SetEOATAction::Request &req, neo_msgs::SetEOATAction::Response &res);
};

#endif /* AUBO_DRIVER_H_ */