#ifndef NEO_DRIVER_H_
#define NEO_DRIVER_H_

#include <ros/ros.h>
#include <string>
#include "neo_msgs/JointAngles.h"
#include "neo_msgs/JointVelocity.h"
#include "neo_msgs/JointCurrent.h"
#include "neo_msgs/RobotCMD.h"
#include "Robot.h"
#include <stdlib.h>


class NeoDriver {
private:
    Robot robot;
    vector<char> motor_ids;

    typedef int (NeoDriver::*pFunc)(std::string);
    std::map<std::string, pFunc> strFuncMap;

    bool _isConnected = false;
    bool _isSetup = false;
    bool _isTest = false;
    bool _isCafe = false;

    ros::NodeHandle handle;
    ros::Timer publish_timer;
    ros::Publisher angles_publisher;
    ros::Publisher current_publisher;
    ros::Publisher velocity_publisher;
    ros::ServiceServer robotcmd_server;

    void buildMap();
    int callFunc(std::string& str, std::string &args);

    int func_move_to_angles(std::string args);
    int func_move_to_position(std::string args);
    int func_stop(std::string args);
    int func_recover(std::string args);
    int func_release(std::string args);
    int func_hold(std::string args);
    int func_calibrate(std::string args);
    int func_reset(std::string args);
    int func_test(std::string args);
    int func_stoptest(std::string args);

    void __SplitString(const std::string &s, std::vector<string> &v, const std::string &c);
    double getRandData(int min, int max);
    
public:

	NeoDriver(std::string host, int port, std::string name);
    ~NeoDriver();

    void timerCallback(const ros::TimerEvent& e);
    bool executeCMDCallback(neo_msgs::RobotCMD::Request &req, neo_msgs::RobotCMD::Response &res);

};

#endif /* AUBO_DRIVER_H_ */