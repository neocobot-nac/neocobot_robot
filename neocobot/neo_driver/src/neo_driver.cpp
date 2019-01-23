#include "neo_driver/neo_driver.h"

NeoDriver::NeoDriver(std::string host, int port, std::string name, bool debug)
{
    if(robot.RobotLogin(host.c_str(), port) == RobotEvent_Succeed)
    {
        ROS_WARN("Connect robot server %s:%d success!",host.c_str(), port);
        _isConnected = true;

        SetupParam parameters;
        RobotEvent result;
        if (debug)
        {
            result = robot.Setup(name.c_str(), parameters, SetupMode_VirtualMode);
        }
        else
        {
            result = robot.Setup(name.c_str(), parameters, SetupMode_RealMode);
        }

        if (result == RobotEvent_Succeed)
        {
            result == robot.get_motor_ids(ids_size, motor_ids);
            if (result == RobotEvent_Succeed)
            {
                angles_publisher = handle.advertise<neo_msgs::JointAngles>("robot_angles", 1);
                velocity_publisher = handle.advertise<neo_msgs::JointVelocity>("robot_velocity", 1);
                current_publisher = handle.advertise<neo_msgs::JointCurrent>("robot_current", 1);
                ROS_WARN("Topic node setup succeed");

                MoveToAngles_server = handle.advertiseService("robot_move_to_angles", &NeoDriver::executeMoveToAngles, this);
                MoveToPose_server = handle.advertiseService("robot_move_to_pose", &NeoDriver::executeMoveToPose, this);

                Release_server = handle.advertiseService("robot_release", &NeoDriver::executeRelease, this);
                Hold_server = handle.advertiseService("robot_hold", &NeoDriver::executeHold, this);
                Stop_server = handle.advertiseService("robot_stop", &NeoDriver::executeStop, this);
                Recover_server = handle.advertiseService("robot_recover", &NeoDriver::executeRecover, this);
                Calibrate_server = handle.advertiseService("robot_calibrate", &NeoDriver::executeCalibrate, this);
                Reset_server = handle.advertiseService("robot_reset", &NeoDriver::executeReset, this);

                GetMotorIds_server = handle.advertiseService("robot_get_motor_ids", &NeoDriver::executeGetMotorIds, this);

                Forward_server = handle.advertiseService("robot_forward", &NeoDriver::executeForward, this);
                Inverse_server = handle.advertiseService("robot_inverse", &NeoDriver::executeInverse, this);

                GetInput_server = handle.advertiseService("robot_get_input", &NeoDriver::executeGetInput, this);
                SetOutput_server = handle.advertiseService("robot_set_output", &NeoDriver::executeSetOutput, this);
                
                SetEOATAction_server = handle.advertiseService("robot_set_EOAT_action", &NeoDriver::executeSetEOATAction, this);
                ROS_WARN("Service node setup succeed");

                publish_timer = handle.createTimer(ros::Duration(0.1),&NeoDriver::timerCallback,this);
                //publish_timer.stop();
                publish_timer.start();
                ROS_WARN("Timer start");

                _isSetup = true;
                ROS_WARN("Robot setup succeed");
            }
            else
            {
                _isSetup = false;
                ROS_WARN("Robot setup failed");
                exit(1);
            }
        }
        else
        {
            _isSetup = false;
            ROS_WARN("Robot setup failed");
            exit(1);
        }
    }
    else
    {
        ROS_WARN("Connect robot server %s:%d failure!",host.c_str(),port);
        _isConnected = false;
        exit(1);
    }
}

NeoDriver::~NeoDriver()
{
    publish_timer.stop();
    if(_isSetup)
    {
        robot.Shutdown();
        _isSetup = false;
        ROS_WARN("Robot shutdown succeed");
    }

    if(robot.RobotLogout() == RobotEvent_Succeed)
    {
        ROS_WARN("Logout succeed");
        _isConnected = false;
    }
    else
    {
        ROS_WARN("Logout failed!");
    }
}

void NeoDriver::timerCallback(const ros::TimerEvent& e)
{
    neo_msgs::JointAngles joint_angles;
    neo_msgs::JointVelocity joint_velocity;
    neo_msgs::JointCurrent joint_current;

    vector<double> _angles_msg;
    vector<double> _velocity_msg;
    vector<double> _current_msg;

    double _angles[MAX_MOTORS];
    double _velocity[MAX_MOTORS];
    double _current[MAX_MOTORS];

    if(_isSetup)
    {
        robot.get_angles(ids_size, motor_ids, _angles);
        robot.get_velocity(ids_size, motor_ids, _velocity);
        robot.get_current(ids_size, motor_ids, _current);

        for (size_t i = 0; i < ids_size; i++)
        {
            joint_angles.angles.push_back(_angles[i]);
            joint_velocity.velocity.push_back(_velocity[i]);
            joint_current.current.push_back(_current[i]);
        }

        angles_publisher.publish(joint_angles);
        velocity_publisher.publish(joint_velocity);
        current_publisher.publish(joint_current);
    }
}

bool NeoDriver::executeMoveToAngles(neo_msgs::MoveToAngles::Request &req, neo_msgs::MoveToAngles::Response &res)
{
    size_t _size = req.motor_ids.size();
    int _motor_ids[MAX_MOTORS];
    double _angles[MAX_MOTORS];

    for (size_t i = 0; i < _size; i++)
    {
        _motor_ids[i] = req.motor_ids.at(i);
        _angles[i] = req.angles.at(i);
    }

    float _velocity = req.velocity;
    float _acceleration = req.velocity;

    RelativeMode _mode;
    if (req.relative == true)
    {
        _mode = RelativeMode_Relative;
    }
    else
    {
        _mode = RelativeMode_Absolute;
    }

    RobotEvent ret = robot.move_to_angles(_size, _motor_ids, _angles, _velocity, _acceleration, _mode);

    if (req.block == true)
    {
        RobotEvent ret = robot.wait_for_motors(_size, _motor_ids);
    }

    res.event = ret;
    return true;
}

bool NeoDriver::executeMoveToPose(neo_msgs::MoveToPose::Request &req, neo_msgs::MoveToPose::Response &res)
{
    Pose _pose;
    _pose.position.x = req.pose.x;
    _pose.position.y = req.pose.y;
    _pose.position.z = req.pose.z;
    _pose.orientation.Rx = req.pose.Rx;
    _pose.orientation.Ry = req.pose.Ry;
    _pose.orientation.Rz = req.pose.Rz;

    float _velocity = req.velocity;
    float _acceleration = req.velocity;

    RelativeMode _mode;
    if (req.relative == true)
    {
        _mode = RelativeMode_Relative;
    }
    else
    {
        _mode = RelativeMode_Absolute;
    }

    RobotEvent ret = robot.move_to_pose(_pose, _velocity, _acceleration, _mode);

    if (req.block == true)
    {
        RobotEvent ret = robot.wait_for_motors(ids_size, motor_ids);
    }

    res.event = ret;
    return true;
}

bool NeoDriver::executeRelease(neo_msgs::Release::Request &req, neo_msgs::Release::Response &res)
{
    size_t _size = req.motor_ids.size();
    int _motor_ids[MAX_MOTORS];

    for (size_t i = 0; i < _size; i++)
    {
        _motor_ids[i] = req.motor_ids.at(i);
    }

    RobotEvent ret = robot.release(_size, _motor_ids);

    res.event = ret;
    return true;
}

bool NeoDriver::executeHold(neo_msgs::Hold::Request &req, neo_msgs::Hold::Response &res)
{
    size_t _size = req.motor_ids.size();
    int _motor_ids[MAX_MOTORS];

    for (size_t i = 0; i < _size; i++)
    {
        _motor_ids[i] = req.motor_ids.at(i);
    }

    RobotEvent ret = robot.hold(_size, _motor_ids);

    res.event = ret;
    return true;
}

bool NeoDriver::executeStop(neo_msgs::Stop::Request &req, neo_msgs::Stop::Response &res)
{
    RobotEvent ret = robot.stop();

    res.event = ret;
    return true;
}

bool NeoDriver::executeRecover(neo_msgs::Recover::Request &req, neo_msgs::Recover::Response &res)
{
    RobotEvent ret = robot.recover();

    res.event = ret;
    return true;
}

bool NeoDriver::executeCalibrate(neo_msgs::Calibrate::Request &req, neo_msgs::Calibrate::Response &res)
{
    RobotEvent ret = robot.calibrate();

    res.event = ret;
    return true;
}

bool NeoDriver::executeReset(neo_msgs::Reset::Request &req, neo_msgs::Reset::Response &res)
{
    RobotEvent ret = robot.reset();

    res.event = ret;
    return true;
}

bool NeoDriver::executeGetMotorIds(neo_msgs::GetMotorIds::Request &req, neo_msgs::GetMotorIds::Response &res)
{
    for (size_t i = 0; i < ids_size; i++)
    {
        res.motor_ids.push_back(motor_ids[i]);
    }

    res.event = RobotEvent_Succeed;
    return true;
}

bool NeoDriver::executeForward(neo_msgs::Forward::Request &req, neo_msgs::Forward::Response &res)
{
    double _angles[MAX_MOTORS];
    Pose pose;

    for (size_t i = 0; i < ids_size; i++)
    {
        _angles[i] = req.angles.at(i);
    }

    RobotEvent ret = robot.forward(_angles, pose);

    res.pose.x = pose.position.x;
    res.pose.y = pose.position.y;
    res.pose.z = pose.position.z;
    res.pose.Rx = pose.orientation.Rx;
    res.pose.Ry = pose.orientation.Ry;
    res.pose.Rz = pose.orientation.Rz;

    res.event = ret;
    return true;
}

bool NeoDriver::executeInverse(neo_msgs::Inverse::Request &req, neo_msgs::Inverse::Response &res)
{
    Pose _pose;
    _pose.position.x = req.pose.x;
    _pose.position.y = req.pose.y;
    _pose.position.z = req.pose.z;
    _pose.orientation.Rx = req.pose.Rx;
    _pose.orientation.Ry = req.pose.Ry;
    _pose.orientation.Rz = req.pose.Rz;

    double _angles[MAX_MOTORS];

    RobotEvent ret = robot.inverse(_pose, _angles);

    for (size_t i = 0; i < ids_size; i++)
    {
        res.angles.push_back(_angles[i]);
    }

    res.event = ret;
    return true;
}

bool NeoDriver::executeGetInput(neo_msgs::GetInput::Request &req, neo_msgs::GetInput::Response &res)
{
    size_t _size = req.motor_ids.size();
    int _motor_ids[MAX_MOTORS];
    int _signal[MAX_MOTORS];

    for (size_t i = 0; i < _size; i++)
    {
        _motor_ids[i] = req.motor_ids.at(i);
    }

    RobotEvent ret = robot.get_input(_size, _motor_ids, _signal);

    for (size_t i = 0; i < _size; i++)
    {
        res.signal.push_back(_signal[i]);
    }

    res.event = ret;
    return true;
}

bool NeoDriver::executeSetOutput(neo_msgs::SetOutput::Request &req, neo_msgs::SetOutput::Response &res)
{
    size_t _size = req.motor_ids.size();
    int _motor_ids[MAX_MOTORS];
    int _signal[MAX_MOTORS];

    for (size_t i = 0; i < _size; i++)
    {
        _motor_ids[i] = req.motor_ids.at(i);
        _signal[i] = req.signal.at(i);
    }

    RobotEvent ret = robot.set_output(_size, _motor_ids, _signal);

    res.event = ret;
    return true;
}

bool NeoDriver::executeSetEOATAction(neo_msgs::SetEOATAction::Request &req, neo_msgs::SetEOATAction::Response &res)
{
    RobotEvent ret = robot.set_EOAT_action(req.name.c_str(), req.action.c_str());

    res.event = ret;
    return true;
}

int main(int argc, char **argv) 
{
	std::string host;
	int port;
    std::string name;
    bool debug;

	ros::init(argc, argv, "neo_driver");
	if (!(ros::param::get("~robot_ip", host))) 
    {
		if (argc > 1) 
        {
			host = argv[1];
		} 
        else
        {
            ROS_WARN("Could not get robot ip. Please supply it as command line parameter or on the parameter server as robot_ip");
			exit(1);
		}
	}

	if (!(ros::param::get("~port", port))) 
    {
		if (argc > 2) 
        {
			port = atoi(argv[2]);
		}
        else
		{
            port = 8301;
		}

		if((port <= 0) or (port >= 65535)) 
        {
            ROS_WARN("Reverse port value is not valid (Use number between 1 and 65534. Using default value of 8887");
            port = 8301;
		}
	}

    if (!(ros::param::get("~robot_name", name))) 
    {
		if (argc > 3) 
        {
			name = argv[3];
		}
        else
		{
            ROS_WARN("Could not get robot name. Please supply it as command line parameter or on the parameter server as robot_name");
			exit(1);
		}
	}

    if (!(ros::param::get("~debug", debug))) 
    {
		if (argc > 4) 
        {
			if(strcmp(argv[4], "true") == 0)
            {
                debug = true;
            }
            else
            {
                debug = false;
            }
		}
        else
		{
            ROS_WARN("Could not get debug mode. Please supply it as command line parameter or on the parameter server as debug");
			exit(1);
		}
	}

	NeoDriver neoDriver(host, port, name, debug);
    ros::AsyncSpinner spinner(6);
	spinner.start();
	ros::waitForShutdown();
	return(0);
}
