#include "neo_driver/neo_driver.h"

NeoDriver::NeoDriver(std::string host, int port, std::string name)
{
    if(robot.RobotLogin(host.c_str(), port) == SUCCEED)
    {
        ROS_INFO("Connect robot server %s:%d success!",host.c_str(), port);
        _isConnected = true;

        SETUPPARAM parameters;
        if (robot.Setup(name.c_str(), parameters, REAL_MODE) == SUCCEED)
        {
            buildMap();

            _isSetup = true;
            ROS_INFO("Robot setup succeed");

            robot.get_motor_ids(motor_ids);

            publish_timer = handle.createTimer(ros::Duration(0.5),&NeoDriver::timerCallback,this);
            publish_timer.stop();
            angles_publisher = handle.advertise<neo_msgs::JointAngles>("robot_angles", 1);
            velocity_publisher = handle.advertise<neo_msgs::JointVelocity>("robot_velocity", 1);
            current_publisher = handle.advertise<neo_msgs::JointCurrent>("robot_current", 1);

            robotcmd_server = handle.advertiseService("robot_cmd", &NeoDriver::executeCMDCallback, this);

            publish_timer.start();
        }
        else
        {
            _isSetup = false;
            ROS_WARN("Robot setup failed");
        }
    }
    else
    {
        ROS_WARN("Connect robot server %s:%d failure!",host.c_str(),port);
        _isConnected = false;
    }
}

NeoDriver::~NeoDriver()
{
    publish_timer.stop();
    if(_isSetup)
    {
        robot.Shutdown();
    }

    if(robot.RobotLogout() == SUCCEED)
    {
        ROS_INFO("Logout succeed");
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

    vector<double> _angles;
    vector<double> _velocity;
    vector<double> _current;
    if(_isSetup)
    {
        if(robot.get_angles(motor_ids, _angles) == SUCCEED)
        {
            joint_angles.angles.assign(_angles.begin(), _angles.end());
        }
        if(robot.get_velocity(motor_ids, _velocity) == SUCCEED)
        {
            joint_velocity.velocity.assign(_velocity.begin(), _velocity.end());
        }
        if(robot.get_current(motor_ids, _current) == SUCCEED)
        {
            joint_current.current.assign(_current.begin(), _current.end());
        }

        angles_publisher.publish(joint_angles);
        velocity_publisher.publish(joint_velocity);
        current_publisher.publish(joint_current);
    }
}

void NeoDriver::buildMap()
{ 
	strFuncMap["move_to_angles"] = &NeoDriver::func_move_to_angles; 
	strFuncMap["move_to_position"] = &NeoDriver::func_move_to_position;
	strFuncMap["stop"] = &NeoDriver::func_stop; 
	strFuncMap["recover"] = &NeoDriver::func_recover;
    strFuncMap["release"] = &NeoDriver::func_release; 
	strFuncMap["hold"] = &NeoDriver::func_hold;
    strFuncMap["calibrate"] = &NeoDriver::func_calibrate; 
	strFuncMap["reset"] = &NeoDriver::func_reset;
    strFuncMap["test"] = &NeoDriver::func_test;
    strFuncMap["stop_test"] = &NeoDriver::func_stoptest;
}

bool NeoDriver::executeCMDCallback(neo_msgs::RobotCMD::Request &req, neo_msgs::RobotCMD::Response &res)
{
    std::string func_name = req.target;
    std::string func_args = req.args;

    int ret = callFunc(func_name, func_args);

    res.result = ret;
    return true;
}

int NeoDriver::callFunc(std::string& name, std::string &args) 
{
    ROS_INFO("Call func %s", name.c_str());
    int ret = 0;  
    if(strFuncMap.count(name))
    {
        ret = (this->*strFuncMap[name])(args);
    }
    else
    {
        ROS_WARN("Call func failed");
    }
    return ret;
}

int NeoDriver::func_move_to_angles(std::string args)
{
    vector<char> _motor_ids;
    vector<double> _angles;
    float _velocity;
    int _acceleration;
    int _relative;
    int _block;

    std::vector<string> args_div;
    __SplitString(args, args_div, ",");
    for (int i = 0; i != args_div.size(); i++)
    {
        std::vector<string> arg_div;
        __SplitString(args_div[i], arg_div, "=");
        string _name = arg_div[0];
        string _param = arg_div[1];

        _name.erase(std::remove(_name.begin(), _name.end(), ' '), _name.end());
        if (!_name.compare("motor_ids"))
        {
            _param.erase(std::remove(_param.begin(), _param.end(), '['), _param.end());
            _param.erase(std::remove(_param.begin(), _param.end(), ']'), _param.end());
            _param.erase(std::remove(_param.begin(), _param.end(), '\''), _param.end());

            std::vector<string> param_div;
            __SplitString(_param, param_div, " ");
            for (int j = 0; j != param_div.size(); j++)
            {
                char id = param_div[j][0];
                _motor_ids.push_back(id);
            }
        }
        else if(!_name.compare("angles"))
        {
            _param.erase(std::remove(_param.begin(), _param.end(), '['), _param.end());
            _param.erase(std::remove(_param.begin(), _param.end(), ']'), _param.end());
            _param.erase(std::remove(_param.begin(), _param.end(), '\''), _param.end());

            std::vector<string> param_div;
            __SplitString(_param, param_div, " ");
            for (int j = 0; j != param_div.size(); j++)
            {
                _angles.push_back(stof(param_div[j]));
            }
        }
        else if(!_name.compare("velocity"))
        {
            _velocity = stof(_param);
        }
        else if(!_name.compare("acceleration"))
        {
            _acceleration = stoi(_param);
        }
        else if(!_name.compare("relative"))
        {
            _relative = stoi(_param);
        }
        else if(!_name.compare("block"))
        {
            _block = stoi(_param);
        }
        else
        {
            return 0;
        }
    }

    robot.move_to_angles(_motor_ids, _angles, _velocity, _acceleration, _relative);
    if (_block)
    {
        robot.wait_for_motors(_motor_ids);
    }
    return 1;
}

int NeoDriver::func_move_to_position(std::string args)
{
    POSE _position;
    float _velocity;
    int _acceleration;
    int _relative;
    int _block;

    std::vector<string> args_div;
    __SplitString(args, args_div, ",");
    for (int i = 0; i != args_div.size(); i++)
    {
        std::vector<string> arg_div;
        __SplitString(args_div[i], arg_div, "=");
        string _name = arg_div[0];
        string _param = arg_div[1];

        _name.erase(std::remove(_name.begin(), _name.end(), ' '), _name.end());
        if(!_name.compare("position"))
        {
            _param.erase(std::remove(_param.begin(), _param.end(), '['), _param.end());
            _param.erase(std::remove(_param.begin(), _param.end(), ']'), _param.end());
            _param.erase(std::remove(_param.begin(), _param.end(), '\''), _param.end());

            std::vector<string> param_div;
            __SplitString(_param, param_div, " ");
            if (param_div.size() != 6)
            {
                return 0;
            }
            _position.position.x = stof(param_div[0]);
            _position.position.y = stof(param_div[1]);
            _position.position.z = stof(param_div[2]);
            _position.orientation.roll = stof(param_div[3]);
            _position.orientation.pitch = stof(param_div[4]);
            _position.orientation.yaw = stof(param_div[5]);
        }
        else if(!_name.compare("velocity"))
        {
            _velocity = stof(_param);
        }
        else if(!_name.compare("acceleration"))
        {
            _acceleration = stoi(_param);
        }
        else if(!_name.compare("relative"))
        {
            _relative = stoi(_param);
        }
        else if(!_name.compare("block"))
        {
            _block = stoi(_param);
        }
        else
        {
            return 0;
        }
    }

    robot.move_to_position(_position, _velocity, _acceleration, _relative);
    if (_block)
    {
        robot.wait_for_motors(motor_ids);
    }
    return 1;
}

int NeoDriver::func_stop(std::string args)
{
    robot.stop();
    return 1;
}

int NeoDriver::func_recover(std::string args)
{
    robot.recover();
    return 1;
}

int NeoDriver::func_release(std::string args)
{
    vector<char> _motor_ids;

    std::vector<string> args_div;
    __SplitString(args, args_div, "=");
    string _name = args_div[0];
    string _param = args_div[1];

    _name.erase(std::remove(_name.begin(), _name.end(), ' '), _name.end());
    if (!_name.compare("motor_ids"))
    {
        _param.erase(std::remove(_param.begin(), _param.end(), '['), _param.end());
        _param.erase(std::remove(_param.begin(), _param.end(), ']'), _param.end());
        _param.erase(std::remove(_param.begin(), _param.end(), '\''), _param.end());

        std::vector<string> param_div;
        __SplitString(_param, param_div, " ");
        for (int i = 0; i != param_div.size(); i++)
        {
            char id = param_div[i][0];
            _motor_ids.push_back(id);
        }
    }

    robot.release(_motor_ids);
    return 1;
}

int NeoDriver::func_hold(std::string args)
{
    vector<char> _motor_ids;

    std::vector<string> args_div;
    __SplitString(args, args_div, "=");
    string _name = args_div[0];
    string _param = args_div[1];

    _name.erase(std::remove(_name.begin(), _name.end(), ' '), _name.end());
    if (!_name.compare("motor_ids"))
    {
        _param.erase(std::remove(_param.begin(), _param.end(), '['), _param.end());
        _param.erase(std::remove(_param.begin(), _param.end(), ']'), _param.end());
        _param.erase(std::remove(_param.begin(), _param.end(), '\''), _param.end());

        std::vector<string> param_div;
        __SplitString(_param, param_div, " ");
        for (int i = 0; i != param_div.size(); i++)
        {
            char id = param_div[i][0];
            _motor_ids.push_back(id);
        }
    }

    robot.hold(_motor_ids);
    return 1;
}

int NeoDriver::func_calibrate(std::string args)
{
    robot.calibrate();
    return 1;
}

int NeoDriver::func_reset(std::string args)
{
    robot.reset();
    return 1;
}

int NeoDriver::func_test(std::string args)
{
    _isTest = true;

    vector<char> _motor_ids;
    vector<double> _angles;
    float _velocity;
    int _acceleration;
    int _relative;
    int _block;

    _motor_ids.push_back('1');
    _motor_ids.push_back('2');
    _motor_ids.push_back('3');
    _motor_ids.push_back('4');
    _motor_ids.push_back('5');
    _motor_ids.push_back('6');

    _velocity = 20.0;
    _acceleration = 70.0;
    _relative = 0;
    _block = 1;

    while(_isTest)
    {
        _angles.clear();
        _angles.push_back(getRandData(-30, 30));
        _angles.push_back(getRandData(-30, 30));
        _angles.push_back(getRandData(-60, 60));
        _angles.push_back(getRandData(-90, 90));
        _angles.push_back(getRandData(-60, 60));
        _angles.push_back(getRandData(-90, 90));

        robot.move_to_angles(_motor_ids, _angles, _velocity, _acceleration, _relative);
        if (_block)
        {
            robot.wait_for_motors(_motor_ids);
        }
    }

    return 1;
}

int NeoDriver::func_stoptest(std::string args)
{
    _isTest = false;
    robot.stop();
    sleep(0.5);
    robot.recover();
    return 1;
}

double NeoDriver::getRandData(int min, int max)
{
    double m1 = (double)(rand()%101)/101;
    min++;
    double m2 = (double)((rand()%(max-min+1))+min);
    m2=m2-1;
    return m1+m2;
}


void NeoDriver::__SplitString(const std::string &s, std::vector<string> &v, const std::string &c)
{
    v.clear();
    std::string::size_type pos1, pos2;
    pos2 = s.find(c);
    pos1 = 0;
    while (std::string::npos != pos2)
    {
        v.push_back(s.substr(pos1, pos2 - pos1));
        pos1 = pos2 + c.size();
        pos2 = s.find(c, pos1);
    }
    if (pos1 != s.length())
        v.push_back(s.substr(pos1));
}

int main(int argc, char **argv) {
	std::string host;
	int port;
    std::string name;

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

	NeoDriver neoDriver(host, port, name);

    ros::AsyncSpinner spinner(6);
	spinner.start();

	ros::waitForShutdown();

	return(0);
}
