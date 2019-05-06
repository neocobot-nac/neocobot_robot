#include "NeoDriver.h"

#include <signal.h>
#include <string>

using std::string;
using std::boolalpha;
using std::istringstream;

bool get_param(const char* name, string* value)
{
    string param_name = "~";
    param_name = param_name + name;

    if (ros::param::has(param_name.c_str()))
    {
        ros::param::get(param_name.c_str(), *value);
    }
    else if (ros::param::has(name))
    {
        ros::param::get(name, *value);
    }
    else
    {
        *value = "";
        return false;
    }
    return true;
}

void NeoSigintHandler(int sig)
{
	ROS_INFO("Neo driver shutdown!");
	ros::shutdown();
}

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "neo_driver");

    string s_ip = "127.0.0.1";
    string s_name = "NX63P1";
    string s_sim = "true";

    const char* ip;
    const char* name;
    bool sim;

    /* get robot_ip parameter */
    if (!(get_param("robot_ip", &s_ip)))
    {
        if (argc > 1)
        {
            s_ip = argv[1];
        }
        else
        {
            ROS_WARN("Could not get robot_ip parameter. Using default value of 127.0.0.1.");
            s_ip = "127.0.0.1";
        }
    }
    ip = s_ip.c_str();

    /* get robot_name parameter */
    if (!(get_param("robot_name", &s_name))) 
    {
		if (argc > 2) 
        {
			s_name = argv[2];
		}
        else
		{
            ROS_WARN("Could not get robot_name parameter. Using default value of NX63P1.");
            s_name = "NX63P1";
		}
	}
    name = s_name.c_str();

    /* get sim parameter */
    if (!(get_param("sim", &s_sim))) 
    {
		if (argc > 3)
        {
			s_sim = argv[3];
		}
        else
		{
            ROS_WARN("Could not get sim parameter. Using default value of false.");
            s_sim = "false";
		}
    }
    istringstream(s_sim)>>boolalpha>>sim;
    ROS_INFO("Get parameters succeed.");

    /* neo_driver */
    NeoDriver neo_driver;
    neo_driver.SetupROSFunction();

    if (!neo_driver.Connect(ip, 8301, name, sim))
    {
        ROS_ERROR("Connect Robot failed !!!");
        neo_driver.Disconnect();
    }

    signal(SIGINT, NeoSigintHandler);

    ros::Rate loop_rate(10);
	while(ros::ok())
    {
        neo_driver.UpdateData();
        ros::spinOnce();
		loop_rate.sleep();
	}
    neo_driver.Disconnect();
    return(0);
}
