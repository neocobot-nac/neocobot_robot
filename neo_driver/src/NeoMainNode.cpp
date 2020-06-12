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
	ros::shutdown();
	ROS_INFO("Neo driver shutdown!");
}

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "neo_driver");

    string s_ip = "127.0.0.1";
    string s_port = "8301";
    string s_serial = "000027104E20";
    string s_timeout = "3";
    string s_transport = "0";
    string s_name = "NX63A1";
    string s_mode = "0";

    const char* ip;
    short int port;
    const char* serial;
    unsigned int timeout;
    int transport;
    const char* name;
    unsigned int mode;

    /* get robot_ip parameter */
    if (!(get_param("robot_ip", &s_ip)))
    {
        if (argc > 1)
        {
            s_ip = argv[1];
        }
        else
        {
            ROS_WARN("Could not get robot_ip parameter. Using default value: 127.0.0.1.");
            s_ip = "127.0.0.1";
        }
    }
    ip = s_ip.c_str();

    /* get robot_port parameter */
    if (!(get_param("robot_port", &s_port)))
    {
        if (argc > 2)
        {
            s_port = argv[2];
        }
        else
        {
            ROS_WARN("Could not get robot_port parameter. Using default value: 8301.");
            s_port = "8301";
        }
    }
    port = stoi(s_port);

    /* get robot_serial parameter */
    if (!(get_param("robot_serial", &s_serial)))
    {
        if (argc > 3)
        {
            s_serial = argv[3];
        }
        else
        {
            ROS_WARN("Could not get robot_serial parameter. Using default value.");
            s_serial = "000027104E20";
        }
    }
    serial = s_serial.c_str();

    /* get robot_timeout parameter */
    if (!(get_param("robot_timeout", &s_timeout)))
    {
        if (argc > 4)
        {
            s_timeout = argv[4];
        }
        else
        {
            ROS_WARN("Could not get robot_timeout parameter. Using default value.");
            s_timeout = "3";
        }
    }
    timeout = stoi(s_timeout);

    /* get robot_transport parameter */
    if (!(get_param("robot_transport", &s_transport)))
    {
        if (argc > 5)
        {
            s_transport = argv[5];
        }
        else
        {
            ROS_WARN("Could not get robot_transport parameter. Using default value.");
            s_transport = "0";
        }
    }
    transport = stoi(s_transport);

    /* get robot_name parameter */
    if (!(get_param("robot_name", &s_name))) 
    {
		    if (argc > 6) 
        {
			      s_name = argv[6];
		    }
        else
		    {
            ROS_WARN("Could not get robot_name parameter. Using default value: NX63A1.");
            s_name = "NX63A1";
		    }
	  }
    name = s_name.c_str();

    /* get robot_mode parameter */
    if (!(get_param("robot_mode", &s_mode))) 
    {
		    if (argc > 7) 
        {
			      s_mode = argv[7];
		    }
        else
		    {
            ROS_WARN("Could not get robot_mode parameter. Using default value.");
            s_mode = "0";
		    }
	  }
    mode = stoi(s_mode);

    /* neo_driver */
    NeoDriver neo_driver;

    if (!neo_driver.Connect(ip, port, serial, timeout, transport, name, mode))
    {
        ROS_ERROR("Connect Robot failed !!!");
        neo_driver.Disconnect();
        return(0);
    }

    signal(SIGINT, NeoSigintHandler);

    neo_driver.SetupROS();

    // ros::Rate loop_rate(100);
	  // while(ros::ok())
    // {
    //     neo_driver.UpdateData();
    //     ros::spinOnce();
		//     loop_rate.sleep();
	  // }

    ros::AsyncSpinner spinner(6);
    spinner.start();
    ros::waitForShutdown();
    neo_driver.Disconnect();
    return(0);
}
