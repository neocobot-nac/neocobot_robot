#include "NeoDriver.h"
#include "NeoDriver.cpp"

#include <string>

int main(int argc, char **argv) 
{
    std::string ip;
    int port = 8301;
    std::string name;
    bool simulator;

    ros::init(argc, argv, "neo_driver");
    if (!(ros::param::get("~robot_ip", ip))) 
    {
        if (argc > 1)
        {
            ip = argv[1];
        }
        else
        {
            ROS_WARN("Could not get robot_ip. Please supply it as command line parameter or on the parameter server as robot_ip");
            exit(1);
        }
    }
	if (!(ros::param::get("~port", port))) 
    {
		if (argc > 2) 
        {
			port = atoi(argv[2]);
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
            ROS_WARN("Could not get robot_name. Please supply it as command line parameter or on the parameter server as robot_name");
			exit(1);
		}
	}
    if (!(ros::param::get("~simulator", simulator))) 
    {
		if (argc > 4) 
        {
			if(strcmp(argv[4], "true") == 0)
            {
                simulator = true;
            }
            else
            {
                simulator = false;
            }
		}
        else
		{
            ROS_WARN("Could not get simulator. Please supply it as command line parameter or on the parameter server as simulator");
			exit(1);
		}
    }

    NeoDriver neo_driver(ip.c_str(), port, name.c_str(), simulator);

    ros::AsyncSpinner spinner(6);
    spinner.start();
    ros::waitForShutdown();
    ROS_WARN("neo_dirver shutdown");
    return(0);
}
