#ifndef _ROSUTILS_H
#define _ROSUTILS_H

#include <ros/ros.h>

#include <string>
using std::string;

#define PI 3.141592653
#define DegreeToRadian(degree) ((degree/180.0)*PI)
#define RadianToDegree(radian) ((radian/PI)*180.0)

static bool get_param(const char* name, string* value)
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
        return false;
    }
    return true;
}

#endif
