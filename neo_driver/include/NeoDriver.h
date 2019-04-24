#ifndef _NEO_DRIVER_H
#define _NEO_DRIVER_H

#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include "NeoServiceInterface.h"

class NeoDriver
{
private:
    NeoServiceInterface NeoSI;

    size_t ids_size;
    int joint_ids;

public:
    NeoDriver(const char* ip, short int port, const char* name, bool simulator);
    ~NeoDriver();
};

#endif
