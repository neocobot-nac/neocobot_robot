#ifndef _NEO_DRIVER_H
#define _NEO_DRIVER_H

#include <ros/ros.h>
#include "NeoServiceInterface.h"

class NeoDriver
{
private:
    NeoServiceInterface NeoSI;

public:
    NeoDriver(const char* ip, short int port, const char* name, bool simulator);
    ~NeoDriver();
};

#endif
