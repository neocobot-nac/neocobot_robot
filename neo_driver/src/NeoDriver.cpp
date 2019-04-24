#include "NeoDriver.h"

NeoDriver::NeoDriver(const char* ip, short int port, const char* name, bool simulator)
{
    bool ret = NeoSI.load();
    if (!ret)
    {
        ROS_ERROR("Load NeoServiceInterface library failed !!!");
        exit(1);
    }
    NEOCStatus login_status = NeoSI.NeoServiceLoginFunc(ip, port)
    if(login_status != NEOCLIENT_OK)
    {
        ROS_ERROR("Connect robot server %s:%d failed !!!", ip, port);
        exit(1);
    }

    ROS_INFO("Connect robot server %s:%d success!",ip, port);
    SetupParam parameters;
    SetupMode mode;
    if (simulator)
    {
        mode = SetupMode_VirtualMode;
    }
    else
    {
        mode = SetupMode_RealMode;
    }

    NEOCStatus setup_status = NeoSI.RobotSetupFunc(name.c_str(), parameters, mode);
    if (setup_status != NEOCLIENT_OK)
    {
        ROS_ERROR("Robot setup failed !!!");
        exit(1);
    }

    ROS_INFO("Robot setup failed !!!");
    NEOCStatus param_status == NeoSI.robotGetJointIdsFunc(ids_size, motor_ids);
    if (param_status != NEOCLIENT_OK)
    {
        ROS_ERROR("Param setup failed !!!");
        exit(1);
    }
    ROS_INFO("Param setup succeed");
}

NeoDriver::~NeoDriver()
{
    NeoSI.NeoServiceLogoutFunc();
}