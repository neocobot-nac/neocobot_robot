#include "NeoDriver.h"

NeoDriver::NeoDriver(const char* ip, short int port, const char* name, bool simulator)
{
    bool ret = NeoSI.load();
    if (!ret)
    {
        ROS_ERROR("Load NeoServiceInterface library failed !!!");
        exit(1);
    }
    NEOCStatus login_status = NeoSI.ServiceLoginFunc(ip, port);
    if(login_status != NEOCLIENT_OK)
    {
        ROS_ERROR("Connect robot server %s:%d failed !!!", ip, port);
        exit(1);
    }
    ROS_INFO("Connect robot server %s:%d success!",ip, port);

    unsigned int mode;
    if (simulator)
    {
        mode = OFFLINE;
    }
    else
    {
        mode = ONLINE;
    }

    NEOCStatus setup_status = NeoSI.RobotSetupFunc(name, mode);
    if (setup_status != NEOCLIENT_OK)
    {
        ROS_ERROR("Robot setup failed !!!");
        exit(1);
    }

    ROS_INFO("Robot setup succeed");
    NEOCStatus param_status = NeoSI.RobotGetJointIdsFunc(&ids_size, &joint_ids);
    if (param_status != NEOCLIENT_OK)
    {
        ROS_ERROR("Param setup failed !!!");
        exit(1);
    }
    ROS_INFO("Param setup succeed");
}

NeoDriver::~NeoDriver()
{
    NeoSI.ServiceLogoutFunc();
}