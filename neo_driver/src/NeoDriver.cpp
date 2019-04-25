#include "NeoDriver.h"

bool NeoDriver::Connect(const char* ip, short int port, const char* name, bool sim)
{
    bool ret = NeoSI.load();
    if (!ret)
    {
        ROS_ERROR("Load NeoServiceInterface library failed !!!");
        return false;
    }
    ROS_INFO("Load NeoServiceInterface library success!");

    NEOCStatus login_status = NeoSI.ServiceLoginFunc(ip, port);
    if(login_status != NEOCLIENT_OK)
    {
        ROS_ERROR("Connect robot server %s:%d failed !!!", ip, port);
        return false;
    }
    ROS_INFO("Connect robot server %s:%d success!",ip, port);

    unsigned int mode;
    if (sim)
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
        return false;
    }

    ROS_INFO("Robot setup succeed");
    NEOCStatus param_status = NeoSI.RobotGetJointIdsFunc(&ids_size, &joint_ids);
    if (param_status != NEOCLIENT_OK)
    {
        ROS_ERROR("Param setup failed !!!");
        return false;
    }
    ROS_INFO("Param setup succeed");

    ROS_INFO("111");

    return true;
}

bool NeoDriver::Disconnect()
{
    NeoSI.ServiceLogoutFunc();
    return true;
}

