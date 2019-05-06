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
    NEOCStatus param_status = NeoSI.RobotGetJointIdsFunc(&ids_size, joint_ids);
    param_status |= NeoSI.RobotGetAnglesFunc(ids_size, joint_ids, joints_record);
    param_status |= NeoSI.RobotGetVelocityFunc(ids_size, joint_ids, velocity_record);
    param_status |= NeoSI.RobotGetStateFunc(robot_state);
    if (param_status != NEOCLIENT_OK)
    {
        ROS_ERROR("Param setup failed !!!");
        return false;
    }

    ROS_INFO("Param setup succeed");
    return true;
}

bool NeoDriver::Disconnect()
{
    NeoSI.ServiceLogoutFunc();
    return true;
}

void NeoDriver::SetupROSFunction()
{
    JointStatePublisher = _handle.advertise<sensor_msgs::JointState>("joint_states", 10);
    RobotStatusPublisher = _handle.advertise<industrial_msgs::RobotStatus>("robot_status", 10);
}

void NeoDriver::UpdateData()
{
    // publish joint_state.
    double joints[MAX_JOINTS];
    double velocity[MAX_JOINTS];
    NEOCStatus ret = NeoSI.RobotGetAnglesFunc(ids_size, joint_ids, joints);
    ret |= NeoSI.RobotGetVelocityFunc(ids_size, joint_ids, velocity);
    if (ret != NEOCLIENT_OK)
    {
        memcpy(joints, joints_record, sizeof(joints_record));
        memcpy(velocity, velocity_record, sizeof(velocity_record));
    }

    sensor_msgs::JointState joint_state;
    joint_state.header.stamp = ros::Time::now();
    joint_state.name.resize(ids_size);
    joint_state.position.resize(ids_size);
    joint_state.velocity.resize(ids_size);
    joint_state.effort.resize(ids_size);
    for(size_t i = 0; i < ids_size; i++)
    {
        joint_state.name[i] = joint_names[i];
        joint_state.position[i] = DegreeToRadian(joints[i]);
        joint_state.velocity[i] = DegreeToRadian(velocity[i]);
        joint_state.effort[i] = 0.0;
    }
    memcpy(joints_record, joints, sizeof(joints));
    memcpy(velocity_record, velocity, sizeof(velocity));
    JointStatePublisher.publish(joint_state);

    NeoSI.RobotGetStateFunc(robot_state);
    industrial_msgs::RobotStatus robot_status;
    robot_status.mode.val = (int8_t)2;
    robot_status.e_stopped.val = (int8_t)robot_state.EmergencyStop;
    robot_status.drives_powered.val = (int8_t)robot_state.PowerOn;
    robot_status.motion_possible.val = (int8_t)robot_state.ReadyToMotion;
    robot_status.in_motion.val = (int8_t)robot_state.InMotion;
    
    robot_status.in_error.val = (int8_t)
    robot_status.error_code = (int32_t)

}
