#include "NeoDriver.h"
NeoDriver::NeoDriver(std::string action_name) : 
    follow_joint_trajectory_action_server(_handle, action_name, boost::bind(&NeoDriver::actionGoal, this, _1), boost::bind(&NeoDriver::actionCancel, this, _1), false)
{
    
}

NeoDriver::~NeoDriver()
{
    
}

void NeoDriver::SetupROS()
{
    joint_state_publisher = _handle.advertise<sensor_msgs::JointState>("joint_states", 10);
    robot_status_publisher = _handle.advertise<industrial_msgs::RobotStatus>("robot_status", 10);
    joint_path_command_publisher = _handle.advertise<trajectory_msgs::JointTrajectory>("joint_path_command", 100);

    move_joints_server = _handle.advertiseService("/neo_driver/move_joints", &NeoDriver::executeMoveJoints, this);
    move_endpos_server = _handle.advertiseService("/neo_driver/move_endpos", &NeoDriver::executeMoveEndpos, this);

    update_timer = _handle.createTimer(ros::Duration(0.1), &NeoDriver::UpdateData, this);
    update_timer.start();

    follow_joint_trajectory_action_server.start();
}

bool NeoDriver::Connect(string ip, string port, string serial, string timeout, string transport, string name, string mode)
{
    string s_ip = ip;
    string s_port = port;
    string s_serial = serial;
    string s_timeout = timeout;
    string s_transport = transport;
    string s_name = name;
    string s_mode = mode;

    bool ret = interfaces.load();
    if (!ret)
    {
        ROS_ERROR("Load NeoService interfaces failed !!!");
        return false;
    }
    ROS_INFO("Load NeoService interfaces success!");

    if(interfaces.ServiceLogin(s_ip.c_str(), stoi(s_port), s_serial.c_str(), stoi(s_timeout), stoi(s_transport)) != NEOCOBOT_OK)
    {
        ROS_ERROR("Connect robot server %s:%s failed !!!", s_ip.c_str(), s_port.c_str());
        return false;
    }
    ROS_INFO("Connect robot server %s:%s success!",s_ip.c_str(), s_port.c_str());

    if (interfaces.RobotSetup(s_name.c_str(), stoi(s_mode)) != NEOCOBOT_OK)
    {
        ROS_ERROR("Robot setup failed !!!");
        interfaces.ServiceLogout();
        return false;
    }
    ROS_INFO("Robot setup succeed");

    NEOStatus status = interfaces.GetJointIds(&joint_size, joint_ids);
    status |= interfaces.GetJoints(joint_size, joint_ids, joints_record);
    status |= interfaces.GetVelocity(joint_size, joint_ids, velocity_record);
    if (status != NEOCOBOT_OK)
    {
        ROS_ERROR("Parameters init failed !!!");
        return false;
    }
    ROS_INFO("Parameters init succeed");
    return true;
}

bool NeoDriver::Disconnect()
{
    ROS_INFO("Unload NeoService interfaces success!");
    interfaces.ServiceLogout();
    return true;
}

void NeoDriver::UpdateData(const ros::TimerEvent& e)
{
    // publish joint_state.
    int8_t error = 0;
    double joints[MAX_JOINTS];
    double velocity[MAX_JOINTS];
    NEOStatus status = interfaces.GetJoints(joint_size, joint_ids, joints);
    status |= interfaces.GetVelocity(joint_size, joint_ids, velocity);
    if (status != NEOCOBOT_OK)
    {
        memcpy(joints, joints_record, sizeof(joints_record));
        memcpy(velocity, velocity_record, sizeof(velocity_record));
        error = 1;
    }

    sensor_msgs::JointState joint_state;
    joint_state.header.stamp = ros::Time::now();
    joint_state.header.frame_id = "Real-time state data of Robot";
    joint_state.name.resize(joint_size);
    joint_state.position.resize(joint_size);
    joint_state.velocity.resize(joint_size);
    joint_state.effort.resize(joint_size);
    for(size_t i = 0; i < joint_size; i++)
    {
        joint_state.name[i] = joint_names[i];
        joint_state.position[i] = DegreeToRadian(joints[i]);
        joint_state.velocity[i] = DegreeToRadian(velocity[i]);
        joint_state.effort[i] = 0.0;
    }
    memcpy(joints_record, joints, sizeof(joints));
    memcpy(velocity_record, velocity, sizeof(velocity));
    joint_state_publisher.publish(joint_state);

    int inner_status = interfaces.GetInnerStatus();
    robot_status.mode.val = (int8_t)2;
    robot_status.e_stopped.val = (int8_t)inner_status & ST_EMERGENCY;
    robot_status.drives_powered.val = (int8_t)((inner_status & ST_READY) >> 9);
    robot_status.motion_possible.val = (int8_t)(1-((inner_status & ST_HALT) >> 1));
    robot_status.in_motion.val = (int8_t)(1 - ((inner_status & ST_RUN_NONE) >> 8));
    robot_status.in_error.val = (int8_t)error;
    robot_status.error_code = (int32_t)status;
    robot_status_publisher.publish(robot_status);
}

bool NeoDriver::executeMoveJoints(neo_msgs::MoveJoints::Request &req, neo_msgs::MoveJoints::Response &res)
{
    size_t _size = req.joint_ids.size();

    int _joint_ids[MAX_JOINTS];
    double _joints[MAX_JOINTS];

    for (size_t i = 0; i < _size; i++)
    {
        _joint_ids[i] = req.joint_ids.at(i);
        _joints[i] = req.joints.at(i);
    }

    double _velocity = req.velocity;
    double _acceleration = req.acceleration;
    unsigned int _mode = req.mode;
    bool _block = req.block;

    NEOStatus _status = interfaces.MoveJoints(_size, _joint_ids, _joints, _velocity, _acceleration, _mode);
    if (_block == true)
    {
        interfaces.WaitForJoints(_size, _joint_ids);
    }
    res.status = _status; 
    return true;
}

bool NeoDriver::executeMoveEndpos(neo_msgs::MoveEndpos::Request &req, neo_msgs::MoveEndpos::Response &res)
{
    Pose _pose;
    _pose.position.x = req.pose.x;
    _pose.position.y = req.pose.y;
    _pose.position.z = req.pose.z;
    _pose.orientation.Rx = req.pose.Rx;
    _pose.orientation.Ry = req.pose.Ry;
    _pose.orientation.Rz = req.pose.Rz;

    double _velocity = req.velocity;
    double _acceleration = req.acceleration;
    unsigned int _mode = req.mode;
    bool _block = req.block;
    NEOStatus _status = interfaces.MoveEndpos(_pose, _velocity, _acceleration, _mode);
    if (_block == true)
    {
        interfaces.WaitForJoints(joint_size, joint_ids);
    }
    res.status = _status; 
    return true;
}

void NeoDriver::actionGoal(FollowJointTrajectoryActionServer::GoalHandle goal_handle)
{
    control_msgs::FollowJointTrajectoryResult result;
    
    if (!goal_handle.getGoal()->trajectory.points.empty())
    {
        



        ROS_ERROR("Controller is not actived.");
        result.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_JOINTS;
        goal_handle.setRejected(result, "Controller is not actived.");
    }
    else
    {
        ROS_ERROR("Joint Trajectory is empty.");
        result.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_GOAL;
        goal_handle.setRejected(result, "Joint Trajectory is empty.");
    }
}

void NeoDriver::actionCancel(FollowJointTrajectoryActionServer::GoalHandle goal_handle)
{
    ROS_INFO("Follow Joint trajectory action cancel.");
}








