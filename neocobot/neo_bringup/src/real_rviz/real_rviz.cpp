#include <string>
#include <map>
#include <ros/ros.h>
#include <neo_msgs/JointAngles.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>

bool callbackFlag = false;
neo_msgs::JointAngles cur_angles;
std::map<int, std::string> joints_name;

void anglesCallback(const neo_msgs::JointAngles::ConstPtr &angles)
{
    cur_angles.angles.assign(angles->angles.begin(), angles->angles.end());
    callbackFlag = true;
}

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "real_rviz");
	ros::NodeHandle handle;

	ros::Publisher rvizJointStatesPublisher = handle.advertise<sensor_msgs::JointState>("joint_states", 1);
	ros::Subscriber anglesSubscriber = handle.subscribe("robot_angles", 100, anglesCallback);

	tf::TransformBroadcaster broadcaster;
    ros::Rate loop_rate(100);

    joints_name[1] = "first_joint";
    joints_name[2] = "second_joint";
    joints_name[3] = "third_joint";
    joints_name[4] = "fourth_joint";
    joints_name[5] = "fifth_joint";
    joints_name[6] = "sixth_joint";

	// message declarations
    sensor_msgs::JointState joint_states;
	while (ros::ok())
    {
        if (callbackFlag)
        {
            joint_states.header.stamp = ros::Time::now();
            int joint_size = cur_angles.angles.size();
            joint_states.name.resize(joint_size);
            joint_states.position.resize(joint_size);
            for(int i = 0; i < joint_size; i++)
            {
                joint_states.name[i] = joints_name[i+1].c_str();
                joint_states.position[i] = cur_angles.angles[i] / 180.0 * M_PI;
            }
            //send the joint state and transform
            rvizJointStatesPublisher.publish(joint_states);
        }
		loop_rate.sleep();
		ros::spinOnce();
	}
    return 0;
}
