#include <mutex>
#include <thread>

#include <ros/ros.h>

#include <actionlib/server/action_server.h>

#include <control_msgs/JointTolerance.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

#include <sensor_msgs/JointState.h>
#include <industrial_msgs/RobotStatus.h>

#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

#include "neo_msgs/Joints.h"
#include "neo_msgs/MoveJ.h"
#include "neo_msgs/Stop.h"

#include "metaType.h"
#include "rosUtils.h"

#include <string>
using std::string;

class FollowJointTrajectoryExecuter
{
public:
    typedef actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction> FollowJointTrajectoryActionServer;

    ros::NodeHandle handle;

    ros::ServiceClient movej_client = handle.serviceClient<neo_msgs::MoveJ>("/neo_driver/movej");
    ros::ServiceClient stop_client = handle.serviceClient<neo_msgs::Stop>("/neo_driver/stop");

    FollowJointTrajectoryActionServer actionServer;
    FollowJointTrajectoryActionServer::GoalHandle active_goal_handle;

private:
    string joint_names[MAX_JOINTS] = { "first_joint", "second_joint", "third_joint", "fourth_joint", "fifth_joint", "sixth_joint", "seventh_joint" };
    string action_name;
    trajectory_msgs::JointTrajectory trajectory_cache;

    bool has_active_goal;

public:
    FollowJointTrajectoryExecuter(string name) :
        action_name(name), has_active_goal(false), 
        actionServer(handle, name, 
                      boost::bind(&FollowJointTrajectoryExecuter::actionGoal, this, _1), 
                      boost::bind(&FollowJointTrajectoryExecuter::actionCancel, this, _1), false) 
    {
        actionServer.start();
    };

    ~FollowJointTrajectoryExecuter() 
    {
        if (has_active_goal)
        {
            neo_msgs::Stop msg;
            stop_client.call(msg);
        }
    };

    void actionGoal(FollowJointTrajectoryActionServer::GoalHandle goal_handle)
    {
        control_msgs::FollowJointTrajectoryResult result;
        if (has_active_goal)
        {
            neo_msgs::Stop msg;
            stop_client.call(msg);
            if (msg.response.status != NEOCOBOT_OK)
            {
                ROS_ERROR("Can not stop previous joint trajectory action with error code %ld.", msg.response.status);
                result.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_JOINTS;
                goal_handle.setRejected(result, "Can not stop previous joint trajectory action.");
                return;
            }
            active_goal_handle.setCanceled();
            has_active_goal = false;
        }

        if (!goal_handle.getGoal()->trajectory.points.empty())
        {
            trajectory_cache = goal_handle.getGoal()->trajectory;
            goal_handle.setAccepted();
            active_goal_handle = goal_handle;
            has_active_goal = true;

            neo_msgs::MoveJ msg;
            for(size_t i = 0; i < trajectory_cache.points.size(); i++)
            {
                neo_msgs::Joints point;
                for(size_t j = 0; j < trajectory_cache.points[i].positions.size(); j++)
                {
                    for(size_t k = 0; k < trajectory_cache.points[i].positions.size(); k++)
                    {
                        if(joint_names[j] == trajectory_cache.joint_names[k])
                        {
                            point.joints.push_back(RadianToDegree(trajectory_cache.points[i].positions[k]));
                            break;
                        }
                    }
                }
                msg.request.points.push_back(point);
            }

            msg.request.velocity = 20.0;
            msg.request.acceleration = 80.0;
            msg.request.interval = 0.025;
            msg.request.mode = OPEN_MODE;
            msg.request.loop = 1;
            movej_client.call(msg);
            if (msg.response.status != NEOCOBOT_OK)
            {
                ROS_ERROR("Can not follow joint trajectory with error code %ld.", msg.response.status);
                result.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_JOINTS;
                active_goal_handle.setAborted(result, "Can not follow joint trajectory.");
                has_active_goal = false;
                return;
            }
            if (has_active_goal)
            {
                active_goal_handle.setSucceeded();
                has_active_goal = false;
                ROS_INFO("Succeeded to follow joint trajectory.");
            }
            else
            {
                ROS_ERROR("Stop the controller.");
                active_goal_handle.setAborted();
            }
        }
        else
        {
            ROS_ERROR("Joint Trajectory is empty.");
            result.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_GOAL;
            goal_handle.setRejected(result, "Joint Trajectory is empty.");
            has_active_goal = false;
        }
    };

    void actionCancel(FollowJointTrajectoryActionServer::GoalHandle goal_handle)
    {
        control_msgs::FollowJointTrajectoryResult result;
        if (has_active_goal && (active_goal_handle == goal_handle))
        {
            has_active_goal = false;
            neo_msgs::Stop msg;
            stop_client.call(msg);
        }
    };
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "follow_joint_trajectory_application");

    string s_name = "NX63A1";

    /* get robot_name parameter */
    if (!(get_param("robot_name", &s_name))) 
    {
        ROS_ERROR("Could not get robot_name parameter. Using default value: %s.", s_name.c_str());
        return 0;
	  }

    string action_name = "/" + s_name + "/follow_joint_trajectory";
    FollowJointTrajectoryExecuter executer(action_name);
    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::waitForShutdown();
    return 0;
}













