#include <string>
#include <stdio.h>
#include <cstdlib>
#include <vector>
#include <RobotMovement.cpp>
#include <ros/ros.h>
#include <gazebo_msgs/GetModelState.h>

// using namespace std;

class RobotDetails
{
public:
    std::string name;
    gazebo_msgs::GetModelState robotPose;
    ros::NodeHandle n;

    RobotMovement move;

    ros::ServiceClient client
    RobotDetails(/* args */);
    ~RobotDetails();
};

// im thinking the txt file goes here to initialise things. 
// remember we can do things here, then call the base constructor later
RobotDetails::RobotDetails(/* args */)
{
}

RobotDetails::~RobotDetails()
{
    ros::ServiceClient client = 
    n.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");

    robotPose.request.model_name = name;

    if (!client.call(robotPose)){
        ROS_INFO("Error: unsuccessful robot info retrieval: %s", name);
        rate.sleep();
        continue;
    }
}
