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
    gazebo_msgs::GetModelState getRobotPose;
    gazebo_msgs::SetModelState setRobotPose;
    ros::NodeHandle n;

    RobotMovement move;

    ros::ServiceClient client
    RobotDetails(/* args */);
    ~RobotDetails();

    void UpdateState(){
        if (client.call(getRobotPose)){
            setRobotPose.request.model_state = getRobotPose;
            client.call(setRobotPose);
            if (!setRobotPose.response.success){
                ROS_INFO("Error: didnt set robot post: %s", name);
            }
        }
    }
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

    getRobotPose.request.model_name = name;
    move.up(getRobotPose);
    UpdateState();
}
