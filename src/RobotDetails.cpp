#include <string>
#include <stdio.h>
#include <cstdlib>
#include <vector>
#include <AbstractSimulator.cpp>
#include <RobotMovement.cpp>
#include <ros/ros.h>
#include <gazebo_msgs/GetModelState.h>
#include <geometry_msgs/Pose.h>
#include <tf/tf.h>



class RobotDetails: public AbstractSimulator
{
public:
    std::string modelName;
    ros::ServiceClient setModelStateClient;
    ros::ServiceClient getModelStateClient;
    ros::NodeHandle n;

    RobotDetails(std::string name);

    bool SetPos(double x, double y);
    bool GetPos(double &x, double &y);
};

// im thinking the txt file goes here to initialise things.
// remember we can do things here, then call the base constructor later
RobotDetails::RobotDetails(std::string name)
{
    setModelStateClient = n.serviceClient<gazebo_msgs::SetModelState("/gazebo/set_model_state");
    getModelStateClient = n.serviceClient<gazebo_msgs::GetModelState("/gazebo/get_model_state");
    modelName = name;
}

bool RobotDetails::SetPos(double x, double y){
    gazebo_msgs::SetModelState setModelState;
    gazebo_msgs::ModelState modelState;

    modelState.model_name = modelName;
    modelState.pose.position.x = x;
    modelState.pose.position.y = y;

    // not sure about this part, found it online
    tf::Quaternion quat;
    quat.setRPY(0,0,0);
    modelState.pose.orientation.x = quat.x();
    modelState.pose.orientation.y = quat.y();
    modelState.pose.orientation.z = quat.z();
    modelState.pose.orientation.w = quat.w();

    setModelState.request.modle_state = modelState;

    if (!setModelStateClient.call(setModelState)){
        ROS_ERROR("Failed to call service set_model_state for: %s", modelName);
        return false;
    }
    return true;
}

bool RobotDetails::GetPos(double &x, double &y){
    gazebo_msgs::GetModelState getModelState;
    getModelState.request.model_name = modelName;

    if (getModelStateClient.call(getModelState)){
        x = getModelState.response.pose.position.x;
        y = getModelState.response.pose.position.y;
        return true;
    } else {
        ROS_ERROR("Failed to call service get_mdoel_state for: %s", modelName);
        return false;
    }
}


