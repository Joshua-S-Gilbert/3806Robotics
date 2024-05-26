#include <gazebo_msgs/GetModelState.h>

class RobotMovement
{
private:
    /* data */
public:
    void Up(gazebo_msgs::GetModelState state){
        state.pose.position.y = round(state.pose.position.y+1);
    }

    void Down(gazebo_msgs::GetModelState state){
        state.pose.position.y = round(state.pose.position.y-1);
    }

    void Right(gazebo_msgs::GetModelState state){
        state.pose.position.x = round(state.pose.position.x+1);
    }

    void Left(gazebo_msgs::GetModelState state){
        state.pose.position.x = round(state.pose.position.x-1);
    }
};

