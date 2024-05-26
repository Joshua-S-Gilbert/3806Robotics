#include <gazebo_msgs/GetModelState.h>

class RobotMovement
{
private:
    /* data */
public:
    void MoveUp(gazebo_msgs::GetModelState state){
        state.pose.position.y += 1;
    }

    void MoveDown(gazebo_msgs::GetModelState state){
        state.pose.position.y -= 1;
    }

    void MoveRight(gazebo_msgs::GetModelState state){
        state.pose.position.x += 1;
    }

    void MoveLeft(gazebo_msgs::GetModelState state){
        state.pose.position.x -= 1;
    }
};

