import rospy
from gazebo_msgs.srv import SetModelState, GetModelState
from gazebo_msgs.msg import ModelState
import tf
from AbstractSimulator import AbstractSimulator
import tf.transformations
import time


class GazeboBot(AbstractSimulator):
    def __init__(self, modelName, movementDelay=1):
        self.modelName=modelName
        self.movementDelay = movementDelay
        rospy.wait_for_service("/gazebo/set_model_state")
        rospy.wait_for_service("/gazebo/get_model_state")
        rospy.init_node('gazebo_bot_node', anonymous=True)
        self.SetStateService = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        self.GetStateService = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

    def RunSimulation(self, fileName:str):
        with open(fileName, "r") as file:
            lines = file.readlines()
            for i in range(len(lines)):
                line = lines[i].strip().split(',')
                self.SetPos(float(line[0]), float(line[1]))
                time.sleep(self.movementDelay)


    def SetPos(self, x, y):
        try:
            stateMsg = ModelState()
            stateMsg.model_name = self.modelName
            stateMsg.pose.position.x = x
            stateMsg.pose.position.y = y
            stateMsg.pose.position.z = 0

            # only required to set state
            quaternion = tf.transformations.quaternion_from_euler(0, 0, 0)
            stateMsg.pose.orientation.x = quaternion[0]
            stateMsg.pose.orientation.y = quaternion[1]
            stateMsg.pose.orientation.z = quaternion[2]
            stateMsg.pose.orientation.w = quaternion[3]

            resp = self.SetStateService(stateMsg)
            return resp.status_message
        
        except rospy.ServiceException as error:
            rospy.logerr("service call failed for: %s" % error)
            return None
        
    def GetPos(self):
        try:
            resp = self.GetStateService(self.modelName, '')
            if resp.success:
                return resp.pose.position
            else:
                rospy.logerr("Failed to get model state: %s" % resp.status_message)
        except rospy.ServiceException as error:
            rospy.logerr("Service call failed: %s" % error)
            return None
