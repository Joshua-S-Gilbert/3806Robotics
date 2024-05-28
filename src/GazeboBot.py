import rospy
from gazebo_msgs.srv import SetModelState, GetModelState
from gazebo_msgs.msg import ModelState
import tf
import AbstractSimulator
import tf.transformations

class GazeboBot(AbstractSimulator.AbstractSimulator):
    def __init__(self, modelName):
        self.modelName=modelName
        rospy.wait_for_service("/gazebo/set_model_state")
        rospy.wait_for_service("/gazebo/get_model_state")
        self.SetStateService = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        self.GetStateService = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

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
