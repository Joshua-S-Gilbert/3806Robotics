import rospy
from gazebo_msgs.srv import SetModelState, GetModelState, SpawnModel, GetWorldProperties, DeleteModel
from gazebo_msgs.msg import ModelState
import tf
import AbstractSimulator
import tf.transformations
import time


class GazeboBot(AbstractSimulator.AbstractSimulator):
    def __init__(self, homeDir, robotModelName="turtlebot3_burger", obstacleModelName="cardboard_box", goalModelName="bowl", movementDelay=1):
        self.modelNames = {"robot" : robotModelName, "obstacle" : obstacleModelName, "target": goalModelName}
        self.modelCount = {self.modelNames["robot"] : 0, self.modelNames["obstacle"] : 0, self.modelNames["target"] : 0}
        self.movementDelay = movementDelay
        rospy.wait_for_service("/gazebo/set_model_state")
        rospy.wait_for_service("/gazebo/get_model_state")
        rospy.wait_for_service("/gazebo/spawn_sdf_model")
        rospy.wait_for_service("/gazebo/get_world_properties")
        rospy.wait_for_service("/gazebo/delete_model")
        rospy.init_node('gazebo_bot_node', anonymous=True)
        self.SetStateService = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        self.GetStateService = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        self.SpawnModelService = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        self.GetWorldPropertiesService = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)
        self.DeleteModelService = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        self.homeDir = homeDir

    def ClearWorld(self):
        try:
            worldProperties = self.GetWorldPropertiesService()
            removalModelNames = worldProperties.model_names
            for modelName in removalModelNames:
                if modelName != "ground_plane" and modelName != "sun":  # Don't delete default models
                    self.DeleteModelService(modelName)
            self.modelNames["robot"] = {self.modelCount["robot"] : 0, self.modelCount["obstacle"] : 0, self.modelCount["target"] : 0}
            rospy.loginfo("Cleared all models from Gazebo world.")
        except rospy.ServiceException as error:
            rospy.logerr(f"Service call failed: {error}")

    def PlotWorld(self, worldGrid, gridSize, stateTypes):
        for i in range(gridSize[0]):
            for j in range(gridSize[1]):
                if worldGrid[i][j] == stateTypes["obstacle"]:
                    self.SpawnModel(self.modelNames["obstacle"], i, j)
                elif worldGrid[i][j] == stateTypes["target"]:
                    self.SpawnModel(self.modelNames["target"], i, j)   

    def SpawnRobot(self, x, y):
        self.SpawnModel(self.modelNames["robot"], x, y)   

    def RobotWalkPath(self, fileName:str, robotNumber):
        with open(fileName, "r") as file:
            lines = file.readlines()
            for i in range(len(lines)):
                line = lines[i].strip().split(',')
                if robotNumber > self.modelCount[self.modelNames["robot"]] and i == 0:
                    self.SpawnRobot(float(line[0]), float(line[1]))
                    time.sleep(self.movementDelay*2)
                else:
                    self.SetPos(f"{self.modelNames['robot']}{robotNumber}", float(line[0]), float(line[1]))
                    time.sleep(self.movementDelay)

    def SpawnModel(self, modelName, x, y):
        modelXML = self.getModelXML(modelName)
        if not modelXML:
            rospy.logerr(f"Failed to get model XML for model type: {modelName}")
        try:
            resp = self.SpawnModelService(model_name=f"{modelName}{self.modelCount[modelName]+1}", model_xml=modelXML, reference_frame="world")
            self.modelCount[modelName] += 1
            self.SetPos(f"{modelName}{self.modelCount[modelName]}", x, y)
            return resp.status_message
        
        except rospy.ServiceException as error:
            rospy.logerr(f"Service call failed for {modelName}{self.modelCount[modelName]-1}: {error}")
            return None

    def SetPos(self, modelName, x, y):
        try:
            stateMsg = ModelState()
            stateMsg.model_name = modelName
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
        
    def GetPos(self, modelName):
        try:
            resp = self.GetStateService(modelName, '')
            if resp.success:
                return resp.pose.position
            else:
                rospy.logerr("Failed to get model state: %s" % resp.status_message)
        except rospy.ServiceException as error:
            rospy.logerr("Service call failed: %s" % error)
            return None
        
    def getModelXML(self, modelName):
        try:
            with open(f"{self.homeDir}catkin_ws/src/3806Robotics/models/{modelName}.sdf", "r") as file:
                model_xml = file.read()
            return model_xml
        except Exception as e:
            rospy.logerr(f"Error reading model file for {modelName}: {e}")
            return None