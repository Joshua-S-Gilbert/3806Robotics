import numpy as np
import os
import sys
import rospy
from abc import ABC, abstractmethod
import time
import random
from gazebo_msgs.srv import SetModelState, GetModelState, SpawnModel, GetWorldProperties, DeleteModel
from gazebo_msgs.msg import ModelState
import tf
import tf.transformations
import collections.abc as collections

class AbstractSimulator(ABC):
    @abstractmethod
    def __init__(self, modelName, movementDelay=1):
        self.modelName = modelName
        self.movementDelay = movementDelay
        
    @abstractmethod
    def RunSimulation(fileName:str):
        pass

    @abstractmethod
    def SetPos(self, x, y):
        pass

    @abstractmethod
    def GetPos(self):
        pass

class Timer:
    def __init__(self):
        self.start = time.time()
        self.end = time.time()
        self.duration = time.time()
    
    def Start(self):
        self.start = time.time()
    
    def Stop(self):
        self.end = time.time()
        self.duration = self.end - self.start
    
    def GetDuration(self):
        return self.duration

class CentralServer:
    def __init__(self, numAgents, fileName="world.txt"):
        self.environment = Environment(fileName)
        self.robotController = RobotController(self.environment.startingPos,
                                               self.environment.worldGrid,
                                               self.environment.stateTypes)
        self.globalQTable = None
        self.localQTables = []
        self.agentsList = [RLAgent(Environment(fileName)) for x in range(numAgents)]
        self.timers = Timer()

    def AggregateQTables(self):
        self.globalQTable = np.mean(self.localQTables, axis=0)
    
    def RunAgents(self, batches=5, printResults=False):
        self.timers.Start()
        for i in range(batches):
            for agent in range(len(self.agentsList)):
                path, rewardTrace, pathLengthTrace = self.agentsList[agent].RunTraining()
                if (printResults and agent == 0):
                    print(f"agent: {agent} path: {path}\nreward trace: {rewardTrace}\npath length trace: {pathLengthTrace}")
                agentTable = np.copy(self.agentsList[agent].qTable)
                obstacleIndices = np.where(agentTable == -100000)
                for i in range(len(obstacleIndices[0])):
                    agentTable[obstacleIndices[0][i]][obstacleIndices[1][i]][obstacleIndices[2][i]] = 0
                self.localQTables.append(agentTable)
            self.AggregateQTables()
            self.UpdateAgents()
        self.timers.Stop()
        if (printResults):
            print(self.globalQTable)
        print(f"Time Taken: {self.timers.GetDuration()}")
    
    def SaveGlobalTable(self, fileName="globalQTable.txt"):
        with open(fileName, 'w') as f:
            shape = np.shape(self.globalQTable)
            f.write(f"{shape[0]} {shape[1]} {shape[2]}\n")
            flatArray = self.globalQTable.flatten()
            for item in flatArray:
                f.write(f"{item}\n")

    def LoadGlobalTable(self, fileName="globalQTable.txt"):
        with open(fileName, 'r') as f:
            shape = tuple(map(int, f.readline().strip().split()))
            flatArray = []
            for line in f:
                flatArray.append(float(line.strip()))
            self.globalQTable = np.array(flatArray).reshape(shape)
    
    def SaveAllTables(self, fileName="allTables.txt"):
        temp = np.asarray(self.localQTables)
        with open(fileName, 'w') as f:
            shape = np.shape(temp)
            f.write(f"{shape[0]} {shape[1]} {shape[2]} {shape[3]}\n")
            flatArray = temp.flatten()
            for item in flatArray:
                f.write(f"{item}\n")

    def LoadAllTables(self, fileName="allTables.txt"):
        with open(fileName, 'r') as f:
            shape = tuple(map(int, f.readline().strip().split()))
            flatArray = []
            for line in f:
                flatArray.append(float(line.strip()))
            temp = np.array(flatArray).reshape(shape)
            self.localQTables = temp.tolist()

    def UpdateAgents(self):
        if (self.globalQTable is None):
            print("Error: central server global q table is empty")
            return
        for agent in self.agentsList:
            agent.qTable = self.globalQTable

    def RunTest(self, worldFileName="world.txt", resultsFileName="results.txt"):
        if self.globalQTable is None:
            print("Error: global q table not trained")
            return
        agent = RLAgent(Environment(worldFileName))
        agent.qTable = np.copy(self.globalQTable)
        path, foundGoal = agent.Test()
        agent.environment.WriteWorld("testWorld.txt")
        agent.WritePath(path, resultsFileName)
    
    def RunStatsTest(self, runs = 100, worldFileName="world.txt"):
        totalFoundGoal = 0
        agent = RLAgent(Environment(worldFileName))
        agent.qTable = np.copy(self.globalQTable)
        for i in range(runs):
            agent.environment.NewStartingPos()
            path, foundGoal = agent.Test()
            if foundGoal:
                totalFoundGoal += 1
        print(f"runs: {runs} found goals: {totalFoundGoal} percent success: {(totalFoundGoal/runs)*100}%")

class Environment:
    def __init__(self, fileName):
        #map data
        self.stateTypes = {"unvisited":"O",
                           "obstacle":"B",
                           "target":"G",
                           "invalid":"N"}
        self.numberObstacles = 0
        self.numberTargets = 1
        self.gridSize = [8,8]
        self.startingPos = np.asarray([0,0])
        self.generateGrid = False
        self.worldGrid = []

        self.ReadFile(fileName)

    def ReadFile(self, fileName):
        try:
            with open(fileName, "r") as file:
                lines = file.readlines()
                numVars = 9
                for i in range(numVars):
                    # prop meants property. there is a default method called property
                    prop, value = lines[i].strip().split()
                    self.ConvertValue(prop, value)
                if not self.generateGrid:
                    self.worldGrid = np.zeros((self.gridSize[0], self.gridSize[1]), dtype=str)
                    gridLength = 0
                    for i in range(numVars, len(lines)):
                        for j in range(self.gridSize[1]):
                            self.worldGrid[gridLength][j]=lines[i][j]
                        gridLength += 1
                            
            if self.generateGrid:
                self.GenerateGrid()

        except FileNotFoundError:
            print(f"Error: Configuration file {fileName} not found")

    def ConvertValue(self, prop:str, value:str):
        if value.find(",") != -1:
            tempList = []
            tempListstring = value.split(',')
            try:
                for i in range(2):  # tempListString should only ever be 2 size
                    tempList.append(int(tempListstring[i]))
            except Exception as error:
                print(f"Error: failed to read gridSize or startingPos. {error}")
            self.UpdateProperty(prop, tempList)
        elif value.isdigit():
            self.UpdateProperty(prop, int(value))
        elif value == "True":
            self.UpdateProperty(prop, True)
        elif value == "False":
            self.UpdateProperty(prop, False)
        else:
            self.stateTypes[prop] = value
    
    def UpdateProperty(self, prop:str, value):
        if hasattr(self, prop):
            setattr(self, prop, value)
        else:
            print(f"Error: couldnt find property {prop}")

    def GenerateGrid(self):
        self.worldGrid = np.full((self.gridSize[0], self.gridSize[1]), fill_value="O", dtype=str)
        self.NewObstaclePos()
        self.PlaceItems(self.stateTypes["target"], self.numberTargets)
        self.NewStartingPos()

    def PlaceItems(self, item, count):
        placedCount = 0
        while placedCount < count:
            row = random.randint(0, self.gridSize[0]-1)
            col = random.randint(0, self.gridSize[1]-1)
            if (self.worldGrid[row][col] == self.stateTypes["unvisited"]):
                self.worldGrid[row][col] = item
                placedCount += 1
    
    def NewObstaclePos(self):
        self.PlaceItems(self.stateTypes["obstacle"], self.numberObstacles)
    
    def NewStartingPos(self):
        FoundStartingPosition = False
        while not FoundStartingPosition:
            row = random.randint(0, self.gridSize[0]-1)
            col = random.randint(0, self.gridSize[1]-1)
            if (self.worldGrid[row][col] == self.stateTypes["unvisited"]):
                self.startingPos = np.asarray([row, col])
                FoundStartingPosition=True

    def WriteWorld(self, fileName):
        with open(fileName, "w") as file:
            file.write(f"unvisited {self.stateTypes['unvisited']}\n")
            file.write(f"obstacle {self.stateTypes['obstacle']}\n")
            file.write(f"numberObstacles {self.numberObstacles}\n")
            file.write(f"target {self.stateTypes['target']}\n")
            file.write(f"invalid {self.stateTypes['invalid']}\n")
            file.write(f"numberTargets {self.numberTargets}\n")
            file.write(f"gridSize {self.gridSize[0]},{self.gridSize[1]}\n")
            file.write(f"startingPos {self.startingPos[0]},{self.startingPos[1]}\n")
            file.write(f"generateGrid False\n")
            for line in self.worldGrid:
                tempstring = ""
                for char in line:
                    tempstring += char
                tempstring += "\n"
                file.write(tempstring)

class GazeboBot(AbstractSimulator):
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

class RLAgent:
    """
        Class for using reinforcement learning 
        to train the agent to find a path

    """
    def __init__(self, environment:Environment):
        self.environment = environment
        self.robotController = RobotController(self.environment.startingPos,
                                               self.environment.worldGrid, 
                                               self.environment.stateTypes)
        self.qTable = np.zeros((self.environment.gridSize[0],
                                self.environment.gridSize[1], 
                                self.robotController.actions.shape[0]), 
                                dtype=float)

    def Greedy(self):
        return np.argmax(self.qTable[self.robotController.state[0], self.robotController.state[1]])

    def EpsilonGreed(self, epsilon):
        if np.random.rand() < epsilon:
            return np.random.randint(self.robotController.actions.shape[0])
        else:
            return self.Greedy()
        
    def RunTraining(self):
        ## maybe optimise alpha and epsilon here in future
        path, rewardTrace, pathLengthTrace = self.Train(gamma=0.99, alpha=0.1, epsilon=0.1, maxIterations=500, maxSteps=1000)
        return path, rewardTrace, pathLengthTrace
    
    def Train(self, gamma=0.99, 
              alpha=0.1, epsilon=0.1,
              maxIterations=1000, maxSteps=1000):

        """
            Function to train the agent
            Returns a path, trace of rewards and trace of path length

        """

        # Set Q values for obstacle to infinity
        #self.qTable[...] = 0
        self.qTable[self.environment.worldGrid == self.environment.stateTypes["obstacle"]] = -100000

        rewardTrace = []
        pathLengthTrace = []

        # train qTable until max Iterations 
        # maybe add stopping condition to avoid overfitting to a specific map???
        for i in range(maxIterations):
            # Safely setting init state (maybe change in future)
            self.robotController.state = self.CopyState(self.environment.startingPos)
            state = self.CopyState(self.robotController.state)

            # Choose exploit action or an explore action
            actionNumber = self.EpsilonGreed(epsilon)

            rewards = 0
            path = np.array(state)

            # pathfind until goal or maxSteps
            for step in range(maxSteps):
                # apply action
                reward = self.robotController.GetActionValue(actionNumber,
                                                             self.environment.worldGrid, 
                                                             self.environment.gridSize, 
                                                             self.environment.stateTypes)
                nextState = self.CopyState(self.robotController.state)
                nextActionNumber = self.EpsilonGreed(epsilon)

                rewards += reward
                path = np.vstack((path, nextState))

                # add new qValue to respective qValue in qTable
                self.qTable[state[0], state[1], actionNumber] += (
                    alpha * (reward + gamma * np.max(self.qTable[nextState[0], nextState[1]]) - self.qTable[state[0], state[1], actionNumber])
                )

                if self.robotController.IsGoal(self.environment.worldGrid, 
                                               self.environment.gridSize, 
                                               self.environment.stateTypes):
                    self.qTable[nextState[0], nextState[1], nextActionNumber] = 0
                    break

                state = nextState
                actionNumber = nextActionNumber
            
            rewardTrace.append(rewards)
            pathLengthTrace.append(step+1)
            if (i%100 == 0):
                self.environment.NewStartingPos()
        return path, rewardTrace, pathLengthTrace

    def Test(self, maxSteps=1000):
        self.robotController.state = self.CopyState(self.environment.startingPos)
        state = self.CopyState(self.robotController.state)
        # run greedy from final qTable
        actionNumber = np.argmax(self.qTable[state[0], state[1]])
        path = np.array(state)
        foundGoal = False
        for step in range(maxSteps):
            self.robotController.GetActionValue(actionNumber,
                                                self.environment.worldGrid,
                                                self.environment.gridSize,
                                                self.environment.stateTypes)
            nextState = self.CopyState(self.robotController.state)
            nextActionNumber = np.argmax(self.qTable[nextState[0], nextState[1]])
            path = np.vstack((path, nextState))
            if self.robotController.IsGoal(self.environment.worldGrid,
                                            self.environment.gridSize,
                                            self.environment.stateTypes):
                foundGoal = True
                break
            actionNumber = nextActionNumber
        return path, foundGoal

    def WritePath(self, path, fileName):
        with open(fileName, "w") as file:
            for step in path:
                file.write(f"{step[0]},{step[1]}\n")

    def CopyState(self, originalState):
        return np.asarray((originalState[0], originalState[1]))

class RobotController:
    def __init__(self, initState, worldGrid, stateTypes):
        # Initializing State
        self.state = initState

        # Initializing Actions
        self.left = np.asarray([0,-1])
        self.right = np.asarray([0,1])
        self.up = np.asarray([-1,0])
        self.down = np.asarray([1,0])
        self.actions = np.asarray([self.left, self.right, self.up, self.down])

        # Defining rewards
        self.rewardStep = -1
        self.rewardPunish = -5
        self.rewardGoal = 30

        # Finding Goal(s)
        self.goalPositions = self.FindGoalLocations(worldGrid, stateTypes)

    def FindGoalLocations(self, worldGrid, stateTypes):
        foundGoals = np.where(worldGrid == stateTypes["target"])
        goalPositions = []
        for i in range(len(foundGoals[0])):
            goalPositions.append([foundGoals[0][i], foundGoals[1][i]])
        return goalPositions

    def GetCurrentState(self):
        return self.state

    def CheckState(self, state, worldGrid, gridSize, stateTypes):
        # check if state is in the form ndarray[x,y]
        if isinstance(state, collections.Iterable) and state.size == 2:
            if (state[0] < 0 or state[1] < 0 or
               state[0] >= gridSize[0] or state[1] >= gridSize[1]):
               return stateTypes["invalid"] # state not inside grid
            return worldGrid[state[0]][state[1]]
        else:
            print("Warning: Invalid state" + str(state))
            return stateTypes["invalid"] # State not valid

    def GetActionValue(self, actionNumber, worldGrid, gridSize, stateTypes):
        nextState = np.asarray([self.state[0] + self.actions[actionNumber][0], self.state[1] + self.actions[actionNumber][1]])
        nextStateType = self.CheckState(nextState, worldGrid, gridSize, stateTypes)

        if nextStateType == stateTypes["obstacle"] or nextStateType == stateTypes["invalid"]:
            return self.rewardPunish
        elif nextStateType == stateTypes["target"]:
            self.state = nextState
            return self.rewardGoal
        else: # if unvisited
            self.state = nextState
            return self.rewardStep

    def IsGoal(self, worldGrid, gridSize, stateTypes):
        return self.CheckState(self.state, worldGrid, gridSize, stateTypes) == stateTypes["target"]

    def GetActions(self):
        return self.actions

def RunSimulation(homeDir):
    worldFile = "world.txt"
    testFile = "agentWorld.txt"
    resultsFile = "results.txt"
    allTables = "allTables.txt"
    globalFile = "globalQTables.txt"

    # Running 1 agent for 1 batch on just a fixed for now to check gazebo works
    
    # Training model
    server = CentralServer(1, worldFile)
    server.RunAgents(batches = 1, printResults=False)

    # Creating test path
    server.RunTest(worldFile, resultsFile)

    # Plotting in gazebo
    simulation = GazeboBot(homeDir)
    simulation.ClearWorld()
    simulation.PlotWorld(server.environment.worldGrid, server.environment.gridSize, server.environment.stateTypes)
    simulation.RobotWalkPath(testFile, 1)
    rospy.spin()

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("USAGE: python3 RunSimulation.py [HOMEDIRECTORY]")
        exit(0)
    homeDir = sys.argv[1]
    RunSimulation(homeDir)