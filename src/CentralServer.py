import numpy as np
from Environment import Environment
from RobotController import RobotController
from RLAgent import RLAgent
import time

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
        np.savetxt(fileName, self.globalQTable)

    def SaveAllTables(self, fileName="allTables.txt"):
        temp = np.asarray(self.localQTables)
        np.savetxt(fileName, temp)
    
    def LoadGlobalTable(self, fileName="globalQTable.txt"):
        self.globalQTable = np.loadtxt(fileName)

    def LoadAllTables(self, fileName="allTables.txt"):
        self.localQTables = list(np.loadtxt(fileName))

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
        path = agent.Test()
        agent.environment.WriteWorld("testWorld.txt")
        agent.WritePath(path, resultsFileName)
    

