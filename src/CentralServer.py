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
        self.agentsList = [RLAgent(self.environment) for x in range(numAgents)]
        self.timers = Timer()

    def AggregateQTables(self):
        self.globalQTable = np.mean(self.localQTables, axis=0)
    
    def RunAgents(self, batches=1, printResults=False):
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

