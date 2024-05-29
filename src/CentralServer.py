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
        self.timers = [Timer() for x in range(numAgents)]

    def AggregateQTables(self):
        self.globalQTable = np.mean(self.localQTables, axis=0)
        print(self.globalQTable)
    
    def RunAgents(self, batches=5, printResults=False):
        for i in range(batches):
            for agent in range(len(self.agentsList)):
                self.timers[agent].Start()
                path, rewardTrace, pathLengthTrace = self.agentsList[agent].RunTraining()
                self.timers[agent].Stop()
                if (printResults):
                    print(f"agent: {agent} time: {self.timers[agent].GetDuration()}\t path: {path}\nreward trace: {rewardTrace}\npath length trace: {pathLengthTrace}")
                self.localQTables.append(self.agentsList[agent].qTable)
            self.AggregateQTables()
            self.UpdateAgents()
    
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
        agent.qTable = self.globalQTable
        path = agent.Test()
        agent.environment.WriteWorld("testWorld.txt")
        agent.WritePath(path, resultsFileName)
    

