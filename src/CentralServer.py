import numpy as np
import Environment
import RobotController
import RLAgent
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
        self.globalQTable = np.empty()
        self.localQTables = np.empty()
        self.AgentsList = [RLAgent(Environment(fileName)) for x in range(numAgents)]
        self.timers = [Timer() for x in range(numAgents)]

    def AggregateQTables(self):
        self.globalQTable = np.mean(self.localQTables, axis=0)
    
    def RunAgents(self, batches=5, printResults=False):
        for i in range(batches):
            for agent in len(self.AgentsList):
                self.timers[agent].Start()
                path, rewardTrace, pathLengthTrace = self.AgentsList[agent].RunTraining()
                self.timers[agent].Stop()
                if (printResults):
                    print(f"agent: {5} time: {self.timers[agent].GetDuration}\t path: {path}\nreward trace: {rewardTrace}\npath length trace: {pathLengthTrace}")

                self.localQTables = np.append(self.localQTables, self.AgentsList[agent].qTable)
            self.AggregateQTables()
            self.UpdateAgents()
    
    def UpdateAgents(self):
        if (self.globalQTable == np.empty()):
            print("Error: central server global q table is empty")
            return
        for agent in self.AgentsList:
            agent.qTable = self.globalQTable

    def RunTest(self, worldFileName="world.txt", resultsFileName="results.txt"):
        agent = RLAgent(Environment(worldFileName))
        agent.qTable = self.globalQTable
        path = agent.Test()
        agent.WritePath(path, resultsFileName)
    

