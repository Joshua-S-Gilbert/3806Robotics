import numpy as np
import Environment
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
        self.env = Environment(fileName)
        self.globalQTable = np.zeros((self.env.gridSize[0], self.env.gridSize[1]))
        self.localQTables = np.empty()
        self.AgentsList = [RLAgent(Environment(fileName)) for x in range(numAgents)]
        self.timers = [Timer() for x in range(numAgents)]

    def AggregateQTables(self):
        self.globalQTable = np.mean(self.localQTables, axis=0)
    
    def RunAgents(self):
        for agent in len(self.AgentsList):
            self.timers[agent].Start()
            self.AgentsList[agent].Train()
            self.timers[agent].Stop()
            self.localQTables = np.append(self.localQTables, self.AgentsList[agent].qTable)
        self.AggregateQTables()
    
    def UpdateAgents(self):
        for agent in self.AgentsList:
            agent.qTable = self.globalQTable

