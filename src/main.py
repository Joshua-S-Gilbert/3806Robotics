from CentralServer import CentralServer
from GazeboBot import GazeboBot
import numpy as np


def main():
    worldFile = "world.txt"
    testFile = "agentWorld.txt"
    resultsFile = "results.txt"
    server = CentralServer(1, worldFile)
    server.RunAgents(batches = 1, printResults=True)
    server.agentsList[0].environment.WriteWorld(testFile)
    server.RunTest(worldFile, resultsFile)
    

if __name__ == "__main__":
    main()