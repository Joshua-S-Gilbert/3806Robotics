from CentralServer import CentralServer
from GazeboBot import GazeboBot
import numpy as np


def main():
    worldFile = "world.txt"
    testFile = "agentWorld.txt"
    resultsFile = "results.txt"
    server = CentralServer(1, testFile)
    server.agentsList[0].environment.WriteWorld(testFile)
    server.RunAgents(batches = 1, printResults=True)
    server.RunTest(testFile, resultsFile)

if __name__ == "__main__":
    main()