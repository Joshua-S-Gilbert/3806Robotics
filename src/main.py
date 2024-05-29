from CentralServer import CentralServer
from GazeboBot import GazeboBot
import numpy as np
import os


def main():
    worldFile = "world.txt"
    testFile = "agentWorld.txt"
    resultsFile = "results.txt"
    allTables = "allTables.txt"
    globalFile = "globalQTables.txt"
    server = CentralServer(20, worldFile)

    if (os.path.exists(f"./{globalFile}")):
        server.LoadGlobalTable(globalFile)
    if (os.path.exists(f"./{allTables}")):
        server.LoadAllTables(allTables)

    server.RunAgents(batches = 1, printResults=False)
    server.agentsList[0].environment.WriteWorld(testFile)

    server.SaveGlobalTable(globalFile)
    server.SaveAllTables(allTables)

    #temporary
    server.agentsList[0].environment.NewStartingPos()

    server.RunTest(worldFile, resultsFile)
    

if __name__ == "__main__":
    main()