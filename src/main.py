from CentralServer import CentralServer
from GazeboBot import GazeboBot
import numpy as np
import os
import matplotlib as mat


def main():
    worldFile = "world.txt"
    testFile = "agentWorld.txt"
    resultsFile = "results.txt"
    allTables = "allTables.txt"
    globalFile = "globalQTables.txt"
    server = CentralServer(1, worldFile)

    # overwrite it so it doesnt contaminate testing
    if (os.path.exists(f"./{allTables}")):
        with open(f"./{allTables}", 'w'):
            pass

    server.RunAgents(batches = 1, printResults=False)
    server.agentsList[0].environment.WriteWorld(testFile)

    # server.SaveGlobalTable(globalFile)
    server.SaveAllTables(allTables)

    #temporary
    # server.agentsList[0].environment.NewStartingPos()
    print("agent: 1")
    server.RunStatsTest(100, testFile)

    # server.RunTest(testFile, resultsFile)
    for i in range(99):
        server = CentralServer(1, testFile)
        if (os.path.exists(f"./{allTables}")):
            server.LoadAllTables(allTables)
        print(f"agent: {i+2}")
        server.RunAgents(batches = 1, printResults=False)
        server.SaveAllTables(allTables)
        server.RunStatsTest(100, testFile)
    

if __name__ == "__main__":
    main()