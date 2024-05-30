from CentralServer import CentralServer
from GazeboBot import GazeboBot
import numpy as np
import os
import sys
import rospy

def RunSimulation(homeDir):
    worldFile = "world.txt"
    testFile = "agentWorld.txt"
    resultsFile = "results.txt"
    allTables = "allTables.txt"
    globalFile = "globalQTables.txt"

    # Running 1 agent for 1 batch on just a fixed for now to check gazebo works
    
    # Training model
    server = CentralServer(1, testFile)
    server.RunAgents(batches = 1, printResults=False)

    # Creating test path
    server.RunTest(testFile, resultsFile)

    # Plotting in gazebo
    simulation = GazeboBot(homeDir)
    simulation.ClearWorld()
    simulation.PlotWorld(server.environment.worldGrid, server.environment.gridSize, server.environment.stateTypes)
    simulation.SpawnRobot()
    simulation.RobotWalkPath()
    rospy.spin()

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("USAGE: python3 RunSimulation.py [HOMEDIRECTORY]")
        exit(0)
    homeDir = sys.argv[1]
    RunSimulation(homeDir)