from CentralServer import CentralServer
from GazeboBot import GazeboBot
import numpy as np


def main():
    worldFile = "/home/student/catkin_ws/src/3806Robotics/src/world.txt"
    testFile = "/home/student/catkin_ws/src/3806Robotics/src/agentWorld.txt"
    resultsFile = "/home/student/catkin_ws/src/3806Robotics/src/results.txt"
    server = CentralServer(1, testFile)
    server.agentsList[0].environment.WriteWorld(testFile)
    server.RunAgents(batches = 1, printResults=True)
    server.RunTest(worldFile, resultsFile)

if __name__ == "__main__":
    main()