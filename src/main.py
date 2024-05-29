from CentralServer import CentralServer
from GazeboBot import GazeboBot


def main():
    worldFile = "/home/student/catkin_ws/src/3806Robotics/src/world.txt"
    resultsFile = "/home/student/catkin_ws/src/3806Robotics/src/results.txt"
    server = CentralServer(1, worldFile)
    server.agentsList[0].environment.WriteWorld(resultsFile)
    server.RunAgents(batches = 1, printResults=True)
    # server.RunTest(worldFile, resultsFile)

if __name__ == "__main__":
    main()