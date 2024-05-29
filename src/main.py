from CentralServer import CentralServer
from GazeboBot import GazeboBot


def main():
    server = CentralServer(3)
    server.RunAgents(printResults=True)
    server.RunTest()

if __name__ == "__main__":
    main()