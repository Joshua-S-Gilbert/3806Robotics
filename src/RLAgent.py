import collections.abc as collections
import numpy as np
import matplotlib.pyplot as plt
import Environment
import RobotController

class RLAgent:
    """
        Class for using reinforcement learning 
        to train the agent to find a path

    """
    def __init__(self, environment)
        self.environment = environment
        self.robotController = RobotController(self.environment.startingPos,
                                               self.environment.worldGrid, 
                                               self.environment.gridSize, 
                                               self.environment.stateTypes)
        self.qTable = np.zeros((self.environment.gridSize[0],
                                self.environment.gridSize[1], 
                                self.robotController.actions.size))

    def Greedy(self):
        return np.argmax(self.qTable[self.robotController.state[0], self.robotController.state[1]])

    def EpsilonGreed(self, epsilon):
        if np.random.rand() < epsilon:
            return np.random.randint(self.robotController.actions.size)
        else:
            return self.greedy(self.state)

    def Train(self, startingPosition, 
              gamma=0.99, alpha=0.1, 
              epsilon=0.1, maxIterations=1000, 
              maxSteps=1000):

        """
            Function to train the agent
            Returns a path, trace of rewards and trace of path length

        """

        # Set Q values for obstacle to infinity
        #self.qTable[...] = 0
        self.qTable[self.environment.worldGrid == 'B'] = -np.inf

        rewardTrace = []
        pathLengthTrace = []

        for i in range(maxIterations):
            # may change environment to robot...
            self.environment.init(startingPosition)
            self.environment.GetCurrentState()