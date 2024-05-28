import collections.abc as collections
import numpy as np
import matplotlib.pyplot as plt
import Environment

class RLAgent:
    """
        Class for using reinforcement learning 
        to train the agent to find a path

    """
    def __init__(self, environment)
        self.environment = environment
        self.size = environment.GetSize()
        self.numActions = len(environment.GetActions())
        self.qTable = np.zeros((self.size[0], self.size[1], self.numActions))

    def Greedy(self, state):
        return np.argmax(self.qTable[state[0], state[1]])

    def EpsilonGreed(self, epsilon, state):
        if np.random.rand() < e:
            return np.random.randint(self.numActions)
        else:
            return self.greedy(state)

    def Train(self, state, 
            startingPosition, gamma=0.99, 
            alpha=0.1, epsilon=0.1, 
            maxIterations=1000, maxSteps=1000):

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