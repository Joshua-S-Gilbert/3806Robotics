import numpy as np
import Environment
import RobotController

class RLAgent:
    """
        Class for using reinforcement learning 
        to train the agent to find a path

    """
    def __init__(self, environment:Environment):
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
            return self.Greedy()
        
    def RunTraining(self):
        ## maybe optimise alpha and epsilon here in future
        self.Train(gamma=0.99, alpha=0.1, epsilon=0.1, maxIteratons=1000, maxSteps=1000)

    def Train(self, gamma=0.99, 
              alpha=0.1, epsilon=0.1,
              maxIterations=1000, maxSteps=1000):

        """
            Function to train the agent
            Returns a path, trace of rewards and trace of path length

        """

        # Set Q values for obstacle to infinity
        #self.qTable[...] = 0
        self.qTable[self.environment.worldGrid == self.environment.stateTypes["obstacle"]] = -np.inf

        rewardTrace = np.empty()
        pathLengthTrace = np.empty()

        # train qTable until max Iterations 
        # maybe add stopping condition to avoid overfitting to a specific map???
        for i in range(maxIterations):
            # Safely setting init state (maybe change in future)
            self.robotController.state = self.environment.startingPos
            state = self.robotController.GetCurrentState()

            # Choose exploit action or an explore action
            actionNumber = self.EpsilonGreed(epsilon)

            rewards = np.empty()
            path = np.array(state)

            # pathfind until goal or maxSteps
            for step in range(maxSteps):
                # apply action
                reward = self.robotController.GetActionValue(actionNumber,
                                                             self.environment.worldGrid, 
                                                             self.environment.gridSize, 
                                                             self.environment.stateTypes)
                nextState = self.robotController.GetCurrentState()
                nextActionNumber = self.EpsilonGreed(epsilon)

                rewards.append(reward)
                path = np.vstack((path, nextState))

                # add new qValue to respective qValue in qTable
                self.qTable[state[0], state[1], actionNumber] += (
                    alpha * (reward * gamma * np.max(self.qTable[nextState[0], nextState[1], :]) - self.qTable[state[0], state[1], actionNumber])
                )

                if self.robotController.IsGoal(self.environment.worldGrid, 
                                               self.environment.gridSize, 
                                               self.environment.stateTypes):
                    self.qTable[nextState[0], nextState[1], nextActionNumber] = 0
                    break

                state = nextState
                actionNumber = nextActionNumber
            
            rewardTrace.append(np.sum(rewards))
            pathLengthTrace.append(step+1)
        return path, rewardTrace, pathLengthTrace

    def Test(self, maxSteps=1000):
        self.robotController.state = self.environment.startingPos
        state = self.robotController.GetCurrentState()
        # run greedy from final qTable
        actionNumber = np.argmax(self.qTable[state[0], state[1], :])
        path = np.array(state)
        for step in range(maxSteps):
            self.robotController.GetActionValue(actionNumber,
                                                self.environment.worldGrid,
                                                self.environment.gridSize,
                                                self.environment.stateTypes)
            nextState = self.robotController.GetCurrentState()
            nextActionNumber = np.argmax(self.qTable[nextState[0], nextState[1], :])
            path = np.vstack((path, nextState))
            if self.robotController.IsGoal(self.environment.worldGrid,
                                            self.environment.gridSize,
                                            self.environment.stateTypes):
                break
            actionNumber = nextActionNumber

    def WritePath(self, path, fileName):
        with open(fileName, "w") as file:
            for step in path:
                file.write(f"{step[0]},{step[1]}")