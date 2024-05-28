import collections.abc as collections
import numpy as np
import Environment

class RobotController:
    def __init__(self, initState, worldGrid):
        # Initializing State
        if self.checkState(initState) == 'O':
            self.state = initState
        else:
            raise ValueError("Invalid initial state")

        # Initializing Goal(s)
        self.goalPositions = self.FindGoalLocations(worldGrid)

        # Initializing Actions
        self.left = np.asarray([0,-1])
        self.right = np.asarray([0,1])
        self.up = np.asarray([1,0])
        self.down = np.asarray([-1,0])
        self.actions = [self.left, self.right, self.up, self.down]

    def FindGoalLocations(self, worldGrid):
        foundGoals = np.where(npGrid == 'G')
        goalPositions = []
        for i in range(len(foundGoals[0])):
            goalPositions.append([foundGoals[0][i], foundGoals[1][i]])
        return goalPositions

    def GetCurrentState():
        return self.state

    def CheckState(self, state, worldGrid, gridSize):
        # check if state is in the form ndarray[x,y]
        if isinstance(state, collections.Iterable) and state.size == 2:
            if (state[0] < 0 or state[1] < 0 or
               state[0] >= gridSize[0] or state[1] >= gridSize[1]):
               return 'N'
            return worldGrid[state]
        else
            return 'F'

    def ActionValue(self, actionNumber):
        nextState = self.state + self.actions[actionNumber]
        nextStateType = self.CheckState(nextState, worldGrid, gridSize)
