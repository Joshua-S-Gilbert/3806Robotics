import collections.abc as collections
import numpy as np

class RobotController:
    def __init__(self, initState, worldGrid, stateTypes):
        # Initializing State
        self.state = initState

        # Initializing Actions
        self.left = np.asarray([0,-1])
        self.right = np.asarray([0,1])
        self.up = np.asarray([-1,0])
        self.down = np.asarray([1,0])
        self.actions = np.asarray([self.left, self.right, self.up, self.down])

        # Defining rewards
        self.rewardStep = -1
        self.rewardPunish = -5
        self.rewardGoal = 30

        # Finding Goal(s)
        self.goalPositions = self.FindGoalLocations(worldGrid, stateTypes)

    def FindGoalLocations(self, worldGrid, stateTypes):
        foundGoals = np.where(worldGrid == stateTypes["target"])
        goalPositions = []
        for i in range(len(foundGoals[0])):
            goalPositions.append([foundGoals[0][i], foundGoals[1][i]])
        return goalPositions

    def GetCurrentState(self):
        return self.state

    def CheckState(self, state, worldGrid, gridSize, stateTypes):
        # check if state is in the form ndarray[x,y]
        if isinstance(state, collections.Iterable) and state.size == 2:
            if (state[0] < 0 or state[1] < 0 or
               state[0] >= gridSize[0] or state[1] >= gridSize[1]):
               return stateTypes["invalid"] # state not inside grid
            return worldGrid[state[0], state[1]]
        else:
            print("Warning: Invalid state" + str(state))
            return stateTypes["invalid"] # State not valid

    def GetActionValue(self, actionNumber, worldGrid, gridSize, stateTypes):
        nextState = np.asarray([self.state[0] + self.actions[actionNumber][0], self.state[1] + self.actions[actionNumber][1]])
        nextStateType = self.CheckState(nextState, worldGrid, gridSize, stateTypes)

        if nextStateType == stateTypes["obstacle"] or nextStateType == stateTypes["invalid"]:
            return self.rewardPunish
        elif nextStateType == stateTypes["target"]:
            self.state = nextState
            return self.rewardGoal
        else: # if unvisited
            self.state = nextState
            return self.rewardStep

    def IsGoal(self, worldGrid, gridSize, stateTypes):
        return self.CheckState(self.state, worldGrid, gridSize, stateTypes) == stateTypes["target"]

    def GetActions(self):
        return self.actions