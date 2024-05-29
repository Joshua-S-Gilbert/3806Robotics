import numpy as np
import random

class Environment:
    def __init__(self, fileName):
        #map data
        self.stateTypes = {"unvisited":"O",
                           "obstacle":"B",
                           "target":"G",
                           "invalid":"N"}
        self.numberObstacles = 0
        self.numberTargets = 1
        self.gridSize = [8,8]
        self.startingPos = np.asarray([0,0])
        self.generateGrid = False
        self.worldGrid = []

        self.ReadFile(fileName)

    def ReadFile(self, fileName):
        try:
            with open(fileName, "r") as file:
                lines = file.readlines()
                numVars = 9
                for i in range(numVars):
                    # prop meants property. there is a default method called property
                    prop, value = lines[i].strip().split()
                    self.ConvertValue(prop, value)
                if not self.generateGrid:
                    gridlength = 0
                    self.worldGrid = np.zeros((self.gridSize[0], self.gridSize[1]), dtype=str)
                    for i in range(numVars, len(lines)):
                        for j in range(len(lines[i])):
                            self.worldGrid[i][j]=lines[i][j]
                        gridlength+=1
                            
            if self.generateGrid:
                self.GenerateGrid()

        except FileNotFoundError:
            print(f"Error: Configuration file {fileName} not found")

    def ConvertValue(self, prop:str, value:str):
        if value.find(",") != -1:
            tempList = []
            tempListstring = value.split(',')
            try:
                for i in range(2):  # tempListString should only ever be 2 size
                    tempList.append(int(tempListstring[i]))
            except Exception as error:
                print(f"Error: failed to read gridSize or startingPos. {error}")
            self.UpdateProperty(prop, tempList)
        elif value.isdigit():
            self.UpdateProperty(prop, int(value))
        elif value == "True":
            self.UpdateProperty(prop, True)
        elif value == "False":
            self.UpdateProperty(prop, False)
        else:
            self.stateTypes[prop] = value
    
    def UpdateProperty(self, prop:str, value):
        if hasattr(self, prop):
            setattr(self, prop, value)
        else:
            print(f"Error: couldnt find property {prop}")

    def GenerateGrid(self):
        self.worldGrid = np.full((self.gridSize[0], self.gridSize[1]), fill_value="O", dtype=str)
        self.PlaceItems(self.stateTypes["obstacle"], self.numberObstacles)
        self.PlaceItems(self.stateTypes["target"], self.numberTargets)
        FoundStartingPosition = False
        while not FoundStartingPosition:
            row = random.randint(0, self.gridSize[0]-1)
            col = random.randint(0, self.gridSize[1]-1)
            if (self.worldGrid[row][col] == self.stateTypes["unvisited"]):
                self.startingPos = np.asarray([row, col])
                FoundStartingPosition=True

    def PlaceItems(self, item, count):
        placedCount = 0
        while placedCount < count:
            row = random.randint(0, self.gridSize[0]-1)
            col = random.randint(0, self.gridSize[1]-1)
            if (self.worldGrid[row][col] == self.stateTypes["unvisited"]):
                self.worldGrid[row][col] = item
                placedCount += 1

    def WriteWorld(self, fileName):
        with open(fileName, "w") as file:
            file.write(f"unvisited {self.stateTypes['unvisited']}\n")
            file.write(f"obstacle {self.stateTypes['obstacle']}\n")
            file.write(f"numberObstacles {self.numberObstacles}\n")
            file.write(f"target {self.stateTypes['target']}\n")
            file.write(f"invalid {self.stateTypes['invalid']}\n")
            file.write(f"numberTargets {self.numberTargets}\n")
            file.write(f"gridSize {self.gridSize[0]},{self.gridSize[1]}\n")
            file.write(f"startingPos {self.startingPos[0]},{self.startingPos[1]}\n")
            file.write(f"generateGrid False\n")
            for line in self.worldGrid:
                tempstring = ""
                for char in line:
                    tempstring += char
                tempstring += "\n"
                file.write(tempstring)

