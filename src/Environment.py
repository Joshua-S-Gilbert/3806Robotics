import collections.abc as collections
import numpy as np
import random

class Environment:
    def __init__(self, fileName):
        #map data
        self.unvisited = "O"
        self.visited = "V"
        self.obstacle = "B"
        self.numberObstacles = 4
        self.target = "G"
        self.numberTargets = 1
        self.gridSize = [8,8]
        self.startingPos = [1,0]
        self.generateGrid = False
        self.worldGrid = []

        self.ReadFile(fileName)
    
    def ReadFile(self, fileName):
        try:
            with open(fileName, "r") as file:
                lines = file.readlines()
                for i in range(9):
                    # prop meants property. there is a default method called property
                    prop, value = lines[i].strip().split()
                    if (hasattr(self, prop)):
                        setattr(self, prop, self.ConvertValue(value))
                    else:
                        print(f"Error: couldn't find attribute: {prop}")
                if not self.generateGrid:
                    gridlength = 0
                    self.worldGrid = []
                    for i in range(9, len(lines)):
                        self.worldGrid.append([])
                        for char in lines[i]:
                            self.worldGrid[gridlength].append(char)
                        gridlength+=1
                            
            if self.generateGrid:
                self.GenerateGrid()
                self.WriteWorld(fileName)
                self.generateGrid = False

        except FileNotFoundError:
            print(f"Error: Configuration file {fileName} not found")
        except Exception as error:
            print(f"Error: error occured loading config file: {error}")
    
    @staticmethod
    def ConvertValue(value=""):
        if value.find(",") != -1:
            tempList = []
            tempListstring = value.split(',')
            try:
                for i in range(2):  # tempListString should only ever be 2 size
                    tempList.append(int(tempListstring[i]))
            except Exception as error:
                print(f"Error: failed to read gridSize or startingPos. {error}")
            return tempList
        elif value.isdigit():
            return int(value)
        elif value == "True":
            return True
        elif value == "False":
            return False
        else:
            return value

    def GenerateGrid(self):
        self.worldGrid = [["O" for j in range(self.gridSize[1])] for i in range(self.gridSize[0])]
        self.PlaceItems(self.obstacle, self.numberObstacles)
        self.PlaceItems(self.obstacle, self.numberTargets)
    
    def PlaceItems(self, item, count):
        placedCount = 0
        while placedCount < count:
            row = random.randint(0, self.gridSize[0]-1)
            col = random.randint(0, self.gridSize[1]-1)
            if (self.worldGrid[row][col] == self.unvisited):
                self.worldGrid[row][col] = item
                placedCount += 1

    def WriteWorld(self, fileName):
        with open(fileName, "w") as file:
            file.write(f"unvisited {self.unvisited}")
            file.write(f"visited {self.visited}")
            file.write(f"obstacle {self.obstacle}")
            file.write(f"numberObstacles {self.numberObstacles}")
            file.write(f"target {self.target}")
            file.write(f"numberTargets {self.numberTargets}")
            file.write(f"gridSize {self.gridSize[0]},{self.gridSize[1]}")
            file.write(f"startingPos {self.startingPos[0]},{self.startingPos[1]}")
            file.write(f"generateGrid False")
            for line in self.worldGrid:
                tempstring = ""
                for char in line:
                    tempstring += char
                file.write(tempstring)

