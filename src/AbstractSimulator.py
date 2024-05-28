from abc import ABC, abstractmethod

class AbstractSimulator(ABC):
    @abstractmethod
    def __init__(self, modelName, movementDelay=1):
        self.modelName = modelName
        self.movementDelay = movementDelay
        
    @abstractmethod
    def RunSimulation(fileName:str):
        pass

    @abstractmethod
    def SetPos(self, x, y):
        pass

    @abstractmethod
    def GetPos(self):
        pass