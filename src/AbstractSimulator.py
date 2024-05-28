from abc import ABC, abstractmethod

class AbstractSimulator(ABC):
    @abstractmethod
    def __init__(self, modelName):
        self.modelName = modelName
        
    @abstractmethod
    def RunSimulation():
        pass

    @abstractmethod
    def SetPos(self, x, y):
        pass

    @abstractmethod
    def GetPos(self):
        pass