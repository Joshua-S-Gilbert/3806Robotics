import collections.abc as collections
import numpy as np

class Environment:
    def __init__(self, fn):
        self.ReadFile(fn)
    
    def ReadFile(self, fn):
        