import numpy as np
import asyncio
from collections import deque



class Pose:
    def __init__(self, position, orientation) -> None:
        self.lock = asyncio.Lock()
        self._position = position
        self._orientation = orientation
        
    @property
    async def position(self):
        async with self.lock:
            pos = self._position
        return pos
    
    @position.setter
    async def position(self, position):
        async with self.lock:
            self._position = position
            
            
    @property
    async def orientation(self):
        async with self.lock:
            orientation = self._orientation
        return orientation
    
    @orientation.setter
    async def orientation(self, orientation):
        async with self.lock:
            self._orientation = orientation
            
class Trajectory(deque):
    
    '''
    class to store a collection of current and past poses
    '''
    def __init__(self,
                 maxlen: int):
        super().__init__(maxlen=maxlen)
    
    @property    
    def current(self) -> Pose:
        return self[-1]
    
    @property
    def last(self)-> Pose:
        return self[-2]
    
    @property
    def before_last(self)-> Pose:
        return self[-3]
