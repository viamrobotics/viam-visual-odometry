import numpy as np
import asyncio
from collections import deque

class Transition:
    def __init__(self, R = np.eye(3).astype(np.float64),
                 t = np.zeros(shape=(3, 1)).astype(np.float64),
                 dt = 0 ):
        self.R = R
        self.t = t
        self.dt = dt
        
        self.point_cloud = None
        self.start:int
        self.end:int
        
        
class Motion(deque):
    '''
    class to store a collection of current and past Transitions
    '''
    def __init__(self,
                 maxlen: int):
        super().__init__(maxlen=maxlen)
        self.lock = asyncio.Lock()
        
    @property    
    def current(self) -> Transition:
        if len(self)>0:
            return self[-1]
        else:
            return Transition()
    
    @property
    def last(self)-> Transition:
        return self[-2]
    
    @property
    def before_last(self)-> Transition:
        return self[-3]
    
    async def get_current_transition_values(self):
        return await self.get_values_at_index(-1)
        
    async def get_values_at_index(self, index:int):
        async with self.lock:
            R, t, dt = self[index].R, self[index].t, self[index].dt
        return R, t, dt