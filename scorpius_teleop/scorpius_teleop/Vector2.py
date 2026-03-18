
# Class used as basic 2d vectors to store data and apply basic operations.
# By SimonP-4

import math

class Vector2:
    def __init__(self, x:float = 0, y:float = 0):
        self.x = x
        self.y = y

    def get_magnitude(self) -> float:
        return math.sqrt(self.x * self.x + self.y * self.y)
        
    def normalized(self):
        mag = self.get_magnitude()
        if mag == 0:
            return self
        return Vector2(self.x / mag , self.y / mag)
    
    def get_angle(self) -> float:
        return math.degrees(math.atan2(self.y, self.x))
    
    @staticmethod
    def angle_between(fromVector, toVector) -> float:
        if not isinstance(fromVector, Vector2):
            raise TypeError(f"{fromVector} is not a Vector2.")
        if not isinstance(toVector, Vector2):
            raise TypeError(f"{toVector} is not a Vector2.")
        dot = fromVector.x * toVector.x + fromVector.y * toVector.y
        return math.degrees(math.acos(dot / (fromVector.get_magnitude() * toVector.get_magnitude())))
    