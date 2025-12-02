import math

class Aerodynamics:
    def __init__(self, parameters):
        self.cd = float(parameters["cd"])
        self.front_area = float(parameters["front_area"])
        self.air_density = 1.225
        
        self.drag_force = 0 
        
    def update(self, velocity):
        self.drag_force = 0.5*self.air_density*math.pow(velocity,2)*self.cd*self.front_area