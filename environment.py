import numpy as np

class Sensor:
    def __init__(self, id, x, y, tau):
        self.id = id
        self.x = x
        self.y = y
        self.tau = tau  # Dwell time
        
class Obstacle:
    def __init__(self, id, vertices):
        self.id = id
        self.vertices = vertices  # List of (x,y)

class Environment:
    def __init__(self, base_station_pos, budget, velocity, radius, min_safety_dist):
        self.base_station = base_station_pos
        self.budget = budget
        self.v = velocity
        self.R = radius
        self.d_safe = min_safety_dist
        self.sensors = []
        self.obstacles = []
        
    def add_sensor(self, sensor):
        self.sensors.append(sensor)
        
    def add_obstacle(self, obstacle):
        self.obstacles.append(obstacle)
        
    def get_polar_angle(self, sensor):
        # phi_i = atan2(y_i - y_0, x_i - x_0)
        return np.arctan2(sensor.y - self.base_station[1], sensor.x - self.base_station[0])
        
    def get_radial_distance(self, sensor):
        # Euclidean distance for heuristic
        return np.sqrt((sensor.x - self.base_station[0])**2 + (sensor.y - self.base_station[1])**2)
