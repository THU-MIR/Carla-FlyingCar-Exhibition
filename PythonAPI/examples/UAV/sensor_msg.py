import numpy as np
import quaternion

class Point():
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

class Vector():
    def __init__(self, x=0, y=0 ,z=0):
        self.x = x
        self.y = y
        self.z = z

class Quaternion():
    def __init__(self, x=0, y=0, z=0, w=0):
        self.x = x
        self.y = y
        self.z = z
        self.w = w
        # quaternion.from_float_array([self.x, self.y, self.z, self.w])


class Imu():
    def __init__(self):
        self.orientation = Quaternion()
        self.orientation_covariance = [0.] * 9
        self.angular_velocity = Vector()
        self.angular_velocity_covariance = [0.] * 9
        self.linear_acceleration = Vector()
        self.linear_acceleration_covariance = [0.] * 9


class Gps():
    def __init__(self) -> None:
        pass