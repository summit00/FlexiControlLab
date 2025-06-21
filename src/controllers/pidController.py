import numpy as np

class PIDController:
    def __init__(self, kp, ki, kd, dt, satMin, satMax):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.satMin = satMin
        self.satMax = satMax
        self.integral = 0.0
        self.prev_error = 0.0

    def update(self, setpoint, feedback):
        error = setpoint - feedback
        self.integral += error * self.dt
        derivative = (error - self.prev_error) / self.dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        return np.clip(output, self.satMin, self.satMax)
