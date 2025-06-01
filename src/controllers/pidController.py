class PIDController:
    def __init__(self, kp, ki, kd, satMin, satMax):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.satMin = satMin
        self.satMax = satMax
        self.integral = 0.0
        self.prev_error = 0.0

    def calculate_control_output(self, error, dt=0.01):
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        if output < self.satMin:
            output = self.satMin
        elif output > self.satMax:
            output = self.satMax
        self.prev_error = error
        return output
