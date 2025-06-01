import math


class DCMotor:
    def __init__(self):
        # Motor state
        self.position = 0.0  # θ (rad)
        self.velocity = 0.0  # ω (rad/s)
        self.current = 0.0  # i (A)
        self.input_voltage = 0.0  # V (V)

        # Motor parameters (typical small DC motor)
        self.J = 0.01  # moment of inertia (kg.m^2)
        self.b = 0.1  # damping coefficient (N.m.s)
        self.K = 0.01  # motor torque and back EMF constant (N.m/A and V.s/rad)
        self.R = 1.0  # armature resistance (Ohms)
        self.L = 0.5  # armature inductance (H)

    def set_motor_parameters(self, J, b, K, R, L):
        self.J = J
        self.b = b
        self.K = K
        self.R = R
        self.L = L

    def reset_state(self):
        self.position = 0.0
        self.velocity = 0.0
        self.current = 0.0
        self.input_voltage = 0.0

    def set_input_voltage(self, voltage):
        self.input_voltage = voltage

    def get_position(self):
        return self.position

    def get_velocity(self):
        return self.velocity * 30 / math.pi

    def get_current(self):
        return self.current

    def update_state(self, dt):
        # Electrical dynamics: di/dt = (V - R*i - K*ω) / L
        di_dt = (
            self.input_voltage - self.R * self.current - self.K * self.velocity
        ) / self.L
        self.current += di_dt * dt

        # Mechanical dynamics: dω/dt = (K*i - b*ω) / J
        dw_dt = (self.K * self.current - self.b * self.velocity) / self.J
        self.velocity += dw_dt * dt

        # Integrate position
        self.position += self.velocity * dt
