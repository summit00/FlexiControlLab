import pytest
from src.controllers.pidController import PIDController


class TestPIDController:
    def test_pid_controller_basic(self):
        pid = PIDController(kp=2.0, ki=0.5, kd=0.1, dt = 0.1, satMin=-100, satMax=100)
        setpoint = 10.0
        measurement = 5.0
        output = pid.update(setpoint, measurement)

        # Manually compute expected output:
        # P = 2.0 * 5.0 = 10.0
        # I = 0.5 * (5.0 * 0.1) = 0.25
        # D = 0.1 * ((5.0 - 0.0) / 0.1) = 5.0
        expected = 10.0 + 0.25 + 5.0

        assert abs(output - expected) < 1e-6, f"Expected {expected}, got {output}"

    def test_pid_controller_saturation_max(self):
        pid = PIDController(kp=100, ki=20, kd=10, dt = 0.1, satMin=-10, satMax=10)
        setpoint = 10.0
        measurement = 5.0
        output = pid.update(setpoint, measurement)

        assert output == 10.0, "Output should be saturated to satMax"

    def test_pid_controller_saturation_min(self):
        pid = PIDController(kp=100, ki=20, kd=10, dt = 0.1, satMin=-10, satMax=10)
        setpoint = -10.0
        measurement = 5.0
        output = pid.update(setpoint, measurement)

        assert output == -10.0, "Output should be saturated to satMin"

    def test_pid_integral_accumulation(self):
        pid = PIDController(kp=0, ki=1.0, kd=0, dt = 0.5, satMin=-100, satMax=100)
        setpoint = 10.0
        measurement = 8.0

        out1 = pid.update(setpoint, measurement)
        out2 = pid.update(setpoint, measurement)

        # Should accumulate integral: I = 2*0.5 + 2*0.5 = 2.0
        assert abs(out2 - 2.0) < 1e-6, f"Expected 2.0, got {out2}"