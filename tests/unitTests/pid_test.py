import pytest
from src.controllers.pidController import PIDController


def test_pid_controller_basic():
    pid = PIDController(kp=2.0, ki=0.5, kd=0.1, satMin=-100, satMax=100)
    error = 5.0
    dt = 0.1
    output = pid.calculate_control_output(error, dt)

    # Manually compute expected output:
    # P = 2.0 * 5.0 = 10.0
    # I = 0.5 * (5.0 * 0.1) = 0.25
    # D = 0.1 * ((5.0 - 0.0) / 0.1) = 5.0
    expected = 10.0 + 0.25 + 5.0

    assert abs(output - expected) < 1e-6, f"Expected {expected}, got {output}"


def test_pid_controller_saturation_max():
    pid = PIDController(kp=100, ki=20, kd=10, satMin=-10, satMax=10)
    error = 1.0
    dt = 0.1
    output = pid.calculate_control_output(error, dt)

    assert output == 10.0, "Output should be saturated to satMax"


def test_pid_controller_saturation_min():
    pid = PIDController(kp=-100, ki=-20, kd=-10, satMin=-10, satMax=10)
    error = 1.0
    dt = 0.1
    output = pid.calculate_control_output(error, dt)

    assert output == -10.0, "Output should be saturated to satMin"


def test_pid_integral_accumulation():
    pid = PIDController(kp=0, ki=1.0, kd=0, satMin=-100, satMax=100)
    error = 2.0
    dt = 0.5

    out1 = pid.calculate_control_output(error, dt)
    out2 = pid.calculate_control_output(error, dt)

    # Should accumulate integral: I = 2*0.5 + 2*0.5 = 2.0
    assert abs(out2 - 2.0) < 1e-6, f"Expected 2.0, got {out2}"
