import pytest
import math
from src.plants.dc_motor import DCMotor


def test_dc_motor_initial_state():
    motor = DCMotor()
    assert motor.position == 0.0, "Initial position should be 0.0"
    assert motor.velocity == 0.0, "Initial velocity should be 0.0"
    assert motor.current == 0.0, "Initial current should be 0.0"
    assert motor.input_voltage == 0.0, "Initial input voltage should be 0.0"


def test_dc_motor_set_motor_parameters():
    motor = DCMotor()
    motor.set_motor_parameters(J=0.02, b=0.2, K=0.02, R=2.0, L=1.0)
    assert motor.J == 0.02, "Moment of inertia should be updated"
    assert motor.b == 0.2, "Damping coefficient should be updated"
    assert motor.K == 0.02, "Motor constant should be updated"
    assert motor.R == 2.0, "Resistance should be updated"
    assert motor.L == 1.0, "Inductance should be updated"


def test_dc_motor_reset_state():
    motor = DCMotor()
    motor.set_input_voltage(5.0)
    motor.update_state(0.01)
    motor.reset_state()
    assert motor.position == 0.0, "Position should be reset to 0.0"
    assert motor.velocity == 0.0, "Velocity should be reset to 0.0"
    assert motor.current == 0.0, "Current should be reset to 0.0"
    assert motor.input_voltage == 0.0, "Input voltage should be reset to 0.0"


def test_dc_motor_update_state():
    motor = DCMotor()
    motor.set_input_voltage(5.0)
    motor.update_state(0.01)
    assert motor.current > 0.0, "Current should increase with input voltage"
    assert motor.velocity > 0.0, "Velocity should increase with current"
    assert motor.position > 0.0, "Position should increase with velocity"


def test_dc_motor_get_methods():
    motor = DCMotor()
    motor.set_input_voltage(5.0)
    motor.update_state(0.01)
    assert (
        motor.get_position() == motor.position
    ), "get_position should return the correct position"
    assert motor.get_velocity() == pytest.approx(
        motor.velocity * 30 / math.pi
    ), "get_velocity should return the correct velocity in RPM"
    assert (
        motor.get_current() == motor.current
    ), "get_current should return the correct current"
