import pytest
from simulation.runSimulation import Simulation
from plants.dc_motor import DCMotor
from controllers.pidController import PIDController


@pytest.fixture
def motor():
    """
    Create a DC motor instance for simulation.
    """
    motor = DCMotor()
    motor.set_motor_parameters(J=7.0865e-4, b=5.4177e-6, K=0.0061, R=0.0045, L=1.572e-4)
    return motor


@pytest.fixture
def pid_controller():
    """
    Create a PID controller instance for simulation.
    """
    return PIDController(kp=0.005, ki=0.03, kd=0.0002, satMin=-5.0, satMax=5.0)


def test_dc_motor_simulation(motor, pid_controller):
    """
    Test the DC motor in a simulation environment with a PID controller.
    """
    # Simulation parameters
    controller_dt = (motor.L / motor.R) / 10
    plant_dt = controller_dt / 4
    set_point = 100.0  # Desired velocity in RPM
    total_time = 0.5  # Simulation duration in seconds

    # Create simulation instance
    sim = Simulation(
        plant=motor,
        controllers=[pid_controller],
        feedback_sources=["get_velocity"],
        sample_times={"plant": plant_dt, "controller_0": controller_dt},
        plot=False,
    )

    # Run simulation
    sim.run(
        set_points=[set_point],
        initial_mv=0.0,
        total_time=total_time,
        extra_signals={"Current": motor.get_current},
    )

    # Assertions
    # Ensure the motor's velocity approaches the set point
    assert (
        abs(motor.get_velocity() - set_point) < 5.0
    ), "Motor velocity should track the set point within tolerance"

    # Ensure the motor's position increases over time
    assert motor.get_position() > 0.0, "Motor position should increase over time"

    # Ensure the motor's current is non-zero
    assert (
        motor.get_current() > 0.0
    ), "Motor current should be non-zero during operation"
