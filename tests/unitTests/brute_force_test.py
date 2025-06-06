import pytest
import numpy as np
from src.tuners.bruteForce import BruteForcePIDTuner


class MockSystem:
    def __init__(self, return_value=0.0):
        self.output = return_value
        self.reset_called = False

    def __call__(self, control_input):
        return self.output

    def reset_state(self):
        self.reset_called = True


class TestBruteForcePIDTuner:
    def test_brute_force_pid_tuner_init(self):
        mock_system = MockSystem()
        tuner = BruteForcePIDTuner(system=mock_system)
        assert tuner.system == mock_system
        assert tuner.initial_Kp == 0.1
        assert tuner.Kp_step == 0.5
        assert tuner.max_iter == 100
        assert tuner.plant_dt == 0.0001
        assert tuner.controller_dt == 0.001
        assert tuner.satMin == -10.0
        assert tuner.satMax == 10.0
        assert tuner.time_sim == 5.0
        assert tuner.set_point == 1.0

    def test_brute_force_pid_tuner_tune(self):
        mock_system = MockSystem(return_value=0.0)
        tuner = BruteForcePIDTuner(
            system=mock_system,
            initial_Kp=0.1,
            Kp_step=0.1,
            max_iter=3,
            plant_dt=0.1,
            controller_dt=0.1,
            time_sim=0.3,
        )  # Reduced iterations and time for faster testing
        Kp, Ki, Kd = tuner.tune()
        assert isinstance(Kp, float)
        assert isinstance(Ki, float)
        assert isinstance(Kd, float)

    def test_brute_force_pid_tuner_with_reset(self):
        # Mock system with a reset_state method
        mock_system = MockSystem()
        tuner = BruteForcePIDTuner(
            system=mock_system,
            initial_Kp=0.1,
            Kp_step=0.1,
            max_iter=3,
            plant_dt=0.1,
            controller_dt=0.1,
            time_sim=0.3,
        )
        tuner.tune()
        assert mock_system.reset_called

    def test_brute_force_pid_tuner_saturation(self):
        mock_system = MockSystem()
        tuner = BruteForcePIDTuner(
            system=mock_system,
            initial_Kp=100,  # High Kp to cause saturation
            Kp_step=10,
            max_iter=1,
            plant_dt=0.1,
            controller_dt=0.1,
            time_sim=0.1,
            satMin=-1,
            satMax=1,
        )
        tuner.tune()
        # In a real test, you'd assert something about the system's behavior
        # under saturation.  Here, we just check that the tuning runs.
        assert True  # Replace with a meaningful assertion