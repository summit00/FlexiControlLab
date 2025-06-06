import pytest
import numpy as np
from tuners.zieglerNichols import ZieglerNicholsTuner


class TestZieglerNicholsTuner:
    def test_tune_pi_successful(self):
        # Mock system that oscillates predictably
        def mock_system(control_signal):
            # Simulate a simple oscillating system
            return np.sin(control_signal)

        tuner = ZieglerNicholsTuner(system=mock_system, initial_Kp=0.5, Kp_step=0.1, max_iter=50)
        Kp, Ki = tuner.tune_pi()

        assert isinstance(Kp, float)
        assert isinstance(Ki, float)
        assert Kp > 0
        assert Ki > 0

    def test_tune_pi_no_oscillation(self):
        # Mock system that doesn't oscillate
        def mock_system(control_signal):
            return 0.1 * control_signal  # Linear, no oscillation

        tuner = ZieglerNicholsTuner(system=mock_system, initial_Kp=1, Kp_step=0.5, max_iter=10)
        with pytest.raises(ValueError):
            tuner.tune_pi()

    def test_saturation_limits(self):
        # Mock system with saturation
        def mock_system(control_signal):
            return max(-1, min(1, control_signal))  # Saturate between -1 and 1

        tuner = ZieglerNicholsTuner(
            system=mock_system,
            initial_Kp=0.5,
            Kp_step=0.1,
            max_iter=50,
            satMin=-0.5,
            satMax=0.5,
        )
        Kp, Ki = tuner.tune_pi()

        assert isinstance(Kp, float)
        assert isinstance(Ki, float)
        assert Kp > 0
        assert Ki > 0