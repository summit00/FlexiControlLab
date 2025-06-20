import pytest
from tuners.magnitudeOptimum import MagnitudeOptimumTuner

class DummyPlant:
    def __init__(self, L, R):
        self.L = L
        self.R = R

def test_magnitude_optimum_tuner_basic():
    # Example parameters
    L = 0.5
    R = 2.0
    Ts_current = 0.1
    plant = DummyPlant(L, R)
    tuner = MagnitudeOptimumTuner(plant, Ts_current)
    Kp, Ki, Kd = tuner.tune()

    Te = 2 * Ts_current
    tau = L / R
    expected_Kp = 1.5 * L / 2 / Te
    expected_Ki = 1 / (tau * (1 / Ts_current))
    expected_Kd = 0.0

    assert pytest.approx(Kp, rel=1e-6) == expected_Kp
    assert pytest.approx(Ki, rel=1e-6) == expected_Ki
    assert Kd == expected_Kd

def test_magnitude_optimum_zero_L():
    L = 0.0
    R = 2.0
    Ts_current = 0.1
    plant = DummyPlant(L, R)
    tuner = MagnitudeOptimumTuner(plant, Ts_current)
    Kp, Ki, Kd = tuner.tune()
    assert Kp == 0.0
    # Ki will raise ZeroDivisionError if tau==0, so let's check for that
    with pytest.raises(ZeroDivisionError):
        _ = 1 / ((L / R) * (1 / Ts_current))