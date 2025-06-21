import pytest
from controllers.pidController import PIDController
from plants.RL_circuit import RLCircuit
from tuners.magnitudeOptimum import MagnitudeOptimumTuner
from simulation.simulator import Simulator

@pytest.mark.parametrize("R, L, setpoint, t_final", [
    (1.0, 0.05, 0.1, 0.05),
])
def test_rl_circuit_magnitude_optimum(R, L, setpoint, t_final):
    controller_dt = 0.001
    plant = RLCircuit(R, L)
    current_tuner = MagnitudeOptimumTuner(plant, controller_dt)
    Kp_current, Ki_current, Kd_current = current_tuner.tune()

    controller = PIDController(
        kp=Kp_current,
        ki=Ki_current,  # as in your script
        kd=Kd_current,
        dt=controller_dt,
        satMin=-5.0,
        satMax=5.0
    )

    setpoint_func = lambda t: setpoint
    sim = Simulator(plant, controller, setpoint_func, dt=controller_dt, t_final=t_final)
    results = sim.run()

    assert abs(results["output"][-1] - setpoint) < 0.01  # tolerance can be adjusted