import pytest
from plants.RL_circuit import RLCircuit

def test_rl_circuit_initialization():
    rl = RLCircuit(R=2.0, L=0.5)
    assert rl.R == 2.0
    assert rl.L == 0.5
    assert rl.current == 0.0
    assert rl.input_voltage == 0.0

def test_set_parameters_and_reset():
    rl = RLCircuit()
    rl.set_parameters(3.0, 0.2)
    assert rl.R == 3.0
    assert rl.L == 0.2
    rl.current = 5.0
    rl.input_voltage = 10.0
    rl.reset_state()
    assert rl.current == 0.0
    assert rl.input_voltage == 0.0

def test_ode_behavior():
    rl = RLCircuit(R=2.0, L=0.5)
    t = 0.0
    x = [0.0]  # initial current
    u = 10.0   # input voltage
    dxdt = rl.ode(t, x, u)
    # di/dt = (u - R*i) / L = (10 - 2*0) / 0.5 = 20.0
    assert pytest.approx(dxdt[0], rel=1e-6) == 20.0

    x = [2.0]
    dxdt = rl.ode(t, x, u)
    # di/dt = (10 - 2*2) / 0.5 = (10 - 4) / 0.5 = 12.0
    assert pytest.approx(dxdt[0], rel=1e-6) == 12.0