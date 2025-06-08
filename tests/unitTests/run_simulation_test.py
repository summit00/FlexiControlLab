import pytest
from unittest.mock import MagicMock
from simulation.runSimulation import Simulation


@pytest.fixture
def mock_plant():
    """
    Create a mock plant for testing.
    """
    plant = MagicMock()
    plant.get_velocity = MagicMock(return_value=0.0)
    plant.get_current = MagicMock(return_value=0.0)
    plant.update_state = MagicMock()
    plant.set_manipulated_variable = MagicMock()
    return plant


@pytest.fixture
def mock_controller_1():
    """
    Create a mock controller for testing.
    """
    controller = MagicMock()
    controller.calculate_control_output = MagicMock(return_value=1.0)
    return controller


@pytest.fixture
def mock_controller_2():
    """
    Create a second mock controller for cascaded control testing.
    """
    controller = MagicMock()
    controller.calculate_control_output = MagicMock(return_value=2.0)
    return controller


def test_single_controller(mock_plant, mock_controller_1):
    """
    Test the simulation with a single controller.
    """
    # Create simulation instance
    sim = Simulation(
        plant=mock_plant,
        controllers=[mock_controller_1],
        feedback_sources=["get_velocity"],
        sample_times={"plant": 0.01, "controller_0": 0.02},
    )

    # Run simulation
    sim.run(
        set_points=[10.0],  # Desired set point
        initial_mv=0.0,  # Initial manipulated variable
        total_time=0.1,  # Simulation duration
        extra_signals={"Current": mock_plant.get_current},  # Extra signal
    )

    # Assertions
    mock_plant.set_manipulated_variable.assert_called()  # Ensure MV is set
    mock_plant.update_state.assert_called()  # Ensure plant state is updated
    mock_controller_1.calculate_control_output.assert_called()  # Ensure controller is called


def test_multiple_controllers(mock_plant, mock_controller_1, mock_controller_2):
    """
    Test the simulation with multiple controllers (cascaded control).
    """
    # Create simulation instance
    sim = Simulation(
        plant=mock_plant,
        controllers=[mock_controller_1, mock_controller_2],
        feedback_sources=["get_velocity", "get_current"],
        sample_times={"plant": 0.01, "controller_0": 0.02, "controller_1": 0.05},
    )

    # Run simulation
    sim.run(
        set_points=[10.0, 5.0],  # Desired set points for both controllers
        initial_mv=0.0,  # Initial manipulated variable
        total_time=0.1,  # Simulation duration
        extra_signals={"Current": mock_plant.get_current},  # Extra signal
    )

    # Assertions
    mock_plant.set_manipulated_variable.assert_called()  # Ensure MV is set
    mock_plant.update_state.assert_called()  # Ensure plant state is updated
    mock_controller_1.calculate_control_output.assert_called()  # Ensure outer controller is called
    mock_controller_2.calculate_control_output.assert_called()  # Ensure inner controller is called


def test_logging_consistency(mock_plant, mock_controller_1):
    """
    Test that the lengths of logged data (times, feedbacks, MVs) are consistent.
    """
    # Create simulation instance
    sim = Simulation(
        plant=mock_plant,
        controllers=[mock_controller_1],
        feedback_sources=["get_velocity"],
        sample_times={"plant": 0.01, "controller_0": 0.02},
    )

    # Mock the _plot_results method to capture logged data
    sim._plot_results = MagicMock()

    # Run simulation
    sim.run(
        set_points=[10.0],
        initial_mv=0.0,
        total_time=0.1,
        extra_signals={"Current": mock_plant.get_current},
    )

    # Extract logged data from the mocked _plot_results call
    times, feedbacks, mvs, set_points, extra_data = sim._plot_results.call_args[0]

    # Assertions
    assert len(times) == len(mvs)  # Times and MVs should have the same length
    assert len(times) == len(
        feedbacks[0]
    )  # Times and feedbacks should have the same length
    assert len(times) == len(
        extra_data["Current"]
    )  # Times and extra signals should have the same length
