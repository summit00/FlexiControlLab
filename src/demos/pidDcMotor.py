import math

from simulation.runSimulation import run_simulation
from plants.dc_motor import DCMotor
from controllers.pidController import PIDController
# from tuners.zieglerNichols import ziegler_nichols_pi_tuning
from tuners.bruteForce import BruteForcePIDTuner


def system_simulator_factory(plant, plant_dt, controller_dt):
    controller_counter = 0
    controller_interval = int(controller_dt / plant_dt)
    mv = 0.0  # Default control signal

    def system(control_input):
        nonlocal controller_counter, mv

        # Update control only at controller interval
        if controller_counter % controller_interval == 0:
            mv = control_input

        controller_counter += 1
        plant.set_manipulated_variable(mv)
        plant.update_state(plant_dt)
        return plant.get_velocity()

    return system


# Adapt DCMotor to use generic method names
class GeneralizedDCMotor(DCMotor):
    def set_manipulated_variable(self, mv):
        self.set_input_voltage(mv)

    def get_output(self):
        return self.get_velocity()

    def reset(self):
        self.reset_state()


if __name__ == "__main__":
    plant = GeneralizedDCMotor()
    plant.set_motor_parameters(J=7.0865e-4, b=5.4177e-6, K=0.0061, R=0.0045, L=1.572e-4)
    satMin = -5.0  # Example saturation limits
    satMax = 5.0

    controller_dt = (plant.L / plant.R) / 10
    plant_dt = controller_dt / 4

    # Create system simulation function
    system = system_simulator_factory(plant, plant_dt, controller_dt)

    # Instantiate the BruteForcePIDTuner
    tuner = BruteForcePIDTuner(
        system=system,
        initial_Kp=0.001,
        Kp_step=0.001,
        max_iter=10,
        plant_dt=plant_dt,
        controller_dt=controller_dt,
        satMin=satMin,
        satMax=satMax,
        time_sim=1,
        set_point=1000.0,
    )

    # Run the tuning process
    Kp_tuned, Ki_tuned, Kd_tuned = tuner.tune()

    # Use tuned parameters in your PID controller
    controller = PIDController(
        kp=Kp_tuned, ki=Ki_tuned, kd=Kd_tuned, satMin=satMin, satMax=satMax
    )
    plant.reset()

    run_simulation(
        set_point=1000.0,
        initial_mv=0.0,
        total_time=1.0,
        plant=plant,
        controller=controller,
        plant_dt=plant_dt,
        controller_dt=controller_dt,
        extra_signals={"Current": plant.get_current},
    )
