import numpy as np


class ZieglerNicholsTuner:
    """
    Implements Ziegler-Nichols tuning for PI controllers.

    This class helps determine suitable Kp and Ki values for a PI controller
    based on the ultimate gain (Ku) and ultimate period (Tu) of the system,
    found through experimentation.
    """

    def __init__(
        self,
        system,
        initial_Kp=1,
        Kp_step=0.5,
        max_iter=200,
        dt=0.001,
        satMin=-10.0,  # Example saturation limits
        satMax=10.0,
    ):
        """
        Initializes the ZieglerNicholsTuner.

        Args:
            system: The system to be controlled.  This should be a callable
                    that accepts a control signal and returns the system output.
            initial_Kp (float): Initial proportional gain for the tuning process.
            Kp_step (float): Step size for increasing Kp during the tuning process.
            max_iter (int): Maximum number of iterations to find Ku.
            dt (float): Time step for the simulation.
            satMin (float): Minimum saturation limit for the control signal.
            satMax (float): Maximum saturation limit for the control signal.
        """
        self.system = system
        self.initial_Kp = initial_Kp
        self.Kp_step = Kp_step
        self.max_iter = max_iter
        self.dt = dt
        self.satMin = satMin
        self.satMax = satMax

    def tune_pi(self):
        """
        Tunes a PI controller using the Ziegler-Nichols method.

        This method iteratively increases the proportional gain (Kp) until
        sustained oscillations are observed.  The ultimate gain (Ku) and
        ultimate period (Tu) are then used to calculate the PI controller
        parameters.

        Returns:
            tuple: A tuple containing the tuned Kp and Ki values.

        Raises:
            ValueError: If Ku and Tu cannot be found within the maximum number
                        of iterations.
        """
        Ku = None
        Tu = None
        Kp = self.initial_Kp
        time_sim = 20  # Increase time to allow oscillations
        time_steps = int(time_sim / self.dt)

        for _ in range(self.max_iter):
            omega_array = np.zeros(time_steps)
            omega = 0

            # Reset plant state here, e.g. plant.reset_state()

            for t in range(time_steps):
                error = 1 - omega
                control_signal = Kp * error  # Only proportional for tuning

                # Apply saturation limits:
                control_signal = max(self.satMin, min(self.satMax, control_signal))

                omega = self.system(control_signal)
                omega_array[t] = omega

            # Peak detection with relaxed tolerance
            peak_indices = (
                np.where(
                    (omega_array[:-2] < omega_array[1:-1])
                    & (omega_array[1:-1] > omega_array[2:])
                )[0]
                + 1
            )

            if len(peak_indices) >= 5:
                peak_times = np.diff(peak_indices) * self.dt
                if np.all(np.abs(np.diff(peak_times)) < 0.05):
                    Ku = Kp
                    Tu = np.mean(peak_times) * 2
                    break

            Kp += self.Kp_step

        if Ku is None or Tu is None:
            raise ValueError("Could not find Ku and Tu. Try different tuning parameters.")

        Kp_tuned = 0.45 * Ku
        Ki_tuned = 1.2 * Ku / Tu

        print(
            f"Z-N tuning complete: Ku={Ku:.2f}, Tu={Tu:.2f}, Kp={Kp_tuned:.2f}, Ki={Ki_tuned:.2f}"
        )
        return Kp_tuned, Ki_tuned


def ziegler_nichols_pi_tuning(
    system,
    initial_Kp=1,
    Kp_step=0.5,
    max_iter=200,
    dt=0.001,
    satMin=-10.0,  # Example saturation limits
    satMax=10.0,
):
    """
    Tunes a PI controller using the Ziegler-Nichols method.

    This function serves as a wrapper for the ZieglerNicholsTuner class.

    Args:
        system: The system to be controlled.  This should be a callable
                that accepts a control signal and returns the system output.
        initial_Kp (float): Initial proportional gain for the tuning process.
        Kp_step (float): Step size for increasing Kp during the tuning process.
        max_iter (int): Maximum number of iterations to find Ku.
        dt (float): Time step for the simulation.
        satMin (float): Minimum saturation limit for the control signal.
        satMax (float): Maximum saturation limit for the control signal.

    Returns:
        tuple: A tuple containing the tuned Kp and Ki values.

    Raises:
        ValueError: If Ku and Tu cannot be found within the maximum number
                    of iterations.
    """
    tuner = ZieglerNicholsTuner(system, initial_Kp, Kp_step, max_iter, dt, satMin, satMax)
    return tuner.tune_pi()