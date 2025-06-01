import numpy as np


def ziegler_nichols_pi_tuning(
    system,
    initial_Kp=1,
    Kp_step=0.5,
    max_iter=200,
    dt=0.001,
    satMin=-10.0,  # Example saturation limits
    satMax=10.0,
):
    Ku = None
    Tu = None
    Kp = initial_Kp
    time_sim = 20  # Increase time to allow oscillations
    time_steps = int(time_sim / dt)

    for _ in range(max_iter):
        omega_array = np.zeros(time_steps)
        omega = 0

        # Reset plant state here, e.g. plant.reset_state()

        for t in range(time_steps):
            error = 1 - omega
            control_signal = Kp * error  # Only proportional for tuning

            # Apply saturation limits:
            control_signal = max(satMin, min(satMax, control_signal))

            omega = system(control_signal)
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
            peak_times = np.diff(peak_indices) * dt
            if np.all(np.abs(np.diff(peak_times)) < 0.05):
                Ku = Kp
                Tu = np.mean(peak_times) * 2
                break

        Kp += Kp_step

    if Ku is None or Tu is None:
        raise ValueError("Could not find Ku and Tu. Try different tuning parameters.")

    Kp_tuned = 0.45 * Ku
    Ki_tuned = 1.2 * Ku / Tu

    print(
        f"Z-N tuning complete: Ku={Ku:.2f}, Tu={Tu:.2f}, Kp={Kp_tuned:.2f}, Ki={Ki_tuned:.2f}"
    )
    return Kp_tuned, Ki_tuned
