import numpy as np


def brute_force_pid_tuning(
    system,
    initial_Kp=0.1,
    Kp_step=0.5,
    max_iter=100,
    plant_dt=0.0001,
    controller_dt=0.001,
    satMin=-10.0,
    satMax=10.0,
    time_sim=5.0,
    set_point=1.0,
):
    """
    Brute-force PID tuner with ITAE cost metric (separate controller and plant time steps).

    Parameters:
    - system: callable object that simulates plant and returns output
    - initial_Kp: starting Kp for sweep
    - Kp_step: step size for Kp
    - max_iter: number of Kp steps
    - plant_dt: plant update timestep
    - controller_dt: controller update timestep
    - satMin, satMax: saturation limits
    - time_sim: total simulation time (s)
    - set_point: reference signal

    Returns:
    - (Kp, Ki, Kd): optimal parameters based on ITAE
    """

    best_cost = float("inf")
    best_params = (0.0, 0.0, 0.0)

    kp_range = [initial_Kp + i * Kp_step for i in range(max_iter)]
    ki_range = np.linspace(0.0, 2.0, 10)
    kd_range = np.linspace(0.0, 0.5, 5)

    total_steps = int(time_sim / plant_dt)
    control_interval = int(controller_dt / plant_dt)

    for Kp in kp_range:
        for Ki in ki_range:
            for Kd in kd_range:
                if hasattr(system, "reset_state"):
                    system.reset_state()

                output = 0.0
                prev_error = 0.0
                integral = 0.0
                cost = 0.0
                mv = 0.0  # start with zero MV

                for i in range(total_steps):
                    t = i * plant_dt

                    if i % control_interval == 0:
                        error = set_point - output
                        integral += error * controller_dt
                        derivative = (
                            (error - prev_error) / controller_dt if i > 0 else 0.0
                        )

                        mv = Kp * error + Ki * integral + Kd * derivative
                        mv = max(satMin, min(satMax, mv))
                        prev_error = error

                    output = system(mv)
                    cost += t * abs(set_point - output)  # ITAE

                if cost < best_cost:
                    best_cost = cost
                    best_params = (Kp, Ki, Kd)
                    print(
                        f"New best → Kp={Kp:.3f}, Ki={Ki:.3f}, Kd={Kd:.3f}, ITAE={cost:.2f}"
                    )

    print(
        f"\nBrute-force tuning complete: Kp={best_params[0]:.3f}, "
        f"Ki={best_params[1]:.3f}, Kd={best_params[2]:.3f}"
    )
    return best_params
