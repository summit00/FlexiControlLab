import numpy as np


def pso_pid_tuning(
    system,
    dt,
    time_sim,
    set_point=1.0,
    satMin=-10.0,
    satMax=10.0,
    num_particles=30,
    max_iter=50,
    kp_bounds=(0, 10),
    ki_bounds=(0, 10),
    kd_bounds=(0, 1),
):
    """
    Particle Swarm Optimization for PID tuning.

    Parameters:
    - system: callable control simulation function (mv) -> output
      Must have system.reset_state() method.
    - dt: simulation time step for system updates
    - time_sim: total simulation time (seconds)
    - set_point: desired reference value
    - satMin, satMax: saturation limits for controller output
    - num_particles: number of particles in swarm
    - max_iter: max iterations
    - kp_bounds, ki_bounds, kd_bounds: parameter search ranges

    Returns:
    - (best_Kp, best_Ki, best_Kd)
    """

    def simulate_pid(Kp, Ki, Kd):
        # Reset plant before simulation
        if hasattr(system, "reset_state"):
            system.reset_state()

        integral = 0.0
        prev_error = 0.0
        output = 0.0
        cost = 0.0

        steps = int(time_sim / dt)
        for i in range(steps):
            t = i * dt
            error = set_point - output
            integral += error * dt
            derivative = (error - prev_error) / dt if i > 0 else 0.0

            mv = Kp * error + Ki * integral + Kd * derivative
            mv = max(satMin, min(satMax, mv))  # Saturation

            output = system(mv)
            cost += t * abs(error)  # ITAE cost

            prev_error = error

        return cost

    # Initialize particles randomly within bounds
    particles = np.zeros((num_particles, 3))  # columns: Kp, Ki, Kd
    velocities = np.zeros_like(particles)
    personal_best_positions = np.zeros_like(particles)
    personal_best_scores = np.full(num_particles, np.inf)

    # Random initial positions
    particles[:, 0] = np.random.uniform(kp_bounds[0], kp_bounds[1], num_particles)
    particles[:, 1] = np.random.uniform(ki_bounds[0], ki_bounds[1], num_particles)
    particles[:, 2] = np.random.uniform(kd_bounds[0], kd_bounds[1], num_particles)

    global_best_position = None
    global_best_score = np.inf

    w = 0.7  # inertia weight
    c1 = 1.5  # cognitive parameter
    c2 = 1.5  # social parameter

    for iteration in range(max_iter):
        for i in range(num_particles):
            cost = simulate_pid(*particles[i])

            if cost < personal_best_scores[i]:
                personal_best_scores[i] = cost
                personal_best_positions[i] = particles[i].copy()

            if cost < global_best_score:
                global_best_score = cost
                global_best_position = particles[i].copy()

        # Update velocities and positions
        r1 = np.random.rand(num_particles, 3)
        r2 = np.random.rand(num_particles, 3)

        velocities = (
            w * velocities
            + c1 * r1 * (personal_best_positions - particles)
            + c2 * r2 * (global_best_position - particles)
        )

        particles += velocities

        # Clamp particles within bounds
        particles[:, 0] = np.clip(particles[:, 0], kp_bounds[0], kp_bounds[1])
        particles[:, 1] = np.clip(particles[:, 1], ki_bounds[0], ki_bounds[1])
        particles[:, 2] = np.clip(particles[:, 2], kd_bounds[0], kd_bounds[1])

        print(
            f"Iteration {iteration+1}/{max_iter}: Best ITAE={global_best_score:.3f} "
            f"at Kp={global_best_position[0]:.4f}, Ki={global_best_position[1]:.4f}, Kd={global_best_position[2]:.4f}"
        )

    print("\nPSO tuning complete:")
    print(
        f"Best parameters → Kp={global_best_position[0]:.4f}, "
        f"Ki={global_best_position[1]:.4f}, Kd={global_best_position[2]:.4f}"
    )

    return tuple(global_best_position)
