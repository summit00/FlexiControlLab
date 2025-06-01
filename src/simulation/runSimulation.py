import matplotlib.pyplot as plt


def run_simulation(
    set_point,
    initial_mv,
    total_time,
    plant,
    controller,
    plant_dt=0.0001,
    controller_dt=0.001,
    extra_signals=None,
):
    """
    Runs a simulation with separate time steps for plant and controller.

    Args:
        set_point: Desired reference value.
        initial_mv: Initial manipulated variable.
        total_time: Duration of the simulation.
        plant: Plant object with set/get methods.
        controller: Controller object with calculate_control_output().
        plant_dt: Time step for plant integration.
        controller_dt: Time step for controller updates.
        extra_signals: Optional dict of label: callable returning extra values.
    """
    if extra_signals is None:
        extra_signals = {}

    plt.close("all")

    current_time = 0.0
    plant.set_manipulated_variable(initial_mv)

    # Data logs
    times = []
    outputs = []
    mvs = []
    extra_data = {label: [] for label in extra_signals}

    next_controller_time = 0.0
    mv = initial_mv

    # Simulation loop
    while current_time <= total_time:
        # Controller update at its own time step
        if (
            abs(current_time - next_controller_time) < 1e-10
            or current_time > next_controller_time
        ):
            output = plant.get_output()
            error = set_point - output
            mv = controller.calculate_control_output(error, dt=controller_dt)
            plant.set_manipulated_variable(mv)
            next_controller_time += controller_dt

            # Log only at controller step
            times.append(current_time)
            outputs.append(output)
            mvs.append(mv)
            for label, func in extra_signals.items():
                extra_data[label].append(func())

        # Always update plant at its (faster) time step
        plant.update_state(plant_dt)
        current_time += plant_dt

    # Plotting
    num_plots = 2 + len(extra_signals)
    fig, axs = plt.subplots(num_plots, 1, figsize=(10, 4 * num_plots))

    axs[0].plot(times, outputs, label="Output")
    axs[0].plot(times, [set_point] * len(times), "r--", label="Set Point")
    axs[0].set_ylabel("Output")
    axs[0].set_title("Reference vs Output")
    axs[0].legend()
    axs[0].grid()

    axs[1].plot(times, mvs, label="MV")
    axs[1].set_ylabel("Manipulated Variable")
    axs[1].set_title("Control Signal")
    axs[1].legend()
    axs[1].grid()

    for idx, (label, values) in enumerate(extra_data.items(), start=2):
        axs[idx].plot(times, values, label=label)
        axs[idx].set_ylabel(label)
        axs[idx].set_title(f"{label} Over Time")
        axs[idx].legend()
        axs[idx].grid()

    axs[-1].set_xlabel("Time (s)")
    plt.tight_layout()
    plt.show()
