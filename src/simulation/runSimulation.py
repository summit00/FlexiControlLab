import matplotlib.pyplot as plt


class Simulation:
    def __init__(self, plant, controllers, feedback_sources, sample_times):
        """
        Initialize the simulation environment.

        Args:
            plant: Plant object with set/get methods.
            controllers: List of controller objects (outer to inner for cascaded control).
            feedback_sources: List of feedback sources (e.g., "get_output", "get_velocity").
            sample_times: Dictionary with sample times for 'plant' and each controller.
        """
        self.plant = plant
        self.controllers = controllers
        self.feedback_sources = feedback_sources
        self.sample_times = sample_times
        self.next_sample_times = {key: 0.0 for key in sample_times}

    def run(self, set_points, initial_mv, total_time, extra_signals=None):
        """
        Run the simulation.

        Args:
            set_points: List of desired reference values (outer to inner for cascaded control).
            initial_mv: Initial manipulated variable for the innermost loop.
            total_time: Duration of the simulation.
            extra_signals: Optional dict of label: callable returning extra values.

        Returns:
            None
        """
        if extra_signals is None:
            extra_signals = {}

        plt.close("all")

        current_time = 0.0
        self.plant.set_manipulated_variable(initial_mv)

        # Data logs
        times = []
        feedbacks = [[] for _ in self.controllers]
        mvs = []
        extra_data = {label: [] for label in extra_signals}

        mv = initial_mv

        # Simulation loop
        while current_time <= total_time:
            # Update controllers in cascaded order
            for i, controller in enumerate(self.controllers):
                if current_time >= self.next_sample_times[f"controller_{i}"]:
                    # Get feedback from the specified source
                    if callable(self.feedback_sources[i]):
                        feedback = self.feedback_sources[i](self.plant)
                    else:
                        feedback = getattr(self.plant, self.feedback_sources[i])()

                    error = set_points[i] - feedback
                    mv = controller.calculate_control_output(error)
                    self.next_sample_times[f"controller_{i}"] += self.sample_times[
                        f"controller_{i}"
                    ]

                    # Log controller outputs
                    feedbacks[i].append(feedback)
                else:
                    # Append the last feedback value to maintain consistent length
                    feedbacks[i].append(feedbacks[i][-1] if feedbacks[i] else 0.0)

            # Update plant state
            if current_time >= self.next_sample_times["plant"]:
                self.plant.set_manipulated_variable(mv)
                self.plant.update_state(self.sample_times["plant"])
                self.next_sample_times["plant"] += self.sample_times["plant"]

            # Log plant data
            times.append(current_time)
            mvs.append(mv)
            for label, func in extra_signals.items():
                extra_data[label].append(func())

            # Increment time
            current_time += min(self.sample_times.values())

        # Plotting
        self._plot_results(times, feedbacks, mvs, set_points, extra_data)

    def _plot_results(self, times, feedbacks, mvs, set_points, extra_data):
        """
        Helper function to plot simulation results.
        """
        num_plots = len(feedbacks) + 1 + len(extra_data)
        fig, axs = plt.subplots(num_plots, 1, figsize=(10, 4 * num_plots))

        for i, feedback in enumerate(feedbacks):
            axs[i].plot(times, feedback, label=f"Feedback {i + 1}")
            axs[i].plot(
                times, [set_points[i]] * len(times), "r--", label=f"Set Point {i + 1}"
            )
            axs[i].set_ylabel(f"Feedback {i + 1}")
            axs[i].set_title(f"Controller {i + 1} Feedback")
            axs[i].legend()
            axs[i].grid()

        axs[len(feedbacks)].plot(times, mvs, label="Manipulated Variable (MV)")
        axs[len(feedbacks)].set_ylabel("MV")
        axs[len(feedbacks)].set_title("Control Signal")
        axs[len(feedbacks)].legend()
        axs[len(feedbacks)].grid()

        for idx, (label, values) in enumerate(
            extra_data.items(), start=len(feedbacks) + 1
        ):
            axs[idx].plot(times, values, label=label)
            axs[idx].set_ylabel(label)
            axs[idx].set_title(f"{label} Over Time")
            axs[idx].legend()
            axs[idx].grid()

        axs[-1].set_xlabel("Time (s)")
        plt.tight_layout()
        plt.show()
