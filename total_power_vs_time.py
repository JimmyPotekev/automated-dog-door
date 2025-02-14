import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import time
from power_consumption import get_power_consumption

class PowerMonitor:
    def __init__(self):
        """Initialize power monitoring variables."""
        self.time_values = []
        self.power_values = []
        self.start_time = None
        self.running = False

    def start(self):
        """Start monitoring power consumption."""
        self.start_time = time.time()
        self.running = True

        # Set up matplotlib figure
        self.fig, self.ax = plt.subplots()
        self.line, = self.ax.plot([], [], 'r-', label="Total Power (W)")

        # Set labels
        self.ax.set_title("Total Power Consumption Over Time")
        self.ax.set_xlabel("Time (s)")
        self.ax.set_ylabel("Power (W)")
        self.ax.legend()
        self.ax.set_xlim(0, 60)  # Show last 60 seconds
        self.ax.set_ylim(0, 10)  # Adjust as needed

        # Start real-time animation
        self.ani = animation.FuncAnimation(self.fig, self.update, interval=1000, blit=False)
        plt.show()  # This will keep the plot running

    def update(self, frame):
        """Update function for the real-time power plot."""
        if not self.running:
            return
        
        # Get new power data
        result = get_power_consumption()
        if result is not None:
            _, total_power = result  # Extract total power value

            # Append new data
            elapsed_time = time.time() - self.start_time
            self.time_values.append(elapsed_time)
            self.power_values.append(total_power)

            # Keep only the last 60 seconds on the plot
            if len(self.time_values) > 60:
                self.time_values.pop(0)
                self.power_values.pop(0)

            # Update plot data
            self.line.set_data(self.time_values, self.power_values)
            self.ax.set_xlim(max(0, elapsed_time - 60), elapsed_time)  # Auto-scroll X-axis
            self.ax.set_ylim(min(self.power_values) - 0.5, max(self.power_values) + 0.5)  # Adjust Y-axis

    def stop(self):
        """Stop monitoring and compute average power consumption."""
        self.running = False
        plt.close(self.fig)  # Close the plot window

        if self.power_values:
            avg_power = np.mean(self.power_values)  # Calculate average power using NumPy
            print(f"\nAverage Power Consumption: {avg_power:.3f} W")
            return avg_power
        else:
            print("\nNo power data collected.")
            return None


