import subprocess
import re

def get_power_consumption():
    try:
        # Run the command to read power data
        result = subprocess.run(["vcgencmd", "pmic_read_adc"], capture_output=True, text=True)

        # Debug: Print raw output
        #print("Raw Output from vcgencmd:")
        #print(result.stdout)

        # Check if command executed successfully
        if result.returncode != 0 or not result.stdout.strip():
            print("Error: No power data received.")
            return None

        output = result.stdout

        # Extract voltage and current values
        voltage_values = {}
        current_values = {}

        for line in output.split("\n"):
            line = line.strip()  # Remove leading/trailing spaces

            # Match lines with voltage or current values
            match = re.match(r"(\S+)\s+(\w+)\(\d+\)=([\d.]+)", line)
            if match:
                name, value_type, value = match.groups()
                value = float(value)

                if value_type == "volt":
                    voltage_values[name] = value
                elif value_type == "current":
                    current_values[name] = value

        # Compute power consumption per rail
        power_consumption = {}
        total_power = 0.0  

        for rail in voltage_values:
            current_rail = rail.replace("_V", "_A")  # Find matching current rail
            if current_rail in current_values:
                power = voltage_values[rail] * current_values[current_rail]
                power_consumption[rail] = power
                total_power += power

        return power_consumption, total_power

    except Exception as e:
        print(f"Error: {e}")
        return None

# ---- Output Formatting ----
result = get_power_consumption()

if result is not None:
    power_data, total_power = result

    if power_data:
        print("\nPower consumption per rail (watts):")
        for rail, power in power_data.items():
            print(f"{rail}: {power:.3f} W")
        print(f"\nTotal power consumption: {total_power:.3f} W")
else:
    print("Failed to retrieve power data.")
