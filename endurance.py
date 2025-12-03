import math
import json
import matplotlib.pyplot as plt
from drivetrain import *
from track import *
from dynamic_loading import *
from aerodynamics import *
from defined_tracks import *

timestep = 0.0001
sim_time = 0

class Accumulator:
    """Battery pack with capacity tracking, thermal modeling, and voltage dynamics"""
    def __init__(self, accu_params, initial_soc_percent=100.0, initial_temp=25.0):
        # Electrical parameters
        self.nominal_voltage = float(accu_params["nominal_voltage"])
        self.max_voltage = float(accu_params["max_voltage"])
        self.min_voltage = float(accu_params["min_voltage"])
        self.min_voltage_ams_fault = float(accu_params["min_voltage_ams_fault"])
        self.total_capacity_wh = float(accu_params["total_capacity_wh"])
        self.total_capacity_ah = self.total_capacity_wh / self.nominal_voltage
        self.max_current = float(accu_params["max_current"])
        self.max_power = float(accu_params["max_power"]) * 1000  # Convert kW to W
        
        # Physical parameters
        self.total_mass = float(accu_params["total_mass"])
        self.num_series = int(accu_params["num_series"])
        self.num_parallel = int(accu_params["num_parallel"])
        self.total_cells = self.num_series * self.num_parallel
        self.internal_resistance = float(accu_params["internal_resistance"])
        
        # Thermal parameters
        self.cell_temp = initial_temp
        self.ambient_temp = 25.0
        self.thermal_mass = self.total_mass * 900  # J/K (specific heat ~900 J/kg/K for Li-ion)
        self.cooling_coefficient = 15.0  # W/K (convective cooling)
        
        # Energy tracking
        self.energy_consumption_multiplier = 1.5
        self.total_capacity_j = self.total_capacity_wh * 3600
        self.usable_capacity_j = self.total_capacity_j
        # Set initial energy_used based on starting SoC
        self.energy_used_j = self.usable_capacity_j * (1.0 - initial_soc_percent / 100.0)
        
        # Real-time states
        self.current_discharge_a = 0
        self.pack_voltage = self.nominal_voltage
        self.voltage_sag = 0
        self.c_rate = 0
        self.power_loss_w = 0
        
    def update(self, power_draw_w, timestep):
        """Update accumulator state with thermal and electrical dynamics"""
        # Calculate initial pack voltage based on SoC
        soc = self.get_soc_percent() / 100.0
        open_circuit_voltage = self.min_voltage + (self.max_voltage - self.min_voltage) * soc
        
        # Calculate desired current (I = P/V)
        desired_current = power_draw_w / open_circuit_voltage if open_circuit_voltage > 0 else 0
        
        # HARD LIMIT: Enforce max current during discharge
        if desired_current > self.max_current:
            actual_current = self.max_current
            actual_power = actual_current * open_circuit_voltage
        else:
            actual_current = desired_current
            actual_power = power_draw_w
        
        # For regenerative braking (negative power), check voltage doesn't exceed max
        if power_draw_w < 0:  # Regen/charging
            regen_voltage = open_circuit_voltage - (desired_current * self.internal_resistance)
            if regen_voltage > self.max_voltage:
                # Limit regen current to prevent overvoltage
                max_regen_current = (self.max_voltage - open_circuit_voltage) / self.internal_resistance
                actual_current = max(max_regen_current, desired_current)  # max_regen_current is negative
                actual_power = actual_current * open_circuit_voltage
        
        # Energy consumption with multiplier (only for discharge, not regen)
        if actual_power > 0:
            energy_delta_j = actual_power * timestep * self.energy_consumption_multiplier
        else:
            energy_delta_j = actual_power * timestep  # Regen doesn't get multiplier
        self.energy_used_j += energy_delta_j
        
        # Store actual current
        self.current_discharge_a = actual_current
        
        # C-rate calculation (current relative to capacity)
        self.c_rate = self.current_discharge_a / self.total_capacity_ah if self.total_capacity_ah > 0 else 0
        
        # Voltage sag due to internal resistance (V_sag = I * R)
        self.voltage_sag = abs(self.current_discharge_a) * self.internal_resistance
        
        # Pack voltage accounting for SoC and voltage sag
        if actual_current >= 0:  # Discharge
            self.pack_voltage = open_circuit_voltage - self.voltage_sag
        else:  # Regen/charge
            self.pack_voltage = open_circuit_voltage + self.voltage_sag
        
        # Enforce hard voltage limits
        self.pack_voltage = max(min(self.pack_voltage, self.max_voltage), self.min_voltage)
        
        # Power loss due to internal resistance (P_loss = I²R)
        self.power_loss_w = (self.current_discharge_a ** 2) * self.internal_resistance
        
        # Thermal model: dT/dt = (P_loss - cooling) / thermal_mass
        cooling_power = self.cooling_coefficient * (self.cell_temp - self.ambient_temp)
        temp_rate = (self.power_loss_w - cooling_power) / self.thermal_mass
        self.cell_temp += temp_rate * timestep
        
    def get_soc_percent(self):
        """State of Charge as percentage"""
        return ((self.usable_capacity_j - self.energy_used_j) / self.usable_capacity_j) * 100
    
    def get_energy_remaining_kwh(self):
        """Energy remaining in kWh"""
        return (self.usable_capacity_j - self.energy_used_j) / 3600000
            
class Kinematics:
    def __init__(self, mass):
        self.mass = mass
        self.acceleration = 0
        self.velocity = 0
        
    def update(self, longitudinal_force):
        self.acceleration = longitudinal_force / self.mass
        self.velocity += self.acceleration * timestep
        
    def braking_distance(self, target_speed, braking_force):
        distance = (pow(target_speed,2)-pow(self.velocity,2))/(2*(-braking_force/self.mass))
        return distance
       
class Vehicle:
    def __init__(self, vehicle_parameters):
        self.mass = float(vehicle_parameters["mass"])
        
        self.kinematics = Kinematics(self.mass)
        self.dynamic_loading = DynamicLoading(self.mass, vehicle_parameters["dynamic_loading"])
        self.aerodynamics = Aerodynamics(vehicle_parameters["aerodynamics"])
        
        self.FL_drivetrain = Drivetrain(vehicle_parameters["FL_drivetrain"])
        self.FR_drivetrain = Drivetrain(vehicle_parameters["FR_drivetrain"])
        self.RL_drivetrain = Drivetrain(vehicle_parameters["RL_drivetrain"])
        self.RR_drivetrain = Drivetrain(vehicle_parameters["RR_drivetrain"])
        
        self.vehicle_tractive_force = 0
        self.longitudinal_force = 0
        self.producable_grip_force = 1
        self.drag_force = 0
        
        self.update(0)
        
    def update(self, drive):
        self.dynamic_loading.update(self.longitudinal_force)
        self.aerodynamics.update(self.kinematics.velocity)
        self.FL_drivetrain.update(self.kinematics.velocity, self.dynamic_loading.FL_z, drive)
        self.FR_drivetrain.update(self.kinematics.velocity, self.dynamic_loading.FR_z, drive)
        self.RL_drivetrain.update(self.kinematics.velocity, self.dynamic_loading.RL_z, drive)
        self.RR_drivetrain.update(self.kinematics.velocity, self.dynamic_loading.RR_z, drive)

        self.vehicle_tractive_force = self.FL_drivetrain.tyre.tractive_force + self.FR_drivetrain.tyre.tractive_force + self.RL_drivetrain.tyre.tractive_force + self.RR_drivetrain.tyre.tractive_force
        self.longitudinal_force = self.vehicle_tractive_force - self.aerodynamics.drag_force

        self.producable_grip_force = self.FL_drivetrain.tyre.producable_grip_force + self.FR_drivetrain.tyre.producable_grip_force + self.RL_drivetrain.tyre.producable_grip_force + self.RR_drivetrain.tyre.producable_grip_force

        self.kinematics.update(self.longitudinal_force)
    
    def get_total_power(self):
        """Get total instantaneous power draw from all motors"""
        return (self.RL_drivetrain.motor.power + self.RR_drivetrain.motor.power + 
                self.FL_drivetrain.motor.power + self.FR_drivetrain.motor.power)

# Load Vehicle Configuration
vehicle_json_file = "vehicles/"+ input('Vehicle JSON file: ')
try:
    with open(vehicle_json_file, "r") as read_file:
        vehicle_parameters = json.load(read_file)
except FileNotFoundError:
    print(f"ERROR: Vehicle file '{vehicle_json_file}' not found!")
    exit(1)
except json.JSONDecodeError as e:
    print(f"ERROR: Invalid JSON in vehicle file: {e}")
    exit(1)
    
vehicle = Vehicle(vehicle_parameters)

# Load Accumulator Configuration
accumulator_json_file = "accumulators/"+ input('Accumulator JSON file: ')
try:
    with open(accumulator_json_file, "r") as read_file:
        content = read_file.read()
        if not content.strip():
            print(f"ERROR: Accumulator file '{accumulator_json_file}' is empty!")
            exit(1)
        accu_params = json.loads(content)
except FileNotFoundError:
    print(f"ERROR: Accumulator file '{accumulator_json_file}' not found!")
    print("Make sure you have created the 'accumulators/' folder and added your JSON file.")
    exit(1)
except json.JSONDecodeError as e:
    print(f"ERROR: Invalid JSON in accumulator file: {e}")
    print("Check that your JSON file has proper formatting (quotes, commas, braces)")
    exit(1)

# ...existing code...

accumulator = Accumulator(accu_params)
track = Track(defined_tracks.calder_autox, vehicle)

# Ask for number of laps
num_laps = int(input('Number of laps to simulate: '))

# Multi-lap tracking
all_lap_data = []
cumulative_time = 0

print(f"\n=== Starting Endurance Simulation ({num_laps} laps) ===")
print(f"Vehicle: {vehicle_parameters['name']}")
print(f"Accumulator: {accumulator.total_capacity_wh}Wh @ {accumulator.nominal_voltage}V")
print(f"Configuration: {accumulator.num_series}S{accumulator.num_parallel}P ({accumulator.total_cells} cells)")
print(f"Internal Resistance: {accumulator.internal_resistance}Ω\n")

for lap_num in range(1, num_laps + 1):
    # Reset per-lap variables (NOT the accumulator)
    sim_time = 0
    x = []
    velocity = []
    available_power = []
    actual_power = []
    soc_percent = []
    discharge_current = []
    energy_remaining_kwh = []
    pack_voltage = []
    voltage_sag = []
    regen_power_track = []
    total_power_draw = []
    power_loss = []
    energy_consumed = 0
    energy_regenerated = 0
    
    print(f"--- Lap {lap_num} ---")
    print(f"Starting SoC: {accumulator.get_soc_percent():.1f}%")
    print(f"Starting Temp: {accumulator.cell_temp:.1f}°C")
    
    # Reset ONLY track and vehicle for new lap (keep accumulator state)
    track.reset()
    vehicle.kinematics.velocity = 0
    vehicle.kinematics.acceleration = 0
    
    # Sim loop for this lap
    while track.is_driving():
        vehicle.update(track.drive(vehicle, timestep))
        sim_time += timestep
        
        # Power and energy calculations
        total_power = vehicle.get_total_power()
        energy_consumed += total_power * timestep
        
        regen_power = (vehicle.RL_drivetrain.regen_power + vehicle.RR_drivetrain.regen_power + 
                       vehicle.FL_drivetrain.regen_power + vehicle.FR_drivetrain.regen_power)
        energy_regenerated += regen_power * timestep
        
        # Update accumulator (persistent across laps)
        net_power = total_power - regen_power
        accumulator.update(net_power, timestep)
        
        # Data logging
        x.append(cumulative_time + sim_time)
        velocity.append(vehicle.kinematics.velocity)
        
        # Power data
        max_available_power = (float(vehicle_parameters["FL_drivetrain"]["power_limit"]) + 
                              float(vehicle_parameters["FR_drivetrain"]["power_limit"]) + 
                              float(vehicle_parameters["RL_drivetrain"]["power_limit"]) + 
                              float(vehicle_parameters["RR_drivetrain"]["power_limit"]))
        available_power.append(max_available_power / 1000)
        actual_power.append(total_power / 1000)
        
        # Accumulator data
        soc_percent.append(accumulator.get_soc_percent())
        discharge_current.append(accumulator.current_discharge_a)
        energy_remaining_kwh.append(accumulator.get_energy_remaining_kwh())
        pack_voltage.append(accumulator.pack_voltage)
        voltage_sag.append(accumulator.voltage_sag)
        regen_power_track.append(regen_power / 1000)
        total_power_draw.append(total_power / 1000)
        power_loss.append(accumulator.power_loss_w / 1000)
    
    # Lap results
    lap_energy_used = (energy_consumed - energy_regenerated) / 3600000
    lap_soc_final = accumulator.get_soc_percent()
    lap_temp_final = accumulator.cell_temp
    lap_min_voltage = min(pack_voltage)
    
    print(f"Lap Time: {sim_time:.2f}s")
    print(f"Energy Used: {lap_energy_used:.2f}kWh")
    print(f"Final SoC: {lap_soc_final:.1f}%")
    print(f"Final Temp: {lap_temp_final:.1f}°C")
    print(f"Min Pack Voltage: {lap_min_voltage:.1f}V\n")
    
    # Store lap data
    all_lap_data.append({
        'lap': lap_num,
        'x': x,
        'velocity': velocity,
        'soc': soc_percent,
        'pack_voltage': pack_voltage,
        'current': discharge_current,
        'energy_remaining': energy_remaining_kwh,
        'energy_used_kwh': lap_energy_used,
        'final_soc': lap_soc_final,
        'final_temp': lap_temp_final,
        'min_voltage': lap_min_voltage
    })
    
    cumulative_time += sim_time
    
    # Check if battery is critically low
    if accumulator.get_soc_percent() < 5:
        print(f"WARNING: Battery critically low at {accumulator.get_soc_percent():.1f}% - stopping simulation")
        break

# ...existing code...

# Print summary table of all laps
print(f"\n=== Lap Summary ===")
print(f"{'Lap':<5} {'Energy (kWh)':<15} {'Final SoC (%)':<15} {'Final Temp (°C)':<15} {'Min Voltage (V)':<15}")
print("-" * 65)
for lap_data in all_lap_data:
    print(f"{lap_data['lap']:<5} {lap_data['energy_used_kwh']:<15.2f} {lap_data['final_soc']:<15.1f} {lap_data['final_temp']:<15.1f} {lap_data['min_voltage']:<15.1f}")

# Print summary table of all laps
print(f"\n=== Lap Summary ===")
print(f"{'Lap':<5} {'Energy (kWh)':<15} {'Final SoC (%)':<15} {'Final Temp (°C)':<15} {'Min Voltage (V)':<15}")
print("-" * 65)
for lap_data in all_lap_data:
    print(f"{lap_data['lap']:<5} {lap_data['energy_used_kwh']:<15.2f} {lap_data['final_soc']:<15.1f} {lap_data['final_temp']:<15.1f} {lap_data['min_voltage']:<15.1f}")

# Plotting all laps
fig, axes = plt.subplots(2, 2, figsize=(16, 10))

for lap_data in all_lap_data:
    lap_num = lap_data['lap']
    axes[0, 0].plot(lap_data['x'], lap_data['velocity'], label=f'Lap {lap_num}', linewidth=0.8)
    axes[0, 1].plot(lap_data['x'], lap_data['soc'], label=f'Lap {lap_num}', linewidth=0.8)
    axes[1, 0].plot(lap_data['x'], lap_data['pack_voltage'], label=f'Lap {lap_num}', linewidth=0.8)
    axes[1, 1].plot(lap_data['x'], lap_data['energy_remaining'], label=f'Lap {lap_num}', linewidth=0.8)

axes[0, 0].set_title('Velocity Profile')
axes[0, 0].set_xlabel('Time (s)')
axes[0, 0].set_ylabel('Velocity (m/s)')
axes[0, 0].grid(True, alpha=0.3)
axes[0, 0].legend()

axes[0, 1].set_title('State of Charge')
axes[0, 1].set_xlabel('Time (s)')
axes[0, 1].set_ylabel('SoC (%)')
axes[0, 1].axhline(y=0, color='k', linestyle='--', alpha=0.3)
axes[0, 1].axhline(y=100, color='g', linestyle='--', alpha=0.3)
axes[0, 1].grid(True, alpha=0.3)
axes[0, 1].legend()

axes[1, 0].set_title('Pack Voltage')
axes[1, 0].set_xlabel('Time (s)')
axes[1, 0].set_ylabel('Voltage (V)')
axes[1, 0].axhline(y=accumulator.min_voltage_ams_fault, color='r', linestyle='--', alpha=0.3, label='AMS Fault')
axes[1, 0].grid(True, alpha=0.3)
axes[1, 0].legend()

axes[1, 1].set_title('Energy Remaining')
axes[1, 1].set_xlabel('Time (s)')
axes[1, 1].set_ylabel('Energy (kWh)')
axes[1, 1].grid(True, alpha=0.3)
axes[1, 1].legend()

plt.tight_layout()
plt.show()