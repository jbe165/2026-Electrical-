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
    def __init__(self, accu_params):
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
        self.cell_temp = 25.0  # Start at 25°C ambient
        self.ambient_temp = 25.0
        self.thermal_mass = self.total_mass * 900  # J/K (specific heat ~900 J/kg/K for Li-ion)
        self.cooling_coefficient = 15.0  # W/K (convective cooling)
        
        # Energy tracking
        self.energy_consumption_multiplier = 1.5
        self.total_capacity_j = self.total_capacity_wh * 3600
        self.usable_capacity_j = self.total_capacity_j
        self.energy_used_j = 0
        
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

accumulator = Accumulator(accu_params)
track = Track(defined_tracks.calder_autox, vehicle)

# Plotting arrays
x = []
velocity = []
available_power = []
actual_power = []

# Accumulator tracking arrays
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

# Sim Loop
print(f"\n=== Starting Lap Simulation ===")
print(f"Vehicle: {vehicle_parameters['name']}")
print(f"Accumulator: {accumulator.total_capacity_wh}Wh @ {accumulator.nominal_voltage}V")
print(f"Configuration: {accumulator.num_series}S{accumulator.num_parallel}P ({accumulator.total_cells} cells)")
print(f"Internal Resistance: {accumulator.internal_resistance}Ω")
print(f"Max Current: {accumulator.max_current}A, Max Power: {accumulator.max_power/1000}kW")
print(f"Energy consumption multiplier: 1.5x (conservative estimate)\n")

while(track.is_driving()):
    vehicle.update(track.drive(vehicle, timestep))

    sim_time += timestep
    
    # Power and energy calculations
    total_power = vehicle.get_total_power()
    energy_consumed += total_power * timestep  # joules
    
    regen_power = (vehicle.RL_drivetrain.regen_power + vehicle.RR_drivetrain.regen_power + 
                   vehicle.FL_drivetrain.regen_power + vehicle.FR_drivetrain.regen_power)
    energy_regenerated += regen_power * timestep  # joules
    
    # Update accumulator
    net_power = total_power - regen_power
    accumulator.update(net_power, timestep)
    
    # Data logging
    x.append(sim_time)
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
    regen_power_track.append(regen_power / 1000)  # Convert to kW
    total_power_draw.append(total_power / 1000)
    power_loss.append(accumulator.power_loss_w / 1000)  # Convert to kW

# Results
print("\n=== Lap Complete ===")
print(f"Lap Time: {sim_time:.2f}s")
print(f"\nEnergy Consumed: {energy_consumed/1000000:.2f}MJ ({energy_consumed/3600000:.2f}kWh)")
print(f"Energy Regenerated: {energy_regenerated/1000000:.2f}MJ ({energy_regenerated/3600000:.2f}kWh)")
print(f"Net Energy Used: {(energy_consumed-energy_regenerated)/3600000:.2f}kWh")
print(f"\nFinal State of Charge: {accumulator.get_soc_percent():.1f}%")
print(f"Energy Remaining: {accumulator.get_energy_remaining_kwh():.2f}kWh")
print(f"Peak Discharge Current: {max(discharge_current):.1f}A")
print(f"Max Voltage Sag: {max(voltage_sag):.1f}V")
print(f"Min Pack Voltage: {min(pack_voltage):.1f}V")
print(f"Peak Regen Power: {max(regen_power_track):.2f}kW")

# Plotting
fig1 = plt.figure(figsize=(16, 12))

# Velocity
plt.subplot(4, 2, 1)
plt.title(f'{vehicle_parameters["name"]} - Velocity Profile')
plt.xlabel("Time (s)")
plt.ylabel("Velocity (m/s)")
plt.plot(x, velocity, '-b', linewidth=0.8)
plt.grid(True, alpha=0.3)

# State of Charge
plt.subplot(4, 2, 2)
plt.title('Accumulator State of Charge')
plt.xlabel("Time (s)")
plt.ylabel("SoC (%)")
plt.plot(x, soc_percent, '-r', linewidth=1.2)
plt.axhline(y=0, color='k', linestyle='--', alpha=0.5, label='Empty')
plt.axhline(y=100, color='g', linestyle='--', alpha=0.5, label='Full')
plt.grid(True, alpha=0.3)
plt.legend()

# Pack Voltage
plt.subplot(4, 2, 3)
plt.title('Pack Voltage')
plt.xlabel("Time (s)")
plt.ylabel("Voltage (V)")
plt.plot(x, pack_voltage, color='blue', linewidth=1.0)
plt.axhline(y=accumulator.min_voltage_ams_fault, color='r', linestyle='--', alpha=0.5, label='AMS Fault')
plt.axhline(y=accumulator.nominal_voltage, color='g', linestyle='--', alpha=0.5, label='Nominal')
plt.grid(True, alpha=0.3)
plt.legend()

# Voltage Sag
plt.subplot(4, 2, 4)
plt.title('Voltage Sag (I×R)')
plt.xlabel("Time (s)")
plt.ylabel("Voltage Drop (V)")
plt.plot(x, voltage_sag, color='orange', linewidth=0.8)
plt.grid(True, alpha=0.3)

# Discharge Current
plt.subplot(4, 2, 5)
plt.title('Discharge Current')
plt.xlabel("Time (s)")
plt.ylabel("Current (A)")
plt.plot(x, discharge_current, color='purple', linewidth=0.8)
plt.axhline(y=accumulator.max_current, color='r', linestyle='--', alpha=0.5, label='Max Current')
plt.grid(True, alpha=0.3)
plt.legend()

# Energy Remaining
plt.subplot(4, 2, 6)
plt.title('Energy Remaining')
plt.xlabel("Time (s)")
plt.ylabel("Energy (kWh)")
plt.plot(x, energy_remaining_kwh, color='magenta', linewidth=1.2)
plt.grid(True, alpha=0.3)

# Regenerative Braking Power
plt.subplot(4, 2, 7)
plt.title('Regenerative Braking Power')
plt.xlabel("Time (s)")
plt.ylabel("Regen Power (kW)")
plt.plot(x, regen_power_track, color='green', linewidth=0.8)
plt.fill_between(x, 0, regen_power_track, color='green', alpha=0.3)
plt.grid(True, alpha=0.3)

# Power: Available vs Actual
plt.subplot(4, 2, 8)
plt.title('Power: Available vs Actual Usage')
plt.xlabel("Time (s)")
plt.ylabel("Power (kW)")
plt.plot(x, available_power, '--', color='gray', linewidth=1.0, label='Available', alpha=0.7)
plt.plot(x, actual_power, '-', color='blue', linewidth=0.8, label='Actual Used')
plt.grid(True, alpha=0.3)
plt.legend()

plt.tight_layout()
plt.show()