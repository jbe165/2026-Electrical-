import math

class TyreClass:
    def __init__(self):
        # Constants
        self.radius = 0.2032 #meters
        self.friction_coefficient = 1.5
        
        # Set by vehicle dynamics
        self.fz = 0 
        
        # Updated acording to fz
        self.producable_grip_force = 0
        self.produceable_grip_torque = 0
        
        # Set according to vehicle speed (assume tyres never slip)
        self.radial_velocity = 0
        
        # Set according to motor torque through gearbox
        self.torque = 0
        self.tractive_force = 0
        
    def update(self, vehicle_velocity = 0, fz = 0, drive_torque = 0, brake_torque = 0):        
        # Set input values
        self.radial_velocity = vehicle_velocity / self.radius
        self.fz = fz
        self.torque = drive_torque + brake_torque
        
        # Calculate grip limit
        self.producable_grip_force = self.fz * self.friction_coefficient
        self.produceable_grip_torque = self.producable_grip_force * self.radius
        
        # Set tractive force used by simulation kinematics to accelerate the car
        self.tractive_force = self.torque / self.radius

class MotorClass:
    def __init__(self, power_limit):
        # Constants
        self.max_rpm = 20000
        self.max_radial_velocity = (self.max_rpm / 60) * 2 * math.pi
        self.power_limit = power_limit 
        #self.torque_limit = 21 #According to AMK datasheet
        self.torque_limit = 31.6 # According to DTI datasheet

        # Set according to tyre radial velocity through gearbox
        self.radial_velocity = 0
        
        # Set according to drive logic
        self.torque_request = 0
        
        self.producable_torque = 0
        self.torque = 0
        self.power = 0 #Watts

    def update(self, radial_velocity = 1000, torque_request = 0):
        # Set input values
        self.radial_velocity = radial_velocity
        self.torque_request = torque_request
        
        # Determine power limited torque at current wheel speed (And avoid division by zero)
        if self.radial_velocity != 0:
            power_limited_torque = self.power_limit/self.radial_velocity
        else:
            power_limited_torque = self.torque_limit

        # Determine if producable torque is power limit limited or torque limit limited
        if power_limited_torque < self.torque_limit:
            self.producable_torque = power_limited_torque
        else:
            self.producable_torque = self.torque_limit

        # Derate torque approaching max RPM
        rpm_limit_derate_factor = 1
        radial_velocty_remaining = self.max_radial_velocity - self.radial_velocity

        if radial_velocty_remaining < 600:
            rpm_limit_derate_factor = radial_velocty_remaining / 600
            
        self.producable_torque = self.producable_torque * rpm_limit_derate_factor
        
        # Set current torque as requested torque limited to producable torque
        if self.torque_request > self.producable_torque:
            self.torque = self.producable_torque
        else:
            self.torque = self.torque_request
            
        # Calculate motor operating power
        self.power = self.torque * self.radial_velocity
            
class GearboxClass:
    def __init__(self, gear_ratio):
        self.gear_ratio = gear_ratio
        
    def velocity_wheel_to_motor(self, wheel_velocity):
        return wheel_velocity*self.gear_ratio
    
    def velocity_motor_to_wheel(self, motor_velocity):
        return motor_velocity/self.gear_ratio
    
    def torque_wheel_to_motor(self, wheel_torque):
        return wheel_torque/self.gear_ratio
    
    def torque_motor_to_wheel(self, motor_torque):
        return motor_torque*self.gear_ratio

class Drivetrain:
    def __init__(self, drivetrain_parameters):
        self.tyre = TyreClass()
        self.gearbox = GearboxClass(float(drivetrain_parameters["gear_ratio"]))
        self.motor = MotorClass(float(drivetrain_parameters["power_limit"]))
        self.max_current = float(drivetrain_parameters.get("max_current", 200))  # Add to JSON if missing
        self.regen_limit = float(drivetrain_parameters["regen_limit"])
        self.regen_power = 0

    def update(self, vehicle_velocity, fz=0, drive=1, pack_voltage=489.6):
        # Calculate voltage-limited power
        max_power_from_voltage = pack_voltage * self.max_current if pack_voltage > 0 else self.motor.power_limit
        
        # Reduce power limit if voltage is low
        voltage_derated_power = min(self.motor.power_limit, max_power_from_voltage)
        original_power_limit = self.motor.power_limit
        self.motor.power_limit = voltage_derated_power
        
        if drive > 0:
            # On Throttle
            motor_torque_request = drive * self.gearbox.torque_wheel_to_motor(self.tyre.produceable_grip_torque)
            tyre_braking_torque = 0
        else:
            if drive == 0:
                # Coasting
                motor_torque_request = 0
                tyre_braking_torque = 0
            else:
                # Braking
                motor_torque_request = 0
                tyre_braking_torque = drive * self.tyre.produceable_grip_torque
        
        # Update motor object, setting radial velocity
        motor_velocity = self.gearbox.velocity_wheel_to_motor(self.tyre.radial_velocity)
        self.motor.update(motor_velocity, motor_torque_request)
        
        # Update tyre object, setting torque
        tyre_driving_torque = self.gearbox.torque_motor_to_wheel(self.motor.torque)
        self.tyre.update(vehicle_velocity, fz, tyre_driving_torque, tyre_braking_torque) 
        
        # Calculate Braking Power
        self.regen_power = -tyre_braking_torque * self.tyre.radial_velocity
        if self.regen_power > self.regen_limit:
            self.regen_power = self.regen_limit
        
        # Restore original power limit for next iteration
        self.motor.power_limit = original_power_limit