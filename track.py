import math

class Track:
    def __init__(self, track_segments, vehicle):
        self.current_segment_index = 0
        self.track_segment = [TrackSegment(0,0,0)]
        self.track_segment = track_segments
        self.segment_markers = []
        self.step_increment = 0
        self.current_segment = self.track_segment[self.current_segment_index]
        self.next_segment = None

        for i in range(len(self.track_segment)):
            self.track_segment[i].compute_velocity_limit(vehicle)
            if i > 0:
                self.track_segment[i-1].exit_velocity = self.track_segment[i].velocity_limit

    def reset(self):
        """Reset track state for a new lap"""
        self.current_segment_index = 0
        self.segment_markers = []
        self.step_increment = 0
        # Reset all track segments
        for segment in self.track_segment:
            segment.distance_travelled = 0
            segment.distance_remaining = segment.length
            segment.braking = 0

    def drive(self, vehicle, timestep):
        drive_value = self.track_segment[self.current_segment_index].drive(vehicle)

        if not self.track_segment[self.current_segment_index].update(vehicle.kinematics.velocity, timestep):
            self.current_segment_index += 1
            self.segment_markers.append(self.step_increment)

        self.step_increment += 1

        return drive_value

    def is_driving(self):
        return self.current_segment_index < len(self.track_segment)

class TrackSegment:
    def __init__(self, length, bend_radius=float(0), bend_angle=float(0)):
        self.length = length
        self.bend_radius = bend_radius
        if bend_angle != 0:
            self.length = (2 * math.pi * bend_radius) * (bend_angle/360)
        self.distance_travelled = 0
        self.velocity_limit = 1000
        self.exit_velocity = 1000
        self.braking = 0
        self.distance_remaining = self.length
        
    def update(self, vehicle_velocity, timestep):
        self.distance_travelled += vehicle_velocity * timestep
        self.distance_remaining = self.length - self.distance_travelled
        
        if self.distance_travelled>self.length:
            return False
        return True
    
    def drive(self, vehicle):
        # Determine if reached braking point
        if self.distance_remaining <= vehicle.kinematics.braking_distance(self.exit_velocity,vehicle.producable_grip_force):
            self.braking = True
            
        if self.braking:
            if vehicle.kinematics.velocity > self.exit_velocity:
                return -1 #Continue braking
            else:
                #Reached target velocity
                return self.velocity_hold_PID(self.exit_velocity, vehicle.kinematics.velocity)
            
        return self.velocity_hold_PID(self.velocity_limit, vehicle.kinematics.velocity)

    def velocity_hold_PID(self, target_velocity, current_velocity):
        error = target_velocity - current_velocity
        
        drive_request = error*1
        
        if drive_request > 1:
            drive_request = 1
        if drive_request < -1:
            drive_request = -1
        
        return drive_request
    
    def compute_velocity_limit(self, vehicle):
        # f=mv^2/r
        # v=sqrt(f*r/m)
        if self.bend_radius !=0:
            self.velocity_limit = math.sqrt(vehicle.producable_grip_force*self.bend_radius/vehicle.mass)