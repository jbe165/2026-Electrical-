class DynamicLoading:
    def __init__(self, mass, dynamic_loading_parameters):
        self.weight = mass*10
        self.FR_z = self.weight/4
        self.FL_z = self.weight/4
        self.RR_z = self.weight/4
        self.RL_z = self.weight/4
        
        self.cg_height = float(dynamic_loading_parameters['cg_height'])
        self.wheel_base = float(dynamic_loading_parameters['wheel_base'])

    def update(self, vehicle_f_x):
        if vehicle_f_x != 0:
            # TODO weight distribution
            vehicle_y_moment = vehicle_f_x*self.cg_height
            f_z_delta = vehicle_y_moment/(self.wheel_base/2)
        else:
            f_z_delta = 0

        self.FR_z = self.weight/4 - f_z_delta/2
        self.FL_z = self.weight/4 - f_z_delta/2
        self.RR_z = self.weight/4 + f_z_delta/2
        self.RL_z = self.weight/4 + f_z_delta/2