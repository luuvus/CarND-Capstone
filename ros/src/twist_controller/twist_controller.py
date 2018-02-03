from pid import PID
from yaw_controller import YawController
from lowpass import LowPassFilter

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, *args, **kwargs):
        # TODO: Implement
		# linear velocity controller
        self.longitudinal = PID(kp=0.4, ki=0.01, kd=0.1, mn=kwargs['decel_limit'], mx=kwargs['accel_limit'])
		# angular velocity controller
        self.lateral = YawController(wheel_base=kwargs['wheel_base'],
                                     steer_ratio=kwargs['steer_ratio'],
                                     min_speed=0,
                                     max_lat_accel=kwargs['max_lat_accel'],
                                     max_steer_angle=kwargs['max_steer_angle'])
		# low-pass filter not used not
        self.LPF = LowPassFilter(tau=1, ts=1)

        # vehicle mass and radius
        self.vehicle_mass = kwargs['vehicle_mass'] 
        self.wheel_radius = kwargs['wheel_radius']
        self.steer_ratio = kwargs['steer_ratio']
        self.comp_in_prev = 0
        self.comp_o_prev = 0;

    def control(self, linspd_current, linspd_tar, rotspd_tar, dt):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
		# velocity error
        linspd_err = linspd_tar - linspd_current
		# compute the drive force
        longitudinal_control = self.longitudinal.step(linspd_err, dt)
        if longitudinal_control >= 0:
            throttle = longitudinal_control
            brake = 0
        else:
            throttle = 0          
            brake = -longitudinal_control*self.vehicle_mass*self.wheel_radius
            
        # compute the steering angle
        steer_angle = self.lateral.get_steering(linspd_tar, rotspd_tar, linspd_current)
        comp_active = False
        if comp_active:
		    T = dt
		    tau1 = 1
		    tau2 = 10
		    k = 0.015
		    c1 = -(T*tau2-1)
		    c2 = k*tau2/tau1
		    c3 = c2*(T*tau1-1)
		    comp_in = steer_angle 
		    comp_o = c1*self.comp_o_prev+c2*comp_in-c3*self.comp_in_prev
		    self.comp_in_prev = comp_in
		    self.comp_o_prev = comp_o
		    steer_angle = comp_o

 
        

        return throttle, brake, steer_angle
