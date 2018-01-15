from pid import PID
from yaw_controller import YawController
from lowpass import LowPassFilter

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, *args, **kwargs):
        # TODO: Implement
		# linear velocity controller
        self.longitudinal = PID(kp=0.4, ki=0.001, kd=0.1, mn=kwargs['decel_limit'], mx=kwargs['accel_limit'])
		# angular velocity controller
        self.lateral = YawController(wheel_base=kwargs['wheel_base'],
                                     steer_ratio=kwargs['steer_ratio'],
                                     min_speed=0,
                                     max_lat_accel=kwargs['max_lat_accel'],
                                     max_steer_angle=kwargs['max_steer_angle'])
		# low-pass filter not used not
        self.LPF = LowPassFilter(tau=1, ts=1)
        

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
            brake = longitudinal_control
        # compute the steering angle
        steer_angle = self.lateral.get_steering(linspd_tar, rotspd_tar, linspd_current)

        return throttle, brake, steer_angle
