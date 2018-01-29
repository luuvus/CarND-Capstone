
MIN_NUM = float('-inf')
MAX_NUM = float('inf')


class PID(object):
    def __init__(self, kp, ki, kd, mn=MIN_NUM, mx=MAX_NUM):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.min = mn
        self.max = mx

        self.int_val = self.last_int_val = self.last_error = 0.

    def reset(self):
        self.int_val = 0.0
        self.last_int_val = 0.0

    def step_v1(self, error, sample_time):
        self.last_int_val = self.int_val

        integral = self.int_val + error * sample_time;
        derivative = (error - self.last_error) / sample_time;

        y = self.kp * error + self.ki * self.int_val + self.kd * derivative;
        val = max(self.min, min(y, self.max))

        if val > self.max:
            val = self.max
        elif val < self.min:
            val = self.min
        else:
            self.int_val = integral
        self.last_error = error

        return val

    def step(self, error, sample_time):
        integral = self.int_val + error * sample_time

        if sample_time > 1.0e-3:
            derivative = (error - self.last_error) / sample_time
        else:
            derivative = 0.0

        val = self.kp * error + self.ki * self.int_val + self.kd * derivative

        # Take into account actuator limits
        if val > self.max:
            val = self.max
        elif val < self.min:
            val = self.min
        else:
            # Accumulate integral error only if we didn't reach actuator limits
            self.int_val = integral

        self.last_error = error

        return val

