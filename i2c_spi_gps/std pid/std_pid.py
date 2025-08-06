#
# standard PID controller 
# rev 1.1 added a timebase to the derivative option
#

import numpy as np
import time

class BaseController:
    def update(self, target_val, current_val, state, future_plan):
    """
    Args:
      target_val: The target lateral acceleration.
      current_val: The current lateral acceleration.
      maxv: maximum output value
      minv: minimum output value
    Returns:
      The control signal to be applied to the vehicle.
    """
        raise NotImplementedError

class Controller(BaseController):
    """
    A simple PID controller with or without cycle time use
    """
    def __init__(self, p=0.3, i=0.05, d=-0.1, o=0.0, use_time=1):
        self.p = p 
        self.i = i
        self.d = d 
        self.error_integral = 0
        self.prev_e_cnt = 0
        self.prev_error = 0
        self.out = o
        self.use_t = use_time
        self.t_ref = time.time()

    def clamp(self, inval, maxv, minv):
        if inval > maxv:
            inval = maxv
        elif inval < minv:
            inval = minv
        return inval

    def update(self, target_val, current_val, minv=0, maxv=4096):
        error = (target_val - current_val)
        self.error_integral += error
        if self.prev_e_cnt == 0:
            error_diff = 0
            self.prev_e_cnt += 1
        else:			
            error_diff = error - self.prev_error
        self.prev_error = error
        if self.use_t == 0:
            self.out = self.clamp(self.p * error + self.i * self.error_integral + self.d * error_diff, maxv, minv)
        elif self.use_t == 1:
            cur_t = time.time()
            tdiff = cur_t - self.t_ref 
            self.out = self.clamp(self.p * error + self.i * self.error_integral + ((self.d * error_diff)*tdiff), maxv, minv) 
            self.t_ref = cur_t
        return self.out

    def hold(self,):
        return self.out

    def reset(self,):
        self.error_integral = 0
        self.prev_error = 0
        self.out = 0.0
        return self.out
		
    def ramp_up(self,rv, maxv):
        self.out += rv
        if self.out > maxv:
            self.out = maxv
        return self.out	

    def ramp_down(self,rv, minv):
        self.out += rv
        if self.out < minv:
            self.out = minv
        return self.out	

    def manual(self,o):
        self.out = o
        self.error_integral = 0
        self.prev_e_cnt = 0