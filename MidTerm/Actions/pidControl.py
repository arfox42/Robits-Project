#!/usr/bin/python
"""
Created by Vlad on 6/20/2018
using PyCharm

Implementation of a Proportional-Integral-Derivative (PID) Controller.

"""

import time


class PID:
    """
    PID Controller
    """

    def __init__(self, P=0.2, I=0.02, D=0.01, windupGuard=20):

        # reset the rest of PID parameters to default values
        # "clear()" is an internal new method that
        # redefines the Python standard function call with the same name.
        self.clear()

        # key PID parameters
        self.Kp = P
        self.Ki = I
        self.Kd = D
        self.windup_guard = windupGuard

        self.sample_time = 0.00
        self.current_time = time.time()
        self.last_time = self.current_time

        pass

    def clear(self):
        """Resets PID coefficients and states"""
        self.SetPoint = 0.0

        self.PTerm = 0.0
        self.ITerm = 0.0
        self.DTerm = 0.0
        self.last_error = 0.0

        # Windup Guard
        self.int_error = 0.0
        self.windup_guard = 20.0

        self.output = 0.0

    def update(self, feedback_value):
        """Calculates PID command for a given reference & feedback
        .. math::
            u(t) = K_p e(t) + K_i \int_{0}^{t} e(t)dt + K_d {de}/{dt}
        """

        # TODO:: a threshold to the error might be effective here:: if abs(err) < threshold : err = 0.
        error =  self.SetPoint - feedback_value

        self.current_time = time.time()
        delta_time = self.current_time - self.last_time
        delta_error = error - self.last_error

        if (delta_time >= self.sample_time):
            self.PTerm = self.Kp * error
            self.ITerm += error * delta_time

            if (self.ITerm < -self.windup_guard):
                self.ITerm = -self.windup_guard
            elif (self.ITerm > self.windup_guard):
                self.ITerm = self.windup_guard

            self.DTerm = 0.0
            if delta_time > 0:
                self.DTerm = delta_error / delta_time

            # Remember the last time and last error for next calculation
            self.last_time = self.current_time
            self.last_error = error

            self.output = self.PTerm + (self.Ki * self.ITerm) + (self.Kd * self.DTerm)

    def setKp(self, proportional_gain):
        """ Proportional Gain"""
        self.Kp = proportional_gain

    def setKi(self, integral_gain):
        """ Integral Gain"""
        self.Ki = integral_gain

    def setKd(self, derivative_gain):
        """ Derivative Gain"""
        self.Kd = derivative_gain

    def setWindup(self, windup):
        """ Integral windup bound """
        self.windup_guard = windup

    def setSampleTime(self, sample_time):
        """PID should be updated at a known&regular interval.
        Based on a pre-determined sampe time, the PID decides if it should compute or return immediately.
        """
        self.sample_time = sample_time


if __name__ == '__main__':
    pass
