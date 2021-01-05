import abc

import numpy as np

class Controller(object):

    def __init__(self, setpoint):
       
        self._setpoint = setpoint

    def _error(self, process_variable):
        return self._setpoint - process_variable

    @abc.abstractmethod
    def tick(self, process_variable, dt):
        """Controls the process at every instantaneous timestep (tick).
        
        Must be overridden by the subclass.
        Args:
            process_variable: The measured value of the process at the current
                insantenous timestep.
            dt: The elapsed time since this method was last called.
        Returns:
            The control signal in (-inf, inf). It is the responsiblity of the
            caller to convert the control signal to a range usable by the 
            process.
        """


class OnOffController(Controller):
    

    def tick(self, process_var, dt):
        del dt  # Unused.
        return np.inf * self._error(process_var)

class PIDController(Controller):
    
    def __init__(self, setpoint, kp=1., ki=0., kd=1.):
        
        super(PIDController, self).__init__(setpoint=setpoint)

        self._kp = kp
        self._ki = ki
        self._kd = kd

        self._error_previous = 0
        self._error_integral = 0

    def tick(self, process_var, dt):
        error = self._error(process_var)
        self._error_integral += error * dt
        derivative = (error - self._error_previous) / dt
        self._error_previous = error
        return (self._kp * error + 
                self._ki * self._error_integral + 
                self._kd * derivative)