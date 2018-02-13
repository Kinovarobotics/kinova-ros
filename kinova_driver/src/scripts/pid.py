""" 
A simple PID controller class.  

This is a mostly literal C++ -> Python translation of the ROS
control_toolbox Pid class: http://ros.org/wiki/control_toolbox.
"""

import time
import math
import numpy as np

#*******************************************************************
# Translated from pid.cpp by Nathan Sprague
# Jan. 2013 (Modified Jan. 2014)
# See below for original license information:
#*******************************************************************

#******************************************************************* 
# Software License Agreement (BSD License)
#
#  Copyright (c) 2008, Willow Garage, Inc.
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   * Neither the name of the Willow Garage nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.
#******************************************************************* 



class PID(object):
    """ A basic adapted pid class for 7Dof robot.

    This class implements a generic structure that can be used to
    create a wide range of pid controllers. It can function
    independently or be subclassed to provide more specific controls
    based on a particular control loop.

    In particular, this class implements the standard pid equation:

    $command = p_{term} + i_{term} + d_{term} $

    where:

    $ p_{term} = p_{gain} * p_{error} $
    $ i_{term} = i_{gain} * i_{error} $
    $ d_{term} = d_{gain} * d_{error} $
    $ i_{error} = i_{error} + p_{error} * dt $
    $ d_{error} = (p_{error} - p_{error last}) / dt $

    given:

    $ p_{error} = p_{target} - p_{state} $.
    """

    def __init__(self, p_gain, i_gain, d_gain, i_min, i_max):
        """Constructor, zeros out Pid values when created and
        initialize Pid-gains and integral term limits. All gains are 
		7x7 matrices.

        Parameters:
          p_gain     The proportional gain.
          i_gain     The integral gain.
          d_gain     The derivative gain.
          i_min      The integral lower limit. 
          i_max      The integral upper limit.
        """
        self.set_gains(p_gain, i_gain, d_gain, i_min, i_max)
        self.reset()

    def reset(self):
        """  Reset the state of this PID controller """
        self._p_error_last = np.zeros((7,1)) # Save position state for derivative
                                 # state calculation.
        self._p_error = np.zeros((7,1))  # Position error.
        self._d_error = np.zeros((7,1))  # Derivative error.
        self._i_error = np.zeros((7,1))  # Integator error.
        self._cmd = np.zeros((7,7))  # Command to send.
        self._last_time = None # Used for automatic calculation of dt.
        
    def set_gains(self, p_gain, i_gain, d_gain, i_min, i_max): 
        """ Set PID gains for the controller. 

         Parameters:
          p_gain     The proportional gain.
          i_gain     The integral gain.
          d_gain     The derivative gain.
          i_min      The integral lower limit. 
          i_max      The integral upper limit.
        """ 
        self._p_gain = p_gain
        self._i_gain = i_gain
        self._d_gain = d_gain
        self._i_min = i_min
        self._i_max = i_max

    @property
    def p_gain(self):
        """ Read-only access to p_gain. """
        return self._p_gain

    @property
    def i_gain(self):
        """ Read-only access to i_gain. """
        return self._i_gain

    @property
    def d_gain(self):
        """ Read-only access to d_gain. """
        return self._d_gain

    @property
    def i_max(self):
        """ Read-only access to i_max. """
        return self._i_max

    @property
    def i_min(self):
        """ Read-only access to i_min. """
        return self._i_min

    @property
    def p_error(self):
        """ Read-only access to p_error. """
        return self._p_error

    @property
    def i_error(self):
        """ Read-only access to i_error. """
        return self._i_error

    @property
    def d_error(self):
        """ Read-only access to d_error. """
        return self._d_error

    @property
    def cmd(self):
        """ Read-only access to the latest command. """
        return self._cmd

    @property
    def last_time(self):
       """ Read-only access to the last time. """
       return self._last_time

    def __str__(self):
        """ String representation of the current state of the controller. """
        result = ""
        result += "p_gain:  " + str(self.p_gain) + "\n"
        result += "i_gain:  " + str(self.i_gain) + "\n"
        result += "d_gain:  " + str(self.d_gain) + "\n"
        result += "i_min:   " + str(self.i_min) + "\n"
        result += "i_max:   " + str(self.i_max) + "\n"
        result += "p_error: " + str(self.p_error) + "\n"
        result += "i_error: " + str(self.i_error) + "\n"
        result += "d_error: " + str(self.d_error) + "\n"
        result += "cmd:     " + str(self.cmd) + "\n"
        return result
        
    def update_PID(self, p_error, dt=None):

        """  Update the Pid loop with nonuniform time step size.

        Parameters:
          p_error  Error since last call (target - state)
          dt       Change in time since last call, in seconds, or None. 
                   If dt is None, then the system clock will be used to 
                   calculate the time since the last update. 
        """
        #print p_error
        if dt == None:
            cur_time = time.time()
            if self._last_time is None:
                self._last_time = cur_time 
            dt = cur_time - self._last_time
            self._last_time = cur_time

        #print "in update_PID(): dt: " + str(dt)

        self._p_error = p_error

        if dt == 0 or math.isnan(dt) or math.isinf(dt):
            return np.zeros((7,7)) # TODO or shold it be 0.0??

        # Calculate proportional contribution to command
        p_term = self._p_gain * self._p_error
		
        #print "in update_PID(): p_term: " + str(p_term)

        #print "in update_PID(): p_gain:" + str(self._p_gain)

        # Calculate the integral error
        self._i_error += dt * self._p_error 
        
        # Calculate integral contribution to command
        i_term = self._i_gain * self._i_error
        
        # Limit i_term so that the limit is meaningful in the output
        """
        if i_term > self._i_max and self._i_gain != 0:
            i_term = self._i_max
            self._i_error = i_term / self._i_gain
        elif i_term < self._i_min and self._i_gain != 0:
            i_term = self._i_min
            self._i_error = i_term / self._i_gain
        """
    
        # Calculate the derivative error
        self._d_error = (self._p_error - self._p_error_last) / dt
        self._p_error_last = self._p_error

        #print "in update_PID(): p_error: " + str(self._p_error)
        #print "in update_PID(): i_error: " + str(self._i_error)
        #print "in update_PID(): d_error: " + str(self._d_error)
        
        # Calculate derivative contribution to command 
        d_term = self._d_gain * self._d_error
        
        self._cmd = p_term + i_term + d_term

        return self._cmd

if __name__ == "__main__":
    P = np.eye(7)
    I = np.zeros((7,7))
    D = np.eye(7)

    controller = PID(P, I, D, -1.0, 1.0)
    print controller
	
    error = np.array([1,2,3,4,5,6,7]).reshape((7,1))
    controller.update_PID(error)

    error = np.array([-0.1,-0.2,-0.3,-0.4,-0.5,-0.6,-0.7]).reshape((7,1))
    controller.update_PID(error)


