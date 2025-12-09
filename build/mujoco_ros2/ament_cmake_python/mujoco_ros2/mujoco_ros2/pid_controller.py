import numpy as np

class PIDController:
    def __init__(self, kp, ki, kd, output_limits=None, sample_time=0.01):
        """
        Discrete PID Controller.
        
        Args:
            kp (float): Proportional gain.
            ki (float): Integral gain.
            kd (float): Derivative gain.
            output_limits (tuple): (min, max) output clamping.
            sample_time (float): Time step in seconds.
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.min_out = output_limits[0] if output_limits else -np.inf
        self.max_out = output_limits[1] if output_limits else np.inf
        self.dt = sample_time
        
        self.integral_sum = 0.0
        self.prev_error = 0.0
        
    def compute(self, target, measured, feedforward=0.0):
        """
        Calculates the control output (velocity command).
        
        Args:
            target (float): Desired position.
            measured (float): Current position.
            feedforward (float): Feedforward velocity (optional).
        """
        # 1. Calculate Error
        error = target - measured
        
        # 2. Proportional Term
        p_term = self.kp * error
        
        # 3. Integral Term (Discrete Sum)
        self.integral_sum += error * self.dt
        # Anti-windup (simple clamping of the integral sum could happen here)
        i_term = self.ki * self.integral_sum
        
        # 4. Derivative Term (Finite Difference)
        d_term = self.kd * (error - self.prev_error) / self.dt
        
        # 5. Total Output
        output = p_term + i_term + d_term + feedforward
        
        # 6. Clamp Output (Saturation)
        output = np.clip(output, self.min_out, self.max_out)
        
        # Save state
        self.prev_error = error
        
        return output
        
    def reset(self):
        self.integral_sum = 0.0
        self.prev_error = 0.0