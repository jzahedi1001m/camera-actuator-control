import numpy as np

class PID:
    def __init__(self, kp, ki, kd, dt, max_output, deadband=0.0):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.dt = dt
        self.max_output = max_output
        self.max_i = max_output * 0.8
        self.deadband = deadband
        self.i = 0.0

    def update(self, ref, xhat):
        p_hat, v_hat = xhat[0, 0], xhat[1, 0]
        e_raw = ref - p_hat
        
        # Continuous soft-mapping deadband
        if abs(e_raw) <= self.deadband:
            e = 0.0
        else:
            e = e_raw - np.sign(e_raw) * self.deadband
            
        p_term = self.kp * e
        d_term = -self.kd * v_hat  # Clean state feedback differentiation
        
        u_unconstrained = p_term + self.ki * (self.i + e * self.dt) + d_term
        
        # Bounded actuator clamping anti-windup check
        saturated = abs(u_unconstrained) > self.max_output
        same_sign = np.sign(e) == np.sign(u_unconstrained)
        
        if saturated and same_sign:
            pass # Lock integration accumulation
        else:
            self.i += e * self.dt
            self.i = np.clip(self.i, -self.max_i/self.ki, self.max_i/self.ki)
            
        return np.clip(p_term + self.ki * self.i + d_term, -self.max_output, self.max_output)