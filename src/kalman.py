import numpy as np

class Kalman2D:
    """ Reconstructs true joint Position and Velocity state vectors """
    def __init__(self, m, c, k, dt, q_scale=1e-2, r_scale=1e-3):
        self.dt = dt
        self.m = m
        self.x = np.zeros((2, 1))
        
        # Discretized system transition matrices
        self.A = np.array([[1.0, dt], [-(k/m)*dt, 1.0 - (c/m)*dt]])
        self.B = np.array([[0.0], [(1.0/m)*dt]])
        self.C = np.array([[1.0, 0.0]])
        
        self.Q = q_scale * np.eye(2)
        self.R = np.array([[r_scale]])
        self.P = np.eye(2) * 0.1
        self.gate_limit = 3.5

    def predict(self, u_sat):
        self.x = self.A @ self.x + self.B * u_sat
        self.P = self.A @ self.P @ self.A.T + self.Q

    def update(self, z):
        y = np.array([[z]]) - self.C @ self.x
        S = self.C @ self.P @ self.C.T + self.R
        inn_var = S[0, 0]
        
        # 3.5-Sigma Mahalanobis validation gate to ignore sensor glitches
        if abs(y[0, 0]) / np.sqrt(inn_var) > self.gate_limit:
            return self.x
            
        K = self.P @ self.C.T / inn_var
        self.x = self.x + K * y[0, 0]
        self.P = (np.eye(2) - K @ self.C) @ self.P
        return self.x