import numpy as np
from collections import deque

class VoiceCoilMotorAF:
    """ Models the 1D physical lens barrel dynamics on the Z-axis (Autofocus) """
    def __init__(self, m=0.01, c=0.1, k=5.0, dt=0.001, max_u=0.3):
        self.m = m          # Lens mass (kg)
        self.c = c          # Viscous damping / back-EMF (N*s/m)
        self.k = k          # Suspension spring constant (N/m)
        self.dt = dt        # Control loop step period (s)
        self.x = 0.0        # Lens physical position relative to chassis (m)
        self.v = 0.0        # Lens physical velocity (m/s)
        self.max_u = max_u  # Voltage/Current driver saturation limit

    def step(self, u):
        # Hardware constraint
        u_sat = np.clip(u, -self.max_u, self.max_u)
        # F_motor - F_damping - F_spring = m * a
        a = (u_sat - self.c * self.v - self.k * self.x) / self.m
        self.v += a * self.dt
        self.x += self.v * self.dt
        return self.x, u_sat


class OISActuatorXY:
    """ Models orthogonal active stabilization element (X/Y-axis) """
    def __init__(self, m=0.005, c=0.05, k=2.0, dt=0.001, max_u=0.4):
        self.m = m
        self.c = c
        self.k = k
        self.dt = dt
        self.x = 0.0        # OIS lens element displacement relative to chassis (m)
        self.v = 0.0        # OIS lens element velocity (m/s)
        self.max_u = max_u

    def step(self, u):
        u_sat = np.clip(u, -self.max_u, self.max_u)
        a = (u_sat - self.c * self.v - self.k * self.x) / self.m
        self.v += a * self.dt
        self.x += self.v * self.dt
        return self.x, u_sat    


class HandTremorGenerator:
    """ real-world multi-harmonic human hand tremor profiles """
    def __init__(self, dt=0.001):
        self.dt = dt
        self.time = 0.0

    def sample(self):
        self.time += self.dt
        # Low-frequency breathing/sway component (~3.5 Hz)
        low_freq = 0.0015 * np.sin(2 * np.pi * 3.5 * self.time)
        # High-frequency structural micro-tremor (~11.8 Hz)
        high_freq = 0.0004 * np.sin(2 * np.pi * 11.8 * self.time)
        # Broadband ambient structural noise
        noise = np.random.normal(0, 0.00005)
        return low_freq + high_freq + noise