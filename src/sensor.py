import numpy as np
from collections import deque

class HardwareFocusSensor:
    """ Image-based AF sensor with latency and quantization """
    def __init__(self, dt=0.001, latency_seconds=0.006, resolution=1e-5, noise_std=0.0008):
        self.dt = dt
        self.resolution = resolution
        self.noise_std = noise_std
        self.delay_steps = max(1, int(latency_seconds / dt))
        self.pipeline = deque([0.0] * self.delay_steps, maxlen=self.delay_steps)

    def read(self, true_relative_pos):
        # wideband readout noise
        noisy = true_relative_pos + np.random.normal(0, self.noise_std)
        # quantization (discrete encoder steps)
        quantized = np.round(noisy / self.resolution) * self.resolution
        self.pipeline.append(quantized)
        return self.pipeline[0]

class HardwareGyroSensor:
    """ MEMS Gyroscope sensor for the OIS loop """
    def __init__(self, dt=0.001, bias_drift=1e-4, noise_std=0.0003):
        self.dt = dt
        self.bias = 0.0
        self.bias_drift = bias_drift
        self.noise_std = noise_std

    def read(self, true_velocity):
        #  random walk bias drift over time
        self.bias += np.random.normal(0, self.bias_drift)
        # final measurement with high-frequency thermal white noise
        return true_velocity + self.bias + np.random.normal(0, self.noise_std)