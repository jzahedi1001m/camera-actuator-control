# Dual-Axis Mechatronic Lens Control: Active OIS & Autofocus Simulation

This repository contains a high-fidelity mechatronic control loop simulation written in Python. It models a dual-axis camera lens assembly running coupled digital control loops for Autofocus (Z-axis) and Active Optical Image Stabilization (OIS) (X/Y-axis).

The framework captures real-world physical limitations, electro-mechanical coupling, multi-harmonic hand tremor profiles, sensor transport latencies, and quantization behaviors.

## System Architecture & Mathematical Core

Instead of a traditional nested cascade structure which introduces phase delays, this system utilizes an Observer-Based State Feedback Control Architecture.
```text
                        +------------------------+
                        |  Hand Tremor (Jitter)  |
                        +-----------+------------+
                                    |
                                    v (Disturbance)
+-----------+   u_sat   +-----------+------------+   z (Noisy)   +-------------------+
|  PID +    +---------->+   Voice Coil Motors    +-------------->+  Hardware Sensors |
| Anti-Wind |           | (Optics Plant Physics) |               |  (With Latency)   |
+-----+-----+           +------------------------+               +---------+---------+
      ^                                                                    |
      |                 +------------------------+                         |
      +----------------+     Kalman Filter        +<-----------------------+
             xhat       |    (State Observer)    |
         [Pos, Vel]^T   +------------------------+

```
### 1. Electro-Mechanical Plants (src/plant.py)
* Autofocus (Z-Axis): Modeled as an electro-mechanical Voice Coil Motor (VCM) with moving lens mass (m = 10g), back-EMF viscous damping (c = 0.1 N*s/m), and structural centering suspension springs (k = 5.0 N/m).
* Active OIS (X/Y-Axes): Models the orthogonal optical elements shifting relative to the chassis (m = 5g, c = 0.05 N*s/m, k = 2.0 N/m).
* Disturbance Engine: Generates a multi-harmonic hand tremor profile combining slow physiological breathing drift (~3.5 Hz) with rapid muscular micro-tremors (~11.8 Hz).

### 2. Transducer & Sensor Simulation (src/sensor.py)
* Focus Sensor Bus: Simulates an image-based focus acquisition metric featuring a hard 6ms transport pipeline delay alongside spatial quantization matching digital encoder grid resolutions (10 micrometers).
* MEMS Gyroscope: Captures structural chassis velocities degraded by high-frequency wideband thermal white noise and non-stationary random walk zero-rate bias drift.

### 3. State Estimation & Filtering (src/kalman.py)
Both control channels implement synchronous Kalman Filters to reconstruct joint Position (x) and Velocity (v) states from noisy outputs. To handle unpredictable environments safely, the filter applies a 3.5-Sigma Mahalanobis Innovation Validation Gate to actively identify and reject transient measurement outlier spikes.

### 4. Controller (src/controller.py)
The tracking loops implement an industrial PID design featuring:
* Direct State-Feedback Damping: Evaluates derivative tracking corrections (Kd) directly on the observer's clean velocity state estimate to entirely eliminate derivative noise amplification.
* Continuous Soft-Mapping Deadband: Eliminates stationary micro-hunting and H-bridge voice coil heating by smoothly wrapping error tracking down to zero within a threshold (+/- 0.4 mm).
* Conditional Clamping Anti-Windup: Disables integration tracking instantly when the actuator commands hit physical driver H-bridge saturation voltage limits (0.5V for AF, 0.4V for OIS).

---

## Getting Started

### Prerequisites
* Python 3.8 or higher

### Installation
1. Clone this repository to your machine:
   git clone https://github.com/yourusername/camera-actuator-control.git
   cd camera-actuator-control

2. Install the required execution and plotting dependencies:
   pip install -r requirements.txt

### Running the Simulation
To execute the runtime hardware simulator loop and view the real-time control, run:
```bash
python src/autofocus_sim.py
```


### Simulation Telemetry & Graph Analysis
When executed, the system tracks a 40mm step command on the autofocus plane under active handheld vibration. The generated output plot tracks three structural engineering layers:
* Top Subplot (Autofocus Performance): Demonstrates how the 2D Kalman filter uses dead reckoning to accurately track the true lens position across the sensor's 6ms blind window, preventing loop oscillations.
* Middle Subplot (Active OIS Attenuation): Contrasts the raw input hand tremor against the residual optical misalignment, showcasing wideband stabilization.
* Bottom Subplot (H-Bridge Commits): Traces the voltage commands sent to the actuators, illustrating anti-windup clamping during sudden movements and smooth deadband holding once the target is acquired.
