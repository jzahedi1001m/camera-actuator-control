import numpy as np
import matplotlib.pyplot as plt

from plant import *
from controller import *
from kalman import *
from sensor import *

if __name__ == "__main__":
    dt = 0.001
    T = 1.5
    N = int(T / dt)

    # Initialize Hardware Blocks
    vcm_af = VoiceCoilMotorAF(dt=dt)
    ois_xy = OISActuatorXY(dt=dt)
    tremor = HandTremorGenerator(dt=dt)
    
    # Initialize Digital Sensor Modules
    af_sensor = HardwareFocusSensor(dt=dt, latency_seconds=0.006)
    gyro_sensor = HardwareGyroSensor(dt=dt)

    # Initialize Controller & Kalman Filter
    kf_af = Kalman2D(m=vcm_af.m, c=vcm_af.c, k=vcm_af.k, dt=dt, q_scale=1e-2, r_scale=1e-3)
    kf_ois = Kalman2D(m=ois_xy.m, c=ois_xy.c, k=ois_xy.k, dt=dt, q_scale=5e-2, r_scale=5e-4)
    
    pid_af = PID(kp=90, ki=450, kd=2.2, dt=dt, max_output=vcm_af.max_u, deadband=0.0004)
    pid_ois = PID(kp=120, ki=600, kd=1.5, dt=dt, max_output=ois_xy.max_u, deadband=0.0)

    # Targets
    af_target_focus = 0.04  # 40mm focus 
    ois_target_center = 0.0 # Zero optical shift

    # Pre-allocation
    log_time = np.linspace(0, T, N)
    log_af_true, log_af_meas, log_af_est = np.zeros(N), np.zeros(N), np.zeros(N)
    log_ois_residual_error, log_tremor, log_u_af, log_u_ois = np.zeros(N), np.zeros(N), np.zeros(N), np.zeros(N)

    u_sat_af = 0.0
    u_sat_ois = 0.0

    # Simulation
    for k in range(N):
        
        # Physical Disturbance Processing
        chassis_shake = tremor.sample()
        
        # Signal Acquisition
        true_relative_af_position = vcm_af.x + chassis_shake
        z_af = af_sensor.read(true_relative_af_position)
        z_ois_vel = gyro_sensor.read(ois_xy.v + np.random.normal(0, 0.01)) 
        
        # Kalman Filtering (update)
        xhat_af = kf_af.update(z_af)
        xhat_ois = kf_ois.update(ois_xy.x) # OIS encoder positioning loop update
        
        # Control: PID + State-Observed Feedback
        u_af = pid_af.update(af_target_focus, xhat_af) 
        u_ois = pid_ois.update(-chassis_shake, xhat_ois)
        
        # Physical Hardware Execution
        _, u_sat_af = vcm_af.step(u_af)
        _, u_sat_ois = ois_xy.step(u_ois)
        
        # Kalman Filtering (predict)
        kf_af.predict(u_sat_af)
        kf_ois.predict(u_sat_ois)
        


        # Store Data
        log_af_true[k] = true_relative_af_position
        log_af_meas[k] = z_af
        log_af_est[k] = xhat_af[0, 0]
        log_tremor[k] = chassis_shake
        log_ois_residual_error[k] = current_optical_misalignment = chassis_shake + ois_xy.x
        log_u_af[k] = u_sat_af
        log_u_ois[k] = u_sat_ois

    # Plots
    plt.figure(figsize=(12, 9))

    # Focus Lens Performance
    plt.subplot(3, 1, 1)
    plt.plot(log_time, log_af_true, label='True Z-Axis Position (VCM + Jitter)', color='blue', linewidth=2)
    plt.plot(log_time, log_af_meas, label='Sensor Signal (Delayed)', color='gray', alpha=0.35)
    plt.plot(log_time, log_af_est, '--', label='KF Estimate', color='red')
    plt.axhspan(af_target_focus - 0.0004, af_target_focus + 0.0004, color='green', alpha=0.1, label='Soft Deadband')
    plt.axhline(y=af_target_focus, color='black', linestyle=':')
    plt.ylabel('AF Target Space (m)')
    plt.title('Production-Grade Co-Design Validation: Coupled Active OIS & Focus Loop Simulation')
    plt.legend(loc='lower right', fontsize=9)
    plt.grid(True, alpha=0.25)

    # Active OIS Jitter Cancellation
    plt.subplot(3, 1, 2)
    plt.plot(log_time, log_tremor, label='Raw Hand Tremor ($d_{jitter}$)', color='orange', alpha=0.6)
    plt.plot(log_time, log_ois_residual_error, label='Stabilized Optical Error', color='green', linewidth=1.5)
    plt.ylabel('OIS Spatial Error (m)')
    plt.legend(loc='lower right', fontsize=9)
    plt.grid(True, alpha=0.25)

    # Actuation Effort Output
    plt.subplot(3, 1, 3)
    plt.plot(log_time, log_u_af, label='AF Motor ($u_{sat}$)', color='purple', alpha=0.8)
    plt.plot(log_time, log_u_ois, label='OIS Motor ($u_{sat}$)', color='magenta', alpha=0.8)
    plt.xlabel('Time Context (Seconds)')
    plt.ylabel('Voltage Rail Commands')
    plt.legend(loc='lower right', fontsize=9)
    plt.grid(True, alpha=0.25)

    plt.tight_layout()
    plt.show()