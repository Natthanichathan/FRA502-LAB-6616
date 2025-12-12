#!/usr/bin/python3

from imu_calibration.dummy_module import dummy_function, dummy_var
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from ament_index_python import get_package_share_directory
from sensor_interface.msg import SensorParameter
from std_msgs.msg import Float64MultiArray
import numpy as np
import math
import os
import yaml

class Kalman1D:
    def __init__(self, dt, co_a, co_ba, co_e, b_ax_init):
        self.dt = float(dt)
        # state x = [p_x, v_x, b_ax]^T
        self.x = np.zeros((3, 1))   
        self.x[2, 0] = b_ax_init        # define b_ax in initial state
        self.P = np.eye(3) * 1.0        # initial covariance P
        
        self.co_a = float(co_a)         # std of accel noise (IMU)
        self.co_ba = float(co_ba)       # bias uncertainty
        self.co_e = float(co_e)         # std of encoder position

        # Measurement model y = Cx = p_x
        self.C = np.array([[1.0, 0.0, 0.0]])    
        self.R = np.array([[(self.co_e ** 2) ]])   
        self._compute_matrices()                

    def _compute_matrices(self):
        dt = self.dt
        dt_2 = dt * dt

        # State Transition Matrix A
        self.A = np.array([
            [1.0, dt, -0.5 * dt_2],
            [0.0, 1.0, -dt],
            [0.0, 0.0, 1.0]
        ])

        # Control Input Matrix B (u = a_x)
        self.B = np.array([[0.5 * dt_2], [dt], [0.0]])
        
        # Process Noise Covariance Q
        F = np.array([[0.5 * dt_2], [dt]])             
        Q_pv = F @ F.T * (self.co_a ** 2)
        co_p2 = Q_pv[0, 0]
        co_v2 = Q_pv[1, 1]
        co_pv = Q_pv[0, 1]
        self.Q = np.array([
            [co_p2,  co_pv, 0.0],
            [co_pv,  co_v2, 0.0],
            [0.0,    0.0,   self.co_ba]
        ])

    def predict(self, a_x):
        u = np.array([[a_x]])
        self.x = self.A @ self.x + self.B @ u           
        self.P = self.A @ self.P @ self.A.T + self.Q    

    def update(self, p_meas):
        z = np.array([[p_meas]])                        
        y = z - self.C @ self.x
        S = self.C @ self.P @ self.C.T + self.R
        K = self.P @ self.C.T @ np.linalg.inv(S)        
        self.x = self.x + K @ y                         
        I = np.eye(3)                                   
        self.P = (I - K @ self.C) @ self.P

    @property
    def p_x(self): return float(self.x[0, 0])
    @property
    def v_x(self): return float(self.x[1, 0])
    @property
    def b_ax(self): return float(self.x[2, 0])


class SensorFusionNode(Node):
    def __init__(self):
        super().__init__('sensor_fusion_node')

        # --- Parameters ---
        dt = self.declare_parameter('dt', 0.01).value       
        self.encoder_resolution = self.declare_parameter('encoder_resolution', 2048 * 4).value
        self.wheel_radius = self.declare_parameter('wheel_radius', 0.00636).value 
        
        # Low co_e = Trust Encoder highly
        co_e = self.declare_parameter('co_e', 5e-5).value 
    
        # Accelerometer Deadband (m/s^2)
        self.accel_deadband = 0.03 

        # Static Velocity Threshold (m/s)
        self.static_vel_threshold = 0.002 

        # Load Calibration
        pkg_name = 'imu_calibration'
        pkg_share = get_package_share_directory(pkg_name)
        ws_path, _ = pkg_share.split('install')
        yaml_path = os.path.join(ws_path, 'src', pkg_name, 'config', 'imu_calib_data.yaml')

        with open(yaml_path, 'r') as f:
            calib = yaml.safe_load(f)

        accel_cov = np.array(calib['accel covariance']) 
        accel_offset = np.array(calib['accel offset'])
        
        var_ax = float(accel_cov[0][0])
        co_a = np.sqrt(var_ax) * 2.7
        b_ax_init = float(accel_offset[0])
        co_ba = b_ax_init 

        # Kalman Filter Initialization
        self.kf = Kalman1D(dt, co_a, co_ba, co_e, b_ax_init)

        # Encoder Variables
        self.last_ticks = None
        self.current_dist_m = 0.0
        self.MAX_TICKS = 4294967295 
        self.HALF_MAX_TICKS = self.MAX_TICKS // 2

        # Static Detection Counter
        self.static_frame_count = 0

        # ROS Communication
        qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)
        
        self.create_subscription(
            SensorParameter,
            'calibrated_parameter',
            self.sensor_callback,
            qos_profile
        )

        self.publisher = self.create_publisher(
            Float64MultiArray,
            'sensor_fusion_publisher',
            10
        )
        self.get_logger().info('Sensor Fusion Node Started (Anti-Jitter Static Lock)')

    def sensor_callback(self, msg: SensorParameter):
        current_ticks = msg.encoder1.data           
        a_x_raw = msg.imu_mpu6050.linear_acceleration.x 

        # 1. Handle Encoder Wrap-Around Logic
        if self.last_ticks is None:
            self.last_ticks = current_ticks
            return 

        delta_ticks = int(current_ticks) - int(self.last_ticks)

        if delta_ticks > self.HALF_MAX_TICKS:
            delta_ticks -= (self.MAX_TICKS + 1)
        elif delta_ticks < -self.HALF_MAX_TICKS:
            delta_ticks += (self.MAX_TICKS + 1)

        self.last_ticks = current_ticks

        # 2. Convert to Distance
        circumference_m = 2.0 * np.pi * self.wheel_radius
        meter_per_tick = circumference_m / float(self.encoder_resolution)
        
        delta_dist_m = delta_ticks * meter_per_tick
        self.current_dist_m += delta_dist_m

        # Calculate Raw Velocity for Validation & Static Check
        v_encoder_raw = delta_dist_m / self.kf.dt
        delta_dist_m / self.kf

        if abs(v_encoder_raw) < self.static_vel_threshold:
            self.static_frame_count += 1
        else:
            self.static_frame_count = 0

        current_bias = self.kf.b_ax
        net_accel = a_x_raw - current_bias

        used_accel = a_x_raw
        if abs(net_accel) < self.accel_deadband:
            used_accel = current_bias # Forces 0 net acceleration

        # Kalman Filter Prediction & Update
        self.kf.predict(used_accel)
        self.kf.update(self.current_dist_m)

        # Hard Static Lock
        if self.static_frame_count > 10:
            self.kf.x[1, 0] = 0.0

        p_x = self.kf.p_x
        v_x = self.kf.v_x
        acc_bias_comp = a_x_raw - self.kf.b_ax

        out = Float64MultiArray()
        # [0] Position, [1] Fused Vel, [2] Net Accel, [3] Raw Encoder Vel
        # out.data = [p_x, v_x, acc_bias_comp, self.current_dist_m]
        out.data = [p_x, v_x, acc_bias_comp]
        # out.data = [p_x, v_x, acc_bias_comp, v_encoder_raw]
        self.publisher.publish(out)

def main(args=None):
    rclpy.init(args=args)
    node = SensorFusionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()