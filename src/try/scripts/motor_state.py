#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import numpy as np

def std_from_velocity(v):
    a, b = 0.00532515, 0.08041608
    return a * np.asarray(v, dtype=float) + b

def noise_for_velocity(v, rng=None):
    sigma = std_from_velocity(v)
    if isinstance(rng, (int, np.integer)) or rng is None:
        rng = np.random.default_rng(rng)
    return rng.normal(0.0, sigma, size=np.shape(v))

def add_noise_to_velocity(v, rng=None):
    """Return v + N(0, sigma(v))."""
    v = np.asarray(v, float)
    return v + noise_for_velocity(v, rng=rng)


class DCMotorSimSimple(Node):
    def __init__(self):
        super().__init__('dc_motor_sim_simple')
        self.theta_initialized = False

        # ----- Fixed constants 
        self.Kt = 0.0146 * 64           # Nm/A
        self.Ke = 0.0134 * 64          # V/(rad/s)
        self.J  = 5.0e-3          # kg*m^2
        self.b  = 0.00706757271          # Nm/(rad/s)
        self.R  = 1.93        # Ohm
        self.L  = 65.0e-6       # H 
        self.dt = 0.001          # Loop control (1000 hz)

        # ----- Safety limits 
        self.volt_limit    = 12.0     # V
        self.current_limit = 3.5     # A
        self.omega_limit   = 100.0   # rad/s
        self.theta_limit   = 1e6      # rad 
        self.dI_limit_per_step = 200.0  # A/step cap 

        # ----- State -----
        self.current = 0.0
        self.omega   = 0.0
        self.theta   = 0.0
        self.volt_cmd = 0.0
        self.kp = 0.0
        self.kd = 0.0

        # I/O
        self.sub = self.create_subscription(Float32MultiArray, '/motor_feedback_publisher', self._on_feedback, 10)
        self.gain_sub = self.create_subscription(Float32MultiArray, '/gain_subscription', self._gain_sub, 10)
        self.ref_sub = self.create_subscription(Float32MultiArray, '/reference_subscription', self._ref_sub, 10)

        self.pub = self.create_publisher(Float32MultiArray, '/motor_state', 10)

        self.timer = self.create_timer(self.dt, self._on_timer) # Control loop
        self.get_logger().info("DCMotorSimSimple (safe) @ 100 Hz | use data[2] as volt")

    def _on_feedback(self, msg: Float32MultiArray):
        # Initialize theta from feedback at first reception
        if not self.theta_initialized and len(msg.data) >= 1:
            theta0 = float(msg.data[0])
            # clamp to omega_limit to be safe
            self.theta = max(-self.theta_limit, min(self.theta_limit, theta0))
            self.theta_initialized = True
            self.get_logger().info(f"Initialized omega from message: {self.omega:.3f} rad/s")

    def _gain_sub(self, msg: Float32MultiArray):
        self.kp = msg.data[0]
        self.kd = msg.data[1]
        # self.get_logger().info(f"Updated gains: Kp={self.kp}, Kd={self.kd}")
        # self.kt = msg.data[2]
        # self.ke = msg.data[3]
        # self.r = msg.data[4]

    def _ref_sub(self, msg: Float32MultiArray):
        # 100 hz loop
        self.ref_theta = msg.data[0]
        self.ref_omega = msg.data[1]
        
        # Compute voltage command using PD control
        error_theta = self.ref_theta - self.theta
        error_omega = self.ref_omega - self.omega
        volt_cmd = self.kp * error_theta + self.kd * error_omega

        # Clamp voltage command to limits
        self.volt_cmd = max(-self.volt_limit, min(self.volt_limit, volt_cmd))
        # self.get_logger().info(f"Voltage command set to: {self.volt_cmd:.3f} V")


    @staticmethod
    def _finite(x: float, default: float = 0.0) -> float:
        return x if math.isfinite(x) else default


    
    def _on_timer(self):
        # 1000 hz loop
        # volt = self._finite(self.volt_cmd, 0.0)
        volt = self.volt_cmd

        # dcurrent = (volt - (self.R * self.current) - (self.Ke * self.omega)) / max(self.L, 1e-12)
        # dcurrent = max(-self.dI_limit_per_step, min(self.dI_limit_per_step, dcurrent))
        # self.current += dcurrent * self.dt
        
        self.current = (volt - (self.Ke * self.omega)) / self.R
        torque = self.Kt * self.current

        domega = (torque - self.b * self.omega) / self.J
        self.omega += domega * self.dt
        
        self.omega = max(-self.omega_limit, min(self.omega_limit, self.omega))
        self.omega = add_noise_to_velocity(self.omega, rng=None)
        self.theta += self.omega * self.dt

        # Ensure no NaN/Inf before publish
        theta  = float(self._finite(self.theta))
        omega  = float(self._finite(self.omega))
        cur    = float(self._finite(self.current))
        tq     = float(self._finite(torque))
        v_out  = float(self._finite(volt))

        out = Float32MultiArray()
        out.data = [theta, omega, cur, tq, v_out]
        self.pub.publish(out)

def main():
    rclpy.init()
    node = DCMotorSimSimple()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
