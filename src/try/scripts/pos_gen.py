#!/usr/bin/env python3
# file: cpg_reference_node.py
import math
from typing import List, Optional

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Float32MultiArray


class CPGReferenceNode(Node):
    def __init__(self):
        super().__init__('cpg_reference_node')

        # ---------- Parameters (runtime adjustable) ----------
        self.declare_parameter('rate_hz', 100.0)           # update rate
        self.declare_parameter('mode', 'sine')             # 'sine' or 'hopf'
        self.declare_parameter('amplitude', 1.0)           # output amplitude
        self.declare_parameter('frequency', 0.25)           # Hz (for sine)
        self.declare_parameter('offset', 10.0)              # DC offset
        self.declare_parameter('phase_deg', 0.0)           # initial phase for sine

        # Hopf-specific (nonlinear oscillator)
        self.declare_parameter('hopf_mu', 1.0)             # sets limit-cycle radius^2
        self.declare_parameter('hopf_omega', 2.0 * math.pi * 0.5)  # rad/s ≈ 0.5 Hz

        # ---------- Internal state ----------
        self._t0 = self.get_clock().now()
        self._phase0 = math.radians(float(self.get_parameter('phase_deg').value))

        # Hopf states
        self._x = math.sqrt(max(1e-6, float(self.get_parameter('hopf_mu').value)))
        self._y = 0.0

        # Publisher
        self.pub = self.create_publisher(Float32MultiArray, '/reference_subscription', 10)

        # Timer
        self._rate = float(self.get_parameter('rate_hz').value)
        self._dt = 1.0 / self._rate
        self.timer = self.create_timer(self._dt, self._on_timer)

        # Parameter callback (live tuning)
        self.add_on_set_parameters_callback(self._on_params)

        self.get_logger().info('CPG Reference Node started (100 Hz default). '
                               'Params: mode={}: amplitude={}, freq={}'.format(
                                   self.get_parameter('mode').value,
                                   self.get_parameter('amplitude').value,
                                   self.get_parameter('frequency').value))

    # ---- Parameter update handler ----
    def _on_params(self, params: List[Parameter]):
        for p in params:
            if p.name == 'rate_hz' and p.type_ in (Parameter.Type.DOUBLE, Parameter.Type.INTEGER):
                new_rate = float(p.value)
                if new_rate > 0.0:
                    self._rate = new_rate
                    self._dt = 1.0 / self._rate
                    self.timer.cancel()
                    self.timer = self.create_timer(self._dt, self._on_timer)
            elif p.name == 'phase_deg':
                self._phase0 = math.radians(float(p.value))
                # reset start time so phase change is immediate
                self._t0 = self.get_clock().now()
            elif p.name in ('hopf_mu',):
                # re-initialize Hopf radius when mu changes
                self._x = math.sqrt(max(1e-6, float(p.value)))
                self._y = 0.0
            elif p.name in ('mode', 'amplitude', 'frequency', 'offset', 'hopf_omega'):
                # nothing special; will take effect next tick
                pass
        return rclpy.parameter.SetParametersResult(successful=True)

    # ---- Timer callback (100 Hz by default) ----
    def _on_timer(self):
        mode = str(self.get_parameter('mode').value).lower()
        amplitude = float(self.get_parameter('amplitude').value)
        offset = float(self.get_parameter('offset').value)

        if mode == 'sine':
            freq = float(self.get_parameter('frequency').value)  # Hz
            t = (self.get_clock().now() - self._t0).nanoseconds * 1e-9
            desired = offset + amplitude * math.sin(2.0 * math.pi * freq * t + self._phase0)

        elif mode == 'hopf':
            # Hopf oscillator integration (Euler)
            mu = float(self.get_parameter('hopf_mu').value)
            omega = float(self.get_parameter('hopf_omega').value)  # rad/s
            r2 = self._x * self._x + self._y * self._y
            dx = (mu - r2) * self._x - omega * self._y
            dy = (mu - r2) * self._y + omega * self._x
            self._x += self._dt * dx
            self._y += self._dt * dy

            # Scale Hopf x to user amplitude (limit-cycle radius ≈ sqrt(mu))
            scale = amplitude / max(1e-6, math.sqrt(max(1e-9, mu)))
            desired = offset + scale * self._x
        else:
            self.get_logger().warn(f"Unknown mode '{mode}', defaulting to sine.")
            freq = float(self.get_parameter('frequency').value)
            t = (self.get_clock().now() - self._t0).nanoseconds * 1e-9
            desired = offset + amplitude * math.sin(2.0 * math.pi * freq * t + self._phase0)

        msg = Float32MultiArray()
        msg.data = [float(desired), 0.0]  # [desired_pos, 0]
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = CPGReferenceNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
