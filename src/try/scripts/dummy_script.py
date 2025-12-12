#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Float32MultiArray
import csv
import time
from typing import List


class TorqueProfileNode(Node):
    def __init__(self):
        super().__init__('torque_profile_node')

        self.torque = 3.5  # Nm
        self.torque_str = str(self.torque).replace('.', '_')

        # ---- Run configuration ----
        self.num_epochs = 5
        self.epoch_now = 1

        # ---- Timing parameters ----
        self.t_pre = 2.0
        self.t_mid = 30.0
        self.t_post = 0.0
        self.rate_hz = 200.0

        # ---- Derived durations ----
        self.total_duration = self.t_pre + self.t_mid + self.t_post
        self.post_start = self.t_pre + self.t_mid

        # ---- ROS interfaces ----
        qos = QoSProfile(depth=10)
        self.pub = self.create_publisher(Float32MultiArray, '/reference_subscription', qos)
        self.sub = self.create_subscription(Float32MultiArray, '/motor_feedback_publisher', self.feedback_cb, qos)

        # ---- Initialize first epoch ----
        self.start_new_epoch()

        # ---- Timer ----
        self.timer = self.create_timer(1.0 / self.rate_hz, self.timer_cb)

        self.get_logger().info(f"TorqueProfileNode ready for {self.num_epochs} runs.")

    # -----------------------------------------------------------
    # Core logic
    # -----------------------------------------------------------

    def start_new_epoch(self):
        """Start a new epoch: reset time, open CSV, reset logging."""
        self.start_time = self.get_clock().now()
        self.logging_active = True
        self.last_feedback: List[float] = []

        csv_name = f"csv_files/{self.torque_str}_run{self.epoch_now}.csv"
        self.csv_file = open(csv_name, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['stamp_sec', 'elapsed_s', 'cmd_torque', 'fb_0', 'fb_1'])

        self.get_logger().info(
            f"=== Starting epoch {self.epoch_now}/{self.num_epochs}, logging to {csv_name} ==="
        )

    def end_epoch(self):
        """End current epoch, flush and close file."""
        if self.logging_active:
            self.csv_file.flush()
            self.csv_file.close()
            self.logging_active = False
        self.get_logger().info(f"Epoch {self.epoch_now} complete.")

        self.epoch_now += 1
        if self.epoch_now <= self.num_epochs:
            # Short delay to separate epochs
            time.sleep(0.5)
            self.start_new_epoch()
        else:
            self.get_logger().info("All epochs complete. Shutting down.")
            self.destroy_timer(self.timer)
            self.destroy_node()

    # -----------------------------------------------------------
    # Timer and torque profile logic
    # -----------------------------------------------------------

    def current_elapsed(self) -> float:
        now = self.get_clock().now()
        return (now - self.start_time).nanoseconds * 1e-9

    def commanded_torque(self, t: float) -> float:
        if t < self.t_pre:
            return 0.0
        elif t < self.post_start:
            return self.torque
        elif t < self.total_duration:
            return 0.0
        else:
            return 0.0

    def timer_cb(self):
        t = self.current_elapsed()
        cmd_tau = self.commanded_torque(t)

        # publish torque command
        msg = Float32MultiArray()
        msg.data = [0.0, 0.0, float(cmd_tau)]
        self.pub.publish(msg)

        # log feedback
        if self.logging_active:
            stamp_sec = time.time()
            fb0 = self.last_feedback[0] if len(self.last_feedback) > 0 else float('nan')
            fb1 = self.last_feedback[1] if len(self.last_feedback) > 1 else float('nan')
            self.csv_writer.writerow([stamp_sec, t, cmd_tau, fb0, fb1])

        # end of epoch
        if t >= self.total_duration:
            self.end_epoch()

    def feedback_cb(self, msg: Float32MultiArray):
        self.last_feedback = list(msg.data)


def main(args=None):
    rclpy.init(args=args)
    node = TorqueProfileNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user.')
    finally:
        if hasattr(node, "csv_file") and not node.csv_file.closed:
            node.csv_file.close()
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
