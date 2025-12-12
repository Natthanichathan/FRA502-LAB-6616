#!/usr/bin/python3

from lab2.dummy_module import dummy_function, dummy_var
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from turtlesim.msg import Pose
from turtlesim_plus_interfaces.srv import GivePosition
from std_srvs.srv import Empty, SetBool
from std_msgs.msg import Bool
import numpy as np
import math
from geometry_msgs.msg import PoseStamped
from controller_interfaces.srv import SetParam
from turtlesim.srv import Kill , Spawn

class DummyNode(Node):
    def __init__(self):
        super().__init__("killer_node")

        self.ns = self.get_namespace().strip("/")
        if self.ns == "":
            self.ns = "killer_turtle"

        # Target eater name from param
        self.declare_parameter("eater_name", "eater_turtle")
        self.eater_name = self.get_parameter("eater_name").value

        # Sampling frequency
        self.declare_parameter("sampling_frequency", 100.0)
        freq = self.get_parameter("sampling_frequency").value

        # Gains
        self.kp_v_temp = 2.0
        self.kp_omega_temp = 4.0
        self.kp_v = 0.0
        self.kp_omega = 0.0

        self.pub_cmd = self.create_publisher(Twist, f"/{self.ns}/cmd_vel", 10)

        self.create_subscription(Pose, f"/{self.ns}/pose", self.killer_pose_callback, 10)
        self.killer_pos = np.array([0.0, 0.0, 0.0])

        self.create_subscription(Pose, f"/{self.eater_name}/pose", self.eater_pose_callback, 10)
        self.eater_pos = np.array([0.0, 0.0, 0.0])

        self.create_subscription(Bool, f"/{self.eater_name}/eat_status", self.eat_status_callback, 10)
        self.eater_is_eating = True

        self.kill_client = self.create_client(Kill, "/remove_turtle")

        self.create_service(SetParam, f"/{self.ns}/set_param", self.set_param_callback)

        self.timer = self.create_timer(1.0 / freq, self.timer_callback)

        self.hunt = False

    def eater_pose_callback(self, msg):
        self.eater_pos[:] = [msg.x, msg.y, msg.theta]

    def killer_pose_callback(self, msg):
        self.killer_pos[:] = [msg.x, msg.y, msg.theta]

    def eat_status_callback(self, msg):
        self.eater_is_eating = msg.data
        self.hunt = not msg.data

    def set_param_callback(self, request, response):
        self.kp_v_temp = request.kp_linear
        self.kp_omega_temp = request.kp_angular
        # self.get_logger().info(f"[KILLER PARAM] kp_v={self.kp_v}, kp_omega={self.kp_omega}")
        return response

    def move(self, v, w):
        msg = Twist()
        msg.linear.x = v
        msg.angular.z = w
        self.pub_cmd.publish(msg)

    def control(self, tx, ty):
        dx = tx - self.killer_pos[0]
        dy = ty - self.killer_pos[1]
        d = math.sqrt(dx*dx + dy*dy)

        target_theta = math.atan2(dy, dx)
        theta_err = math.atan2(math.sin(target_theta - self.killer_pos[2]),
                               math.cos(target_theta - self.killer_pos[2]))

        v = self.kp_v * d
        w = self.kp_omega * theta_err
        self.move(v, w)

        return d

    def timer_callback(self):

        # wait until eater finishes pizza
        if not self.hunt:
            # self.get_logger().info(f'hunt = {self.hunt}')
            self.kp_omega = 0.0
            self.kp_v = 0.0
            self.control(self.eater_pos[0], self.eater_pos[1])
            return
        else:
            self.get_logger().info(f'w = {self.kp_omega}, v = {self.kp_v}')
            self.kp_omega = self.kp_omega_temp
            self.kp_v = self.kp_v_temp

        dist = self.control(self.eater_pos[0], self.eater_pos[1])

        if dist < 0.5 and self.hunt:
            req = Kill.Request()
            req.name = self.eater_name
            self.kill_client.call_async(req)
            self.kp_omega = 0.0
            self.kp_v = 0.0


def main(args=None):
    rclpy.init(args=args)
    node = DummyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()