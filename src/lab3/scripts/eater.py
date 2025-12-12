#!/usr/bin/python3

from lab2.dummy_module import dummy_function, dummy_var
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, Point
from turtlesim.msg import Pose
from turtlesim_plus_interfaces.srv import GivePosition
from std_srvs.srv import Empty
from std_msgs.msg import Int64, Bool
from controller_interfaces.srv import SetMaxPizza, SetParam
import numpy as np
import math

class DummyNode(Node):
    def __init__(self):
        super().__init__('eater_node')
        
        self.ns = self.get_namespace().strip("/")
        if self.ns == "":
            self.ns = "eater_turtle"

        self.declare_parameter("sampling_frequency", 100.0)
        freq = self.get_parameter("sampling_frequency").value

        self.kp_v = 2.0
        self.kp_omega = 4.0
        self.kp_omega_temp = 4.0
        self.kp_v_temp = 2.0

        self.pub_cmd = self.create_publisher(Twist, f"/{self.ns}/cmd_vel", 10)
        self.pub_eat_status = self.create_publisher(Bool, f"/{self.ns}/eat_status", 10)

        self.create_subscription(Pose, f"/{self.ns}/pose", self.pose_callback, 10)
        self.create_subscription(Point, "/mouse_position", self.mouse_callback, 10)
        self.create_subscription(Int64, f"/{self.ns}/pizza_count", self.pizza_count_callback, 10)

        self.spawn_pizza_client = self.create_client(GivePosition, "/spawn_pizza")
        self.eat_client = self.create_client(Empty, f"/{self.ns}/eat")

        self.create_service(SetMaxPizza, f"/{self.ns}/set_max_pizza", self.set_max_pizza_callback)
        self.create_service(SetParam, f"/{self.ns}/set_controller_param", self.set_param_callback)

        self.turtle_pos = np.array([0.0, 0.0, 0.0])
        self.mouse_pos = np.array([0.0, 0.0])
        self.goal_pos = np.array([0.0, 0.0, 0.0])

        self.max_pizza = 5
        self.pizza = []
        self.eaten_pizza = 0
        self.current_pizza = 0

        self.target = 0   # 1 = moving to click/goal
        self.eat = False      # 1 = eating pizza

        self.timer = self.create_timer(1.0 / freq, self.timer_callback)

    def set_max_pizza_callback(self, request, response):
        req_val = request.max_pizza
        self.max_pizza = req_val
        response.log = f"[SUCCESS] max pizza updated to {self.max_pizza}"
        return response

    def set_param_callback(self, request, response):
        self.kp_v_temp = request.kp_linear
        self.kp_omega_temp = request.kp_angular
        return response

    def pose_callback(self, msg):
        self.turtle_pos[:] = [msg.x, msg.y, msg.theta]

    def mouse_callback(self, msg):
        self.target = 1 
        self.mouse_pos[:] = [msg.x, msg.y]
        if self.current_pizza < self.max_pizza:
            self.current_pizza += 1
            self.spawn_pizza(self.mouse_pos[0], self.mouse_pos[1])
            self.pizza.append((self.mouse_pos[0], self.mouse_pos[1]))
            self.get_logger().info('spawn waiooo')

    def pizza_count_callback(self, msg: Int64):
        self.eaten_pizza = msg.data

    def spawn_pizza(self, x, y):
        req = GivePosition.Request()
        req.x = x
        req.y = y
        self.spawn_pizza_client.call_async(req)

    def move_turtle(self, v, w):
        msg = Twist()
        msg.linear.x = v
        msg.angular.z = w
        self.pub_cmd.publish(msg)

    def control(self, tx, ty):
        dx = tx - self.turtle_pos[0]
        dy = ty - self.turtle_pos[1]
        d = math.sqrt(dx**2 + dy**2)

        target_theta = math.atan2(dy, dx)
        theta_err = math.atan2(math.sin(target_theta - self.turtle_pos[2]),
                               math.cos(target_theta - self.turtle_pos[2]))

        v = self.kp_v * d
        w = self.kp_omega * theta_err
        self.move_turtle(v, w)
        if self.target == 1:
            if d < 0.5 and abs(theta_err) < 0.1:
                self.target = 0
        else:
            if d < 0.5 and abs(theta_err) < 0.1 and self.eat == True and len(self.pizza) != 0:
                self.eat_pizza()
                if len(self.pizza) <= 0 and self.eaten_pizza <= self.max_pizza:
                    self.eat = False

    def eat_pizza(self):
        req = Empty.Request()
        self.eat_client.call_async(req)

        if len(self.pizza) > 0:
            self.pizza.pop(0)

    def timer_callback(self):
        eat_status_msg = Bool()
        eat_status_msg.data = self.eaten_pizza < self.max_pizza
        self.get_logger().info(f"eat status: {eat_status_msg}")
        self.eat = eat_status_msg.data
        self.pub_eat_status.publish(eat_status_msg)

        if len(self.pizza) > 0 and self.eat == True:
            self.kp_omega = self.kp_omega_temp
            self.kp_v = self.kp_v_temp
            # self.get_logger().info(f'v = {self.kp_v},  w = {self.kp_omega},  pizza = {self.eaten_pizza}')
            self.control(self.pizza[0][0], self.pizza[0][1])

        elif self.eat == False and self.target == 0:        # when starting and no target (killer state)
            # self.get_logger().info("2")
            self.kp_v = 0.0
            self.kp_omega = 0.0
            self.control(self.mouse_pos[0], self.mouse_pos[1])

        elif self.target == 1:
            dx, dy = self.mouse_pos
            self.kp_omega = self.kp_omega_temp
            self.kp_v = self.kp_v_temp
            self.control(dx, dy)

        else:
            self.kp_omega = 0.0
            self.kp_v = 0.0
            self.control(0.0, 0.0)

def main(args=None):
    rclpy.init(args=args)
    node = DummyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()