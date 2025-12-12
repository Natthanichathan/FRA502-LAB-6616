#!/usr/bin/python3

from lab2.dummy_module import dummy_function, dummy_var
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, Point
from turtlesim.msg import Pose
from turtlesim_plus_interfaces.srv import GivePosition
from std_srvs.srv import Empty, SetBool
from std_msgs.msg import Int64
import numpy as np
import math

class DummyNode(Node):
    def __init__(self):
        super().__init__('eater_node')
        
        self.publisher_kub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pub_pizza_eaten = self.create_publisher(Int64, '/turtle1/pizza_count', 10)
        
        # have tested that works
        self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.turtle_pos = np.array( [0.0, 0.0, 0.0] ) # x, y, theta of turtle for storing position

        # have tested that works
        self.create_subscription(Point, '/mouse_position', self.mouse_position_callback, 10)
        self.mouse_pos = np.array( [0.0, 0.0] ) # x, y of mouse click position

        # have tested that works
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_pose_callback, 10)
        self.goal_position = np.array( [0.0, 0.0, 0.0] ) # x, y, theta of goal pose

        # have tested that works
        self.spawn_pizza_client = self.create_client(GivePosition, '/spawn_pizza') #create client called 'spawn_pizza'

        self.create_subscription(Int64, '/set_max_pizza', self.set_max_pizza_callback, 10)
        self.max_pizza = 5
        self.pizza = []  # list to store pizza positions
        self.current_pizza = 0
        self.eaten_pizza = 0

        self.pizza_eat_client = self.create_client(Empty, '/turtle1/eat')

        self.killer_state_client = self.create_client(SetBool, '/killer_state')

        timer_period = 0.01  # seconds
        self.create_timer(timer_period, self.timer_callback)

        self.target = 0
        self.eat = 0
        self.kp_v = 0.0
        self.kp_omega = 0.0

    def move_turtle(self, v, w):
        msg = Twist()
        msg.linear.x = v
        msg.angular.z = w
        self.publisher_kub.publish(msg)

    def send_state_to_killer(self, finished):
        req = SetBool.Request()
        req.data = finished
        self.killer_state_client.call_async(req)

    def set_max_pizza_callback(self, msg):
        self.max_pizza = msg.data
        # print(f"Max pizza set to: {self.max_pizza}")

    def pose_callback(self, msg): # msg from topic /turtle1/pose
        if msg.x == self.turtle_pos[0] and msg.y == self.turtle_pos[1] and msg.theta == self.turtle_pos[2]:
            return
        self.turtle_pos[0] = msg.x
        self.turtle_pos[1] = msg.y
        self.turtle_pos[2] = msg.theta
        # self.get_logger().info(f"Position : {self.turtle_position}")

    def mouse_position_callback(self, msg):
        self.target = 1
        self.mouse_pos[0] = msg.x
        self.mouse_pos[1] = msg.y
        if self.current_pizza < self.max_pizza:
            self.current_pizza += 1
            self.Spawn_pizza(self.mouse_pos[0], self.mouse_pos[1])
            self.pizza.append((self.mouse_pos[0], self.mouse_pos[1]))
        # print(f"Mouse position: x={msg.x}, y={msg.y}")

    def goal_pose_callback(self, msg):
        self.target = 1
        self.goal_position[0] = (10.88000012/10)*(msg.pose.position.x + 5)
        self.goal_position[1] = (10.88000012/10)*(msg.pose.position.y + 5)
        if self.current_pizza < self.max_pizza:
            self.current_pizza += 1
            self.Spawn_pizza(self.goal_position[0], self.goal_position[1])
            self.pizza.append((self.goal_position[0], self.goal_position[1]))
        # print(f"Goal position: x={self.goal_position[0]}, y={self.goal_position[1]}")

    def Spawn_pizza(self, X, Y):
        position_request = GivePosition.Request() #create request object (client -> request, server -> response)
        position_request.x = X
        position_request.y = Y
        self.spawn_pizza_client.call_async(position_request) #send request to server

    def control(self,x,y):
        d_x = x - self.turtle_pos[0]
        d_y = y - self.turtle_pos[1]
        d = math.sqrt(d_x**2 + d_y**2)

        target_theta = math.atan2(d_y, d_x)
        theta = target_theta - self.turtle_pos[2]
        theta = math.atan2(math.sin(theta), math.cos(theta))

        v = self.kp_v * d
        wz = self.kp_omega * theta
        self.move_turtle(v, wz)
        if self.target == 1:
            if d < 0.5 and abs(theta) < 0.1:
                self.target = 0
        else:
            if d < 0.5 and abs(theta) < 0.1 and self.eat == 1 and len(self.pizza) != 0:
                self.eat_pizza()
                if len(self.pizza) <= 0 and self.eaten_pizza == self.max_pizza:
                    self.eat = 0
                    self.get_logger().info('eat = 0')   
                    self.send_state_to_killer(True)

    def eat_pizza(self):
        eat_request = Empty.Request()
        self.pizza_eat_client.call_async(eat_request)
        if len(self.pizza) > 0:
            self.pizza.pop(0)
            self.eaten_pizza += 1

        pizza_count_msg = Int64()
        pizza_count_msg.data = self.eaten_pizza
        self.pub_pizza_eaten.publish(pizza_count_msg)

            # self.get_logger().info(f"[EAT] Pizza eaten: {self.eaten_pizza}")

        # if temp != self.pizza[0] and len(self.pizza) != 0:
        #     self.eaten_pizza += 1

    def timer_callback(self):
        if len(self.pizza) > 0:
            self.eat = 1
            self.send_state_to_killer(False)
        if self.eat == 1 and len(self.pizza) > 0:
            # self.get_logger().info("1")
            self.kp_omega = 4.0
            self.kp_v = 2.0
            self.control(self.pizza[0][0], self.pizza[0][1])
        elif self.eat == 0 and self.target == 0:        # when starting and no target (killer state)
            # self.get_logger().info("2")
            self.kp_v = 0.0
            self.kp_omega = 0.0
            self.control(self.mouse_pos[0], self.mouse_pos[1])
        elif self.target == 1:
            # self.get_logger().info("3")
            self.kp_omega = 4.0
            self.kp_v = 2.0
            self.control(self.mouse_pos[0], self.mouse_pos[1])
        else:
            # self.get_logger().info("4")
            self.kp_omega = 0.0
            self.kp_v = 0.0
            self.control(self.mouse_pos[0], self.mouse_pos[1])

def main(args=None):
    rclpy.init(args=args)
    node = DummyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()