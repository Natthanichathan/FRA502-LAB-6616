#!/usr/bin/python3

from lab2.dummy_module import dummy_function, dummy_var
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from turtlesim.msg import Pose 
from turtlesim_plus_interfaces.srv import GivePosition
from std_srvs.srv import Empty, SetBool
import numpy as np
import math
from geometry_msgs.msg import PoseStamped
from turtlesim.srv import Kill , Spawn

class DummyNode(Node):
    def __init__(self):
        super().__init__("killer_node")
        self.publisher_kub = self.create_publisher(Twist, "/turtle2/cmd_vel", 10)

        self.spawn_turtle_client = self.create_client(Spawn, "/spawn_turtle")
        self.kill_turtle_client = self.create_client(Kill, "/remove_turtle")

        self.state_service = self.create_service(SetBool, '/killer_state', self.killer_state_callback)
        self.spawn_service = self.create_service(SetBool, '/spawn_killer', self.spawn_killer_callback)

        self.create_subscription(Pose, "/turtle1/pose", self.eater_pose_callback, 10)
        self.eater_pos = np.array([0.0, 0.0, 0.0])

        self.create_subscription(Pose, "/turtle2/pose", self.killer_pose_callback, 10)
        self.killer_pos = np.array([0.0, 0.0, 0.0])
        
        self.create_timer(0.01, self.timer_callback)
        
        self.kp_v = 2.0
        self.kp_omega = 4.0
        self.hunt = False

    def spawn_killer_callback(self, request, response):
        spawn_req = Spawn.Request()
        spawn_req.name = "turtle2"
        spawn_req.x = 1.0
        spawn_req.y = 1.0
        spawn_req.theta = 0.0

        self.spawn_turtle_client.call_async(spawn_req)

        response.success = True
        response.message = "turtle2 spawned successfully"
        return response

    def killer_state_callback(self, request, response):
        if request.data:
            self.hunt = True
            response.success = True
            response.message = "Entering hunt mode"
        else:
            self.hunt = False
            response.success = True
            response.message = "Staying in idle mode"
        return response

    def eater_pose_callback(self, msg): # msg from topic /turtle1/pose
        self.eater_pos[:] = [msg.x, msg.y, msg.theta]

    def killer_pose_callback(self, msg): # msg from topic /turtle2/pose
        self.killer_pos[:] = [msg.x, msg.y, msg.theta]

    def hunt_callback(self, msg):
        if msg.data == True:
            self.hunt = True

    def control(self, x, y):
        d_x = x - self.killer_pos[0]
        d_y = y - self.killer_pos[1]
        d = math.sqrt(d_x**2 + d_y**2)

        target_theta = math.atan2(d_y, d_x)
        theta = target_theta - self.killer_pos[2]
        theta = math.atan2(math.sin(theta), math.cos(theta))

        v = self.kp_v * d
        wz = self.kp_omega * theta
        self.move_turtle(v, wz)

    def move_turtle(self, v, w):
        msg = Twist()
        msg.linear.x = v
        msg.angular.z = w
        self.publisher_kub.publish(msg)

    def timer_callback(self):
        if self.hunt:
            self.control(self.eater_pos[0], self.eater_pos[1])

            # Kill condition
            if abs(self.eater_pos[0] - self.killer_pos[0]) < 0.5 and \
               abs(self.eater_pos[1] - self.killer_pos[1]) < 0.5:

                self.kill_turtle_client.call_async(Kill.Request(name="turtle1"))
        if not self.hunt:
            self.move_turtle(0.0, 0.0)


def main(args=None):
    rclpy.init(args=args)
    node = DummyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()