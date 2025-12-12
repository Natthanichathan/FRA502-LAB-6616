#!/usr/bin/python3

from imu_calibration.dummy_module import dummy_function, dummy_var
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from ament_index_python import get_package_share_directory
from sensor_msgs.msg import Imu
from sensor_interface.msg import SensorParameter
import numpy as np
import os
import yaml


class DummyNode(Node):
    def __init__(self):
        super().__init__('imu_calib_node')
        self.declare_parameter('file', 'imu_calib_data.yaml')
        pkg_name = 'imu_calibration'

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )

        self.create_subscription(
            SensorParameter,
            'sensor_parameter_publisher',
            self.imu_callback,
            qos_profile
        )

        imu_calib_pkg_share_path = get_package_share_directory(pkg_name)
        ws_path, _ = imu_calib_pkg_share_path.split('install')

        file = self.get_parameter('file').value
        self.imu_calib_path = os.path.join(ws_path, 'src', pkg_name, 'config', file)

        self.n = 0
        self.n_max = 10000
        self.accel_list = []
        self.gyro_list = []
        self.get_logger().info(f'IMU Calibration Node has been started.')

    def save_calibration(self, mean, cov, name : str):

        with open(self.imu_calib_path, 'r') as file:
            value = yaml.safe_load(file) or {}

        mean_list = mean.tolist()
        covariance_list = cov.tolist()

        value[f'{name} offset'] = mean_list
        value[f'{name} covariance'] = covariance_list

        with open(self.imu_calib_path, 'w') as file:
            yaml.dump(value, file)

    def imu_callback(self, msg: SensorParameter):
        imu: Imu = msg.imu_mpu6050
        if self.n < self.n_max:
            self.accel_list.append([
                imu.linear_acceleration.x,
                imu.linear_acceleration.y,
                imu.linear_acceleration.z - 9.81
            ])
            
            self.gyro_list.append([
                imu.angular_velocity.x,
                imu.angular_velocity.y,
                imu.angular_velocity.z
            ])

            self.n += 1
            print("collect data: ", self.n)

        else:
            accel_array = np.array(self.accel_list)
            accel_offset = np.mean(accel_array, 0)
            accel_cov = np.absolute(np.cov(accel_array.T))

            gyro_array = np.array(self.gyro_list)
            gyro_offset = np.mean(gyro_array, 0)
            gyro_cov = np.absolute(np.cov(gyro_array.T))               


            self.save_calibration(accel_offset, accel_cov, 'accel')
            self.save_calibration(gyro_offset, gyro_cov, 'gyro')

            print("==============================")
            print(accel_offset)
            print(accel_cov)
            print("------------------------------")
            print(gyro_offset)
            print(gyro_cov)
            exit()

def main(args=None):
    rclpy.init(args=args)
    node = DummyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
