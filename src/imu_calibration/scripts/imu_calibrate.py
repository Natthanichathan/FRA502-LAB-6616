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
        super().__init__('imu_calibrate_node')
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

        # publish IMU หลัง calibrated
        self.pub_calib = self.create_publisher(
            SensorParameter, 
            'calibrated_parameter', 
            qos_profile
        )

        imu_calib_pkg_share_path = get_package_share_directory(pkg_name)
        ws_path, _ = imu_calib_pkg_share_path.split('install')

        file = self.get_parameter('file').value
        self.imu_calib_path = os.path.join(ws_path, 'src', pkg_name, 'config', file)

        self.get_logger().info(f'loading .yamlrc configuration file : {self.imu_calib_path}')

        with open(self.imu_calib_path, 'r') as file:
            calib = yaml.safe_load(file)

        self.accel_offset = np.array(calib['accel offset'])
        self.gyro_offset  = np.array(calib['gyro offset'])
        self.accel_cov    = np.array(calib['accel covariance'])
        self.gyro_cov     = np.array(calib['gyro covariance'])

        self.get_logger().info(f'Accel offset: {self.accel_offset}')
        self.get_logger().info(f'Gyro offset : {self.gyro_offset}')
    
    def imu_callback(self, msg: SensorParameter):
        imu_raw: Imu = msg.imu_mpu6050

        # raw data
        accel_raw = np.array([
            imu_raw.linear_acceleration.x,
            imu_raw.linear_acceleration.y,
            imu_raw.linear_acceleration.z
        ])

        gyro_raw = np.array([
            imu_raw.angular_velocity.x,
            imu_raw.angular_velocity.y,
            imu_raw.angular_velocity.z
        ])

        # calibrated data
        accel_calib = accel_raw - self.accel_offset
        gyro_calib  = gyro_raw  - self.gyro_offset

        self.get_logger().info(f'Calibrated Accel: {accel_calib}\nGyro: {gyro_calib}')

        # Publish calibrated IMU
        imu_msg = Imu()
        imu_msg.header = imu_raw.header

        imu_msg.linear_acceleration.x = accel_calib[0]
        imu_msg.linear_acceleration.y = accel_calib[1]
        imu_msg.linear_acceleration.z = accel_calib[2] # -9.81  # if you want to remove gravity effect

        imu_msg.angular_velocity.x = gyro_calib[0]
        imu_msg.angular_velocity.y = gyro_calib[1]
        imu_msg.angular_velocity.z = gyro_calib[2]
        
        # define encoder message
        sensor_msg = SensorParameter()
        sensor_msg.encoder1 = msg.encoder1
        sensor_msg.encoder2 = msg.encoder2
        sensor_msg.imu_mpu6050 = imu_msg

        self.pub_calib.publish(sensor_msg)


def main(args=None):
    rclpy.init(args=args)
    node = DummyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
