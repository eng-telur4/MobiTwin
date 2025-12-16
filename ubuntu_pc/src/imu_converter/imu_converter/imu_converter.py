#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
from rclpy.qos import qos_profile_sensor_data

class ImuConverter(Node):
    def __init__(self):
        super().__init__('imu_converter')
        self.subscriber = self.create_subscription(
            Imu,
            '/livox/imu',  # 元のIMUデータのトピック
            self.listener_callback,
            qos_profile_sensor_data
        )
        self.publisher = self.create_publisher(
            Imu,
            '/imu_converted',  # 変換後のIMUデータのトピック
            qos_profile_sensor_data
        )

    def listener_callback(self, msg):
        # 加速度をgからm/s²に変換
        conversion_factor = 9.81
        converted_msg = Imu()

        # ヘッダー情報をコピー
        converted_msg.header = msg.header

        # ジャイロスコープデータはそのままコピー
        converted_msg.angular_velocity = msg.angular_velocity

        # 加速度データを変換
        converted_msg.linear_acceleration = Vector3(
            x=msg.linear_acceleration.x * conversion_factor,
            y=msg.linear_acceleration.y * conversion_factor,
            z=msg.linear_acceleration.z * conversion_factor
        )

        # 姿勢情報もそのままコピー（必要に応じて変更）
        converted_msg.orientation = msg.orientation

        # 変換したIMUデータをパブリッシュ
        self.publisher.publish(converted_msg)
        #self.get_logger().info('Published converted IMU data')

def main(args=None):
    rclpy.init(args=args)
    imu_converter = ImuConverter()
    rclpy.spin(imu_converter)
    imu_converter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()