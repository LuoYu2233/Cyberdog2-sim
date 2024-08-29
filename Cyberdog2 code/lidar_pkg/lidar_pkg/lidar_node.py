#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32  # 导入Float32消息类型
from rclpy.qos import qos_profile_sensor_data, QoSProfile

class LidarNode(Node):
    def __init__(self):
        super().__init__('lidar_node')
        qos = qos_profile_sensor_data  # 预定义的QoS配置之一
        self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, qos)
        # 添加两个发布者，话题分别为lidar_distance_90和lidar_distance_0
        self.distance_pub_90 = self.create_publisher(Float32, 'lidar_distance_90', 10)
        self.distance_pub_0 = self.create_publisher(Float32, 'lidar_distance_0', 10)

    def lidar_callback(self, msg):
        # 获取第91度和第1度的距离值
        dist_90 = msg.ranges[90]
        dist_0 = msg.ranges[0]
        # 创建两个Float32类型的消息
        distance_message_90 = Float32()
        distance_message_0 = Float32()
        # 将距离值赋给消息的data字段
        distance_message_90.data = dist_90
        distance_message_0.data = dist_0
        # 发布距离消息
        self.distance_pub_90.publish(distance_message_90)
        self.distance_pub_0.publish(distance_message_0)
        # # 如果需要，可以在控制台中打印距离信息
        # self.get_logger().info("90度测距 = {} 米".format(dist_90))
        # self.get_logger().info("0度测距 = {} 米".format(dist_0))

def main(args=None):
    rclpy.init(args=args)
    lidar_node = LidarNode()
    rclpy.spin(lidar_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()