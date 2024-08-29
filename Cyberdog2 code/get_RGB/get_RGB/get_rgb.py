# 导入必要的库
import sys
import os
import time
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import rclpy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import numpy as np

# 导入自定义模块 robot_control_cmd_lcmt
sys.path.append('/home/cyberdog_sim/src/cyberdog_simulator/motion')
from robot_control_cmd_lcmt import robot_control_cmd_lcmt


# 创建图像处理类
class ImageHandler:
    def __init__(self, node):
        self.node = node
        self.bridge = CvBridge()
        self.image_received = False
        self.current_image = None
        self.timer_period = 2
        self.image_counter = 0
        self.last_order = None

    # 图像回调函数,处理接收到的图像消息
    def image_callback(self, msg):
        self.current_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.image_received = True
        self.detect_color_positions(self.current_image)


    # 检测颜色分布的方法
    def detect_color_positions(self, image):

        # 定义颜色范围和对应的数值
        color_ranges = {
            'red': ([0, 50, 50], [10, 255, 255], 111111),
            'pink': ([140, 50, 50], [170, 255, 255], 0),
            'yellow': ([20, 100, 100], [174, 255, 255], 3333),
            'green': ([30, 77, 77], [85, 255, 255], 22),
            'blue': ([100, 50, 50], [140, 255, 255], 55),
            'gray': ([0, 0, 80], [255, 50, 220], 2),
            'black': ([0, 0, 0], [99, 99, 90], 8),  # 修正了 'blcak' 为 'black'
        }



        #萨维塔算法：
        # 定义每个区域的权重值
        left_weight = 7
        middle_weight = 19
        right_weight = 23

        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        height, width, _ = image.shape
        third_width = width // 3

        # 初始化每个区域的颜色值总和
        color_sums = [0, 0, 0]

        for i in range(3):  # 循环处理左、中、右区域
            section = hsv_image[:, i*third_width:(i+1)*third_width, :]
            colors_found = []

            for color_name, (lower_color, upper_color, value) in color_ranges.items():
                mask = cv2.inRange(section, np.array(lower_color), np.array(upper_color))
                if np.count_nonzero(mask) > 0:  # 检查该颜色是否在当前区域存在
                    colors_found.append(value)

            # 累加当前区域的颜色值
            color_sums[i] = sum(colors_found)

        # 计算最终结果
        res = (left_weight * color_sums[0] +
               middle_weight * color_sums[1] +
               right_weight * color_sums[2])

        # 输出左、中、右三个区域的颜色信息和最终结果
        #print(f"左边区域颜色值总和: {color_sums[0]}, 中间区域颜色值总和: {color_sums[1]}, 右边区域颜色值总和: {color_sums[2]}")
        # print(f"Result: {res}")

        # condition judge
        if res == 490:
            mm = 0
        elif res ==164885:
            mm = 1
        elif res ==2275961:
            mm = 2
        elif res ==166502:
            mm = 3
        elif res == 3053265:
            mm = 4
        elif res == 3053771:
            mm = 5
        elif 4660000 <= res <= 4670000:
             # 定义绿色范围
            lower_green = np.array([30, 70, 70])
            upper_green = np.array([85, 255, 255])
            # 创建掩模
            mask = cv2.inRange(hsv_image, lower_green, upper_green)
            # 将图像分割为左右两半
            height, width = mask.shape
            left_half = mask[:, :width // 2]
            right_half = mask[:, width // 2:]
            # 计算左右两半的绿色像素数
            left_green_count = np.sum(left_half > 0)
            right_green_count = np.sum(right_half > 0)
            # 计算左右两半的绿色像素比例
            total_pixels = mask.size
            left_green_ratio = left_green_count / total_pixels
            right_green_ratio = right_green_count / total_pixels
            if left_green_ratio > right_green_ratio:
                mm = 7
            elif right_green_ratio > left_green_ratio:
                mm = 8
            else:
                mm = "wrong"
        else:
            mm = res
            # 给定的一组值
            values = [490, 164885, 2275961, 166502, 3053265]

            # 找到与 mm 最接近的值
            closest_value = min(values, key=lambda x: abs(mm - x))

            mm = closest_value

            # # 判断是偏大还是偏小
            # if mm > closest_value:
            #     mm = "left"
            # elif mm < closest_value:
            #     mm = "right"


# ...[其他代码]...

        # 发布颜色信息
        if mm!=self.last_order:
            color_publisher = self.node.create_publisher(String, 'color_positions', 10)
            msg = String()
            msg.data = str(mm)
            color_publisher.publish(msg)
            print(mm)
            self.last_order = mm

    # 定时器回调函数
    def timer_callback(self):
        if self.image_received:
            # self.display_image(self.current_image, "图像")
            self.image_received = False

    # 显示图像
    # def display_image(self, image, window_name="图像"):
    #     cv2.imshow(window_name, image)
    #     cv2.waitKey(1000)
    #     cv2.destroyAllWindows()

# 主函数
def main():
    rclpy.init(args=None)
    node = rclpy.create_node('get_rgb')
    image_handler = ImageHandler(node)

    # 设置QoS配置
    qos_profile = QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=10
    )

    # 创建图像订阅
    subscription = node.create_subscription(
        Image,
        '/rgb_camera/image_raw',
        image_handler.image_callback,
        qos_profile
    )

    # 创建定时器
    timer = node.create_timer(image_handler.timer_period, image_handler.timer_callback)

    # 等待获取第一张图像
    while not image_handler.image_received:
        rclpy.spin_once(node)

    rclpy.spin(node)

    node.destroy_timer(timer)
    node.destroy_subscription(subscription)
    rclpy.shutdown()

if __name__ == '__main__':
    main()