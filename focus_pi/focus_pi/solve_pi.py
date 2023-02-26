#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy                            # ROS2 Python接口库
from rclpy.node import Node             # ROS2 节点类
from sensor_msgs.msg import Image       # 图像消息类型
from cv_bridge import CvBridge          # ROS与OpenCV图像转换类
import cv2                              # Opencv图像处理库
import numpy as np                      # Python数值计算库
from std_msgs.msg import Int32

"""
创建一个订阅者节点
"""
class ImageSubscriber(Node):
    def __init__(self, name):
        super().__init__(name)                                  # ROS2节点父类初始化
        self.sub = self.create_subscription(
            Image, 'image_raw', self.listener_callback, 10)     # 创建订阅者对象（消息类型、话题名、订阅者回调函数、队列长度）
        self.cv_bridge = CvBridge()                             # 创建一个图像转换对象，用于OpenCV图像与ROS的图像消息的互相转换
        self.pub = self.create_publisher(
            Int32, 'contour', 10 )        

    def solvin(self, img):
        im =cv2.cvtColor(img, cv2.COLOR_BGR2GRAY).astype(np.float64)
        blurred = cv2.GaussianBlur(im, (49,49), 5)  # 对图片用高斯模糊
        blurred_sq = blurred * blurred
        im2d = im * im
        sigma = np.sqrt(abs(cv2.GaussianBlur(im2d, (49,49), 5) - blurred_sq))/255

        structdis_1 = (im - blurred) / (sigma+1)
        structdis_1 = structdis_1 / np.max(structdis_1)
        structdis_1 = structdis_1 + abs(np.min(structdis_1))
        structdis_2 = structdis_1 / np.max(structdis_1)
        gray_img = np.array(structdis_2 * 255).astype('uint8')
        
        _, thresh = cv2.threshold(gray_img, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
#         h = gray_img.shape[0]
#         w = gray_img.shape[1]
#         # 遍历灰度层
#         for i in range(h):
#             for j in range(w):
#                 if gray_img[i][j]<130:
#                     gray_img[i][j] = 0
#                 else:
#                     gray_img[i][j] = 255
#         self.contours, hierarchy = cv2.findContours(gray_img, cv2.RETR_LIST, 2)
        self.contours, hierarchy = cv2.findContours(thresh, cv2.RETR_LIST, 2)

    def listener_callback(self, data):
        self.get_logger().info('Receiving video frame')         # 输出日志信息，提示已进入回调函数
        image = self.cv_bridge.imgmsg_to_cv2(data, 'bgr8')      # 将ROS的图像消息转化成OpenCV图像
        self.solvin(image)                               # 苹果检测
        data = self.contours
        self.pub.publish(data)


def main(args=None):                                        # ROS2节点主入口main函数
    rclpy.init(args=args)                                   # ROS2 Python接口初始化
    node = ImageSubscriber("solve_pi")              # 创建ROS2节点对象并进行初始化
    rclpy.spin(node)                                        # 循环等待ROS2退出
    node.destroy_node()                                     # 销毁节点对象
    rclpy.shutdown()                                        # 关闭ROS2 Python接口
