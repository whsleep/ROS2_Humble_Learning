import rclpy
from rclpy.node import Node
from face_service_type.srv import FaceDetector

import face_recognition
import cv2
# 获取 share 目录绝对路径
from ament_index_python.packages import get_package_share_directory
import os
from cv_bridge import CvBridge
import time

class FaceDetectClientNode(Node):
    def __init__(self):
        super().__init__('face_detect_client_node')
        self.resource_path = os.path.join(
                get_package_share_directory('demo_python_service'), 'resource/s_l.jpg'
            )
        self.bridge = CvBridge()
        self.client = self.create_client(FaceDetector, 'face_detect')
        self.image = cv2.imread(self.resource_path)
        self.get_logger().info("Face Detect Client is Ready.")

    def send_request(self):
        # 判断服务端是否在线
        while self.client.wait_for_service(timeout_sec=1.0) is False:
            self.get_logger().info('等待服务端上线...')
        # 构造 request
        request = FaceDetector.Request()
        request.image = self.bridge.cv2_to_imgmsg(self.image)
        self.get_logger().info('客户端发送请求...')
        # 发送request并等待响应
        future = self.client.call_async(request)
        def result_callback(future):
            try:
                response = future.result()
            except Exception as e:
                self.get_logger().error(f'Service call failed: {e}')
            else:
                self.get_logger().info('收到服务端响应')
                self.show_response(response)

        future.add_done_callback(result_callback)
    
    def show_response(self, response):
        self.get_logger().info(f'检测到 {response.number} 张人脸')
        self.get_logger().info(f'检测耗时 {response.use_time:.4f} 秒')
        for i in range(response.number):
            self.get_logger().info(f'第 {i+1} 张人脸位置: top={response.top[i]}, right={response.right[i]}, bottom={response.bottom[i]}, left={response.left[i]}')
        # 绘制图像
        for i in range(response.number):
            top = response.top[i]
            right = response.right[i]
            bottom = response.bottom[i]
            left = response.left[i]
            cv2.rectangle(self.image, (left, top), (right, bottom), (0, 255, 0), 2)
        cv2.imshow('Face Detection', self.image)
        cv2.waitKey(0)

def main():
    rclpy.init()
    node = FaceDetectClientNode()
    node.send_request()
    rclpy.spin(node)
    rclpy.shutdown()