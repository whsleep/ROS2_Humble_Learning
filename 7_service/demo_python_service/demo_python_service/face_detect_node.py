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

class FaceDetectNode(Node):
    def __init__(self):
        super().__init__('face_detect_node')
        # 创建服务
        self.srv = self.create_service(FaceDetector, 'face_detect', self.face_detect_callback)
        self.bridge = CvBridge()
        self.get_logger().info("Face Detect Service is Ready.")

    def face_detect_callback(self, request, response):
        # 判断图像是否为空
        if request.image.data:
            cv_image = self.bridge.imgmsg_to_cv2(request.image)
        else:
            resource_path = os.path.join(
                get_package_share_directory('demo_python_service'), 'resource/zidane.jpg'
            )
            cv_image = cv2.imread(resource_path)
            self.get_logger().info('request 图像为空，使用默认图像')
        self.get_logger().info('图片加载完成,开始检测人脸...')
        star_time = time.time()
        # 检测人脸
        face_location = face_recognition.face_locations(cv_image,number_of_times_to_upsample=1,model="hog")
        response.use_time = time.time() - star_time
        self.get_logger().info(f'人脸检测完成,耗时 {response.use_time:.4f} 秒')
        # 人脸数量
        response.number = len(face_location)
        self.get_logger().info(f'检测到 {response.number} 张人脸')
        # 记录复选框
        for (top, right, bottom, left) in face_location:
            response.top.append(top)
            response.right.append(right)
            response.bottom.append(bottom)
            response.left.append(left)
        self.get_logger().info('检测结果返回完成')
        return response



def main():
    rclpy.init()
    node = FaceDetectNode()
    rclpy.spin(node)
    rclpy.shutdown()