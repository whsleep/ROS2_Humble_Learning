import face_recognition
import cv2
# 获取 share 目录绝对路径
from ament_index_python.packages import get_package_share_directory
import os

def main():
    # 获取资源文件路径
    resource_path = os.path.join(
            get_package_share_directory('demo_python_service'), 'resource/zidane.jpg'
        )
    # 打印真实路径
    print(resource_path)
    # 读取图片
    img = cv2.imread(resource_path)
    # 检测人脸
    face_location = face_recognition.face_locations(img,number_of_times_to_upsample=1,model="hog")
    # 绘制复选框
    for (top, right, bottom, left) in face_location:
        cv2.rectangle(img, (left, top), (right, bottom), (255, 0, 0), 2)
    # 显示图片
    cv2.imshow("face detect", img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()