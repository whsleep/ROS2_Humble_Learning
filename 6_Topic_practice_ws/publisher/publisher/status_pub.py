import rclpy
from rclpy.node import Node
from interfaces.msg import SystemStatus # 导入消息接口
import psutil
import platform

class SysStatusPub(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        # 创建发布者
        self.status_publisher_ = self.create_publisher(
            SystemStatus, 'sys_status', 10)
        # 创建定时器1s回调一次
        self.timer = self.create_timer(1, self.timer_callback)

    def timer_callback(self):
        # 获取cpu使用率
        cpu_percent = psutil.cpu_percent()
        # 获取系统内存信息
        memory_info = psutil.virtual_memory()
        # 获取网络信息
        net_io_counters = psutil.net_io_counters()

        msg = SystemStatus()
        msg.stamp = self.get_clock().now().to_msg()
        msg.host_name = platform.node()
        msg.cpu_percent = cpu_percent
        msg.memory_percent = memory_info.percent
        msg.memory_total = memory_info.total / 1024 / 1024
        msg.memory_available = memory_info.available / 1024 / 1024
        msg.net_sent = net_io_counters.bytes_sent / 1024 / 1024
        msg.net_recv = net_io_counters.bytes_recv / 1024 / 1024

        self.get_logger().info(f'发布:{str(msg)}')
        self.status_publisher_.publish(msg)


def main():
    rclpy.init()
    node = SysStatusPub('sys_status_pub')
    rclpy.spin(node)
    rclpy.shutdown()