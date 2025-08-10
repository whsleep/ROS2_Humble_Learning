import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

# 定义一个轨迹控制节点类
class TrajNode(Node):
    def __init__(self, node_name, goal_name):
        super().__init__(node_name)
        # 创建速度指令发布者，发布到乌龟的速度话题
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        # 初始化目标位置和当前位置
        self.goal_pose = None
        self.current_pose = None
        # 标志变量，用于标记是否第一次接收到current_pose
        self.is_first_pose_received = False
        # 创建目标位置订阅者，订阅目标位置话题
        self.subscription_goal = self.create_subscription(Pose, goal_name, self.goal_callback, 10)
        # 创建当前位置订阅者，订阅乌龟的位姿话题
        self.subscription_pose = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.get_logger().info("Starting motion control...")

    # 目标位置回调函数，接收目标位置消息
    def goal_callback(self, msg):
        self.goal_pose = msg
        self.calculate_and_publish()

    # 当前位置回调函数，接收当前位姿消息
    def pose_callback(self, msg):
        self.current_pose = msg
        # 第一次接收current_pose时，将其赋值给goal_pose
        if not self.is_first_pose_received:
            self.goal_pose = msg
            self.is_first_pose_received = True
            self.get_logger().info("First current_pose received and set as goal_pose.")
        self.calculate_and_publish()

    # 计算速度指令并发布
    def calculate_and_publish(self):
        if self.goal_pose is not None and self.current_pose is not None:
            # 计算目标位置与当前位置之间的差值
            delta_x = self.goal_pose.x - self.current_pose.x
            delta_y = self.goal_pose.y - self.current_pose.y
            delta_theta = self.goal_pose.theta - self.current_pose.theta

            # 创建并发布速度指令消息
            twist_msg = Twist()
            twist_msg.linear.x = delta_x/0.2;
            twist_msg.linear.y = delta_y/0.2;
            twist_msg.angular.z = delta_theta/0.2
            self.publisher.publish(twist_msg)

            # 打印当前状态
            self.get_logger().info(f"Moving towards goal: current_pose=({self.current_pose.x:.2f}, {self.current_pose.y:.2f}), goal_pose=({self.goal_pose.x:.2f}, {self.goal_pose.y:.2f})")

def main():
    rclpy.init()
    node = TrajNode("trajectory_node", "goal_pose")  # 创建轨迹控制节点
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()
