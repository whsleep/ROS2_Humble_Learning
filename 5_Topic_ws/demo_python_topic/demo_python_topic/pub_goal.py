import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose

class GoalPublisher(Node):
    def __init__(self):
        super().__init__('goal_publisher')
        # 创建一个发布者，发布目标点到/goal_pose话题
        self.publisher_ = self.create_publisher(Pose, '/goal_pose', 10)
        # 定时器，每秒发布一次目标点
        timer_period = 0.2
        self.timer = self.create_timer(timer_period, self.timer_callback)
        # 定义矩形轨迹的四个顶点
        self.waypoints = [
            Pose(x=8.0, y=8.0, theta=0.0),
            Pose(x=8.0, y=2.0, theta=0.0),
            Pose(x=2.0, y=2.0, theta=0.0),
            Pose(x=2.0, y=8.0, theta=0.0)
        ]
        # 当前目标点索引
        self.current_waypoint = 0

    def timer_callback(self):
        # 获取当前目标点
        current_goal = self.waypoints[self.current_waypoint]
        # 发布目标点
        self.publisher_.publish(current_goal)
        # 打印当前发布的目标点信息
        self.get_logger().info(f'Published goal: x={current_goal.x}, y={current_goal.y}')
        # 更新下一个目标点索引
        self.current_waypoint = (self.current_waypoint + 1) % len(self.waypoints)

def main(args=None):
    rclpy.init(args=args)
    goal_publisher = GoalPublisher()
    try:
        rclpy.spin(goal_publisher)
    except KeyboardInterrupt:
        goal_publisher.get_logger().info('Shutting down...')
    finally:
        goal_publisher.destroy_node()
        rclpy.shutdown()
