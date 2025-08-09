import rclpy
from rclpy.node import Node
import numpy as np

class Person_Node(Node):
    def __init__(self, node_name, 
                 PlannerName: str,
                 start: np.array,
                 goal: np.array):
        super().__init__(node_name)
        self.name = PlannerName
        self.start = start
        self.goal = goal 
        print("----------------")
        print(f"启用 {self.get_node_names()} 节点")
        print(f"包含 {self.name} 规划器")
        print("----------------")
    """
    判断是否抵达目标点
    """
    def isGoalReached(self, r: float) -> bool:
        # 计算欧氏距离
        distance = np.linalg.norm(self.start - self.goal)
        if distance < r:
            self.get_logger().info(f"距离小于 {r}")
            return True
        self.get_logger().info(f"距离大于 {r}")
        return False
    
def main():
    rclpy.init()
    node = Person_Node("P_Node","Teb",np.array([0.0, 1.0]), np.array([2.0, 2.0]))
    node.isGoalReached(1.0)
    rclpy.spin(node)
    rclpy.shutdown()