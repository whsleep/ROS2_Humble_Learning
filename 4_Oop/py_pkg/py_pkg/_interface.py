import numpy as np

class PlannerInterface:
    """
    规划器基类接口
    """
    def __init__(self, start: np.array, goal: np.array):
        self.name = "PlannerInterface"  # 定义name属性
        self.start = start
        self.goal = goal
        print("----------------")
        print(f"启用规划器基类接口: {self.name}")
        # 修正打印方式，使用f-string格式化输出
        print(f"start: {self.start}, goal: {self.goal}")
        print("----------------")

    """
    判断是否抵达目标点
    """
    def isGoalReached(self, r: float) -> bool:
        # 计算欧氏距离
        distance = np.linalg.norm(self.start - self.goal)
        if distance < r:
            return True
        return False
    
if __name__ == "__main__":
    # 修正numpy数组的创建方式，使用圆括号
    dwa = PlannerInterface(np.array([0.0, 1.0]), np.array([2.0, 2.0]))
    # 测试是否到达目标点
    print(f"距离小于1.0: {dwa.isGoalReached(1.0)}")
    print(f"距离小于3.0: {dwa.isGoalReached(3.0)}")