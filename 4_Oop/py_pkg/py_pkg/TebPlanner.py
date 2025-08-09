from _interface import PlannerInterface
import numpy as np

class TebPlanner(PlannerInterface):
    def __init__(self,
                 PlannerName: str,
                 start: np.array,
                 goal: np.array
                 ):
        super().__init__(start, goal)
        self.name = PlannerName
        print("----------------")
        print(f"启用 {self.name} 接口")
        print("----------------")

if __name__ == "__main__":
    # 修正numpy数组的创建方式，使用圆括号
    Teb = TebPlanner("Teb", np.array([0.0, 1.0]), np.array([2.0, 2.0]))
    # 测试是否到达目标点
    print(f"距离小于1.0: {Teb.isGoalReached(1.0)}")
    print(f"距离小于3.0: {Teb.isGoalReached(3.0)}")
