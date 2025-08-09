#include <string>
#include "rclcpp/rclcpp.hpp"
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

// 公共继承节点类
class Person_Node : public rclcpp::Node
{

private:
    string node_name;
    string PlannerName;
    Array<float, 2, 1> start;
    Array<float, 2, 1> goal;

public:
    Person_Node(
        const string &_nodename,
        const string &_PlannerName,
        Array<float, 2, 1> &_start,
        Array<float, 2, 1> &_goal) : Node(_nodename),
                                     PlannerName(_PlannerName),
                                     start(_start),
                                     goal(_goal)
    {
        this->node_name = _nodename;
    };

    bool isGoalReached(double r)
    {
        Array<float, 2, 1> difference = this->goal - this->start;
        double distance = difference.matrix().norm();
        if (distance < r)
        {
            RCLCPP_INFO(this->get_logger(), "Goal reached! Distance: %f", distance);
            return true;
        }
        RCLCPP_INFO(this->get_logger(), "Goal not reached. Distance: %f", distance);
        return false;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    Array<float, 2, 1> start(0, 0);
    Array<float, 2, 1> goal(5, 5);
    auto node = make_shared<Person_Node>("person_node", "AStarPlanner", start, goal);
    node->isGoalReached(1.0);
    rclcpp::spin(node);
    rclcpp::shutdown();
}