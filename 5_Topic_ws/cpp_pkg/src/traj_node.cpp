#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <turtlesim/msg/pose.hpp>

using namespace std::placeholders;

class TrajNode : public rclcpp::Node
{
public:
    TrajNode() : Node("trajectory_node")
    {
        // 创建速度指令发布者
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
        // 创建目标位置订阅者
        subscription_goal_ = this->create_subscription<turtlesim::msg::Pose>("/goal_pose", 10,
                                                                             std::bind(&TrajNode::goal_callback, this, _1));
        // 创建当前位置订阅者
        subscription_pose_ = this->create_subscription<turtlesim::msg::Pose>("/turtle1/pose", 10,
                                                                             std::bind(&TrajNode::pose_callback, this, _1));
        // 初始化目标位置和当前位置
        goal_pose_ = nullptr;
        current_pose_ = nullptr;
        // 标志变量，用于标记是否第一次接收到current_pose
        is_first_pose_received_ = false;
        // 定时器，用于定期计算和发布速度指令
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&TrajNode::calculate_and_publish, this));
        RCLCPP_INFO(this->get_logger(), "Starting motion control...");
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscription_goal_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscription_pose_;
    turtlesim::msg::Pose::SharedPtr goal_pose_;
    turtlesim::msg::Pose::SharedPtr current_pose_;
    bool is_first_pose_received_;
    rclcpp::TimerBase::SharedPtr timer_;

    void goal_callback(const turtlesim::msg::Pose::SharedPtr msg)
    {
        goal_pose_ = msg;
    }

    void pose_callback(const turtlesim::msg::Pose::SharedPtr msg)
    {
        current_pose_ = msg;
        // 第一次接收current_pose时，将其赋值给goal_pose
        if (!is_first_pose_received_)
        {
            goal_pose_ = msg;
            is_first_pose_received_ = true;
            RCLCPP_INFO(this->get_logger(), "First current_pose received and set as goal_pose.");
        }
    }

    void calculate_and_publish()
    {
        if (goal_pose_ != nullptr && current_pose_ != nullptr)
        {
            // 计算目标位置与当前位置之间的差值
            double delta_x = goal_pose_->x - current_pose_->x;
            double delta_y = goal_pose_->y - current_pose_->y;
            double delta_theta = goal_pose_->theta - current_pose_->theta;

            // 创建并发布速度指令消息
            auto twist_msg = geometry_msgs::msg::Twist();
            twist_msg.linear.x = delta_x / 0.2;
            twist_msg.linear.y = delta_y / 0.2;
            twist_msg.angular.z = delta_theta / 0.2;
            publisher_->publish(twist_msg);

            // 打印当前状态
            RCLCPP_INFO(this->get_logger(), "Moving towards goal: current_pose=(%f, %f), goal_pose=(%f, %f)",
                        current_pose_->x, current_pose_->y, goal_pose_->x, goal_pose_->y);
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TrajNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}