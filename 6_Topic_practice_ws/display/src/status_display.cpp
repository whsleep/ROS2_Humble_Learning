#include <QApplication>
#include <QLabel>
#include <QString>
#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/system_status.hpp"

using SystemStatus = interfaces::msg::SystemStatus;

class SysStatusDisplay : public rclcpp::Node
{
public:
    SysStatusDisplay() : Node("sys_status_display")
    {
        // 这里使用Lambda表达式作为回调函数，Lambda表达式
        // 的参数列表中的&符表示它可以通过引用的方式直接捕获外部变量，
        // 这也是可以在表达式中直接调用label_设置文本的原因
        subscription_ = this->create_subscription<SystemStatus>(
            "sys_status", 10, [&](const SystemStatus::SharedPtr msg) -> void
            { label_->setText(get_qstr_from_msg(msg)); });
        // 创建一个空的 SystemStatus 对象，转化成 QString 进行显示
        label_ = new QLabel(get_qstr_from_msg(std::make_shared<SystemStatus>()));
        label_->show();
    }
    QString get_qstr_from_msg(const SystemStatus::SharedPtr msg)
    {
        std::stringstream show_str;
        show_str
            << "===========System Status Visualization Tool============\n"
            << "Data Time:\t" << msg->stamp.sec << "\ts\n"
            << "Host Name:\t" << msg->host_name << "\t\n"
            << "CPU Usage:\t" << msg->cpu_percent << "\t%\n"
            << "Memory Usage:\t" << msg->memory_percent << "\t%\n"
            << "Total Memory:\t" << msg->memory_total << "\tMB\n"
            << "Available Memory:\t" << msg->memory_available << "\tMB\n"
            << "Network Sent:\t" << msg->net_sent << "\tMB\n"
            << "Network Received:\t" << msg->net_recv << "\tMB\n"
            << "======================================================";

        return QString::fromStdString(show_str.str());
    }

private:
    rclcpp::Subscription<SystemStatus>::SharedPtr subscription_;
    QLabel *label_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    QApplication app(argc, argv);
    auto node = std::make_shared<SysStatusDisplay>();

    // spin 单开一个线程，防止阻塞
    std::thread spin_thread([&]() -> void
                            { rclcpp::spin(node); });
    spin_thread.detach();
    app.exec();
    rclcpp::shutdown();
    return 0;
}