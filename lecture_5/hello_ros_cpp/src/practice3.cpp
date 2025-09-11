#include "rclcpp/rclcpp.hpp"

class LoggingNode:public rclcpp::Node
{
public:
    LoggingNode():Node("logging_node"), i_(1)
    {
        // 1초 = std::chrono::seconds(1)마다 timer_callback함수를 호출하도록 timeer_ 생성
        // timer_멤버변수에 저장하여 생성자가 끝나고도 소멸되지 않도록함
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&LoggingNode::timer_callback, this));
    }
private:
    void timer_callback()
    {
        RCLCPP_INFO(this->get_logger(),"counted to %d", i_);
        if (i_ % 3 == 0)
            RCLCPP_INFO(this->get_logger(),"is divisible by 3.");
        else if (i_ % 5 == 0)
            RCLCPP_DEBUG(this->get_logger(),"is divisible by 5.");
        else if (i_ % 7 == 0)
            RCLCPP_WARN(this->get_logger(),"is divisible by 7.");
        else if (i_ % 11 == 0)
            RCLCPP_ERROR(this->get_logger(),"is divisible by 11.");
        else if (i_ % 13 == 0)
            RCLCPP_FATAL(this->get_logger(),"is divisible by 13.");
        i_++;
    }
    //  스마트 포인터(SharePtr)를 사용해서 메모리 관리, 마지막 포인터가 객체를 가리키는 것을 멈출 때 객체의 메모리를 해제
    rclcpp::TimerBase::SharedPtr timer_;
    int i_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LoggingNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
