#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <chrono>
#include <functional>
#include <stdlib.h>



class ReverseVelNode : public rclcpp::Node
{
    public: //생성자
        ReverseVelNode():Node("pub_reverse_vel_node")
        {
            publisher_=this -> create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel_reverse",10);
            timer_=this->create_wall_timer(
                std::chrono::milliseconds(100),
                // 멤버함수를 일반 함수로 
                std::bind(&ReverseVelNode::timer_callback, this));
            msg_=std::make_shared<geometry_msgs::msg::Twist>();
        }
    private: //각 멤버 변수를 저장할 스마트 포인터 변수, 비교적 무거운 객체들으 관리하는데 특화
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
        geometry_msgs::msg::Twist::SharedPtr msg_;
    
    void timer_callback()
    {
        double linear_x = double(rand()-double(RAND_MAX)/2.0)/double(RAND_MAX);
        double angular_z = 2 * double(rand())/double(RAND_MAX)-1;

        msg_->linear.x = -linear_x;
        msg_->angular.z = -angular_z;
        publisher_ -> publish(*msg_);
        RCLCPP_INFO(this -> get_logger(), "Twist Message - Linear X: %f Angular Z: %f",
            msg_ -> linear.x, msg_ -> angular.z);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ReverseVelNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
}