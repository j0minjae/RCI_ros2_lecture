#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/bool.hpp"


class DirectionCheckNode : public rclcpp::Node
{
    public: //생성자
        DirectionCheckNode():Node("direction_check_node")
        {
            subscription_ = this -> create_subscription<geometry_msgs::msg::Twist>
                ("turtle1/cmd_vel", 10, std::bind(&DirectionCheckNode::twist_callback, this, std::placeholders::_1));
            
            publisher_=this -> create_publisher<std_msgs::msg::Bool>("linear_x_sign",10);

            RCLCPP_INFO(this -> get_logger(), "DirectionCheck node has been started");
        }
    private: //각 멤버 변수를 저장할 스마트 포인터 변수, 비교적 무거운 객체들으 관리하는데 특화
        void twist_callback(geometry_msgs::msg::Twist::SharedPtr msg)
        {
            std_msgs::msg::Bool bool_msg;
            bool_msg.data = (msg -> linear.x >= 0);

            if (bool_msg.data)
            {
                RCLCPP_INFO(this -> get_logger(), "Bool Linear_x: True");
            }
            else
            {
                RCLCPP_INFO(this -> get_logger(), "Bool Linear_x: False");
            }
            publisher_ -> publish(bool_msg);
        }
    
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DirectionCheckNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}