#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"
#include "geometry_msgs/msg/twist.hpp"

class PubVelToggle: public rclcpp::Node
{
    public:
        PubVelToggle() : Node("pubvel_toggle"), forward_(true), cnt_(0)
        {
            // 요청과 응답에 아무 데이터도 없는 서비스 타입을 사용한 서비스 
            server_ = this -> create_service<std_srvs::srv::Empty>(
                "toggle_forward",   //서비스 이름, 다른 노느뎅서 이 이름으로 서비스를 호출
                std::bind(&PubVelToggle::toggle_forward_callback, this, std::placeholders::_1, std::placeholders::_2)
            );//호출되었을때 toggle_forward_callback를 실행하고 _1과_2는 각각 요청과 응답 객체를 위한 자리 표시자임
            

            pub_= this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);

            // 0.5초마다 속도 발행
            timer_ = this->create_wall_timer(
                std::chrono::milliseconds(500),
                std::bind(&PubVelToggle::publish_velocity, this)
            );

            RCLCPP_INFO(this->get_logger(), "PubVelToggle node has been started");
        }
    private:
        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr server_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
        rclcpp::TimerBase::SharedPtr timer_;
        bool forward_;
        int cnt_;
        
        // ros2 service call /toggle_forward std_srvs/srv/Empty와 같이 외부에서 서비스가 호출될 떄만 실행됨 
        bool toggle_forward_callback(
            const std::shared_ptr<std_srvs::srv::Empty::Request> request,
            std::shared_ptr<std_srvs::srv::Empty::Response> response)
        {
            // cnt_ 1 증가 및 forward 변경
            cnt_++;
            forward_= !forward_;
            RCLCPP_INFO(this->get_logger(), "Now sending %s commands.", forward_ ? "forward" : "rotate");
            return true;
        }

        void publish_velocity()
        {   
            // cnt_는 서비스가 호출되는 횟수, 서비스가 한 번 이라도 호출되기 전에는 아무것도 하지 않음
            if(cnt_ == 0){
                return;
            }
            
            // 서비스가 한번이라도 호출된다면 아래를 실행 
            auto msg = std::make_shared<geometry_msgs::msg::Twist>();
            
            // forwrd_ 상태에 따라 직진 또는 회전 속도 설정
            msg->linear.x = forward_ ? 1.0 : 0.0;
            msg->angular.z = forward_ ? 0.0: 1.0;

            pub_->publish(*msg);

            RCLCPP_INFO(this->get_logger(), "Published velocity command - linear.x: %.2f, angular.z: %.2f", msg->linear.x, msg->angular.z);
        }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<PubVelToggle>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}