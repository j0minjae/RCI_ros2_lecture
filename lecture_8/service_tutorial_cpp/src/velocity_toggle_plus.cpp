#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "my_srv/srv/changerate.hpp"

class PubVelToggleRate: public rclcpp::Node
{
    public:
        PubVelToggleRate() : Node("pubvel_toggle_plus"), forward_(true), cnt_(0), frequency_(2.0)
        {
            toggle_server_ = this -> create_service<std_srvs::srv::Empty>(
                "toggle_forward",   
                std::bind(&PubVelToggleRate::toggle_forward_callback, this, std::placeholders::_1, std::placeholders::_2)
            );
            change_rate_server_ = this -> create_service<my_srv::srv::Changerate>(
                "change_rate",   
                std::bind(&PubVelToggleRate::change_rate_callback, this, std::placeholders::_1, std::placeholders::_2)
            );
            pub_= this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);

            RCLCPP_INFO(this->get_logger(), "PubVelToggleRate node has been started");
        }
        // main에서 호출하기 때문에 public에 둬야한다.
        void loop()
        {
            while(rclcpp::ok())
            {
                //루프의 실행 주기를 일정하게 유지해주는 도우미 클래스 Rate
                rclcpp::Rate rate(frequency_);
                publish_velocity();
                //멈추지않고 내 일을 하다가 틈틈히 쌓여있는 콜백을 딱 한 번만 처리하고 다음 코드로 넘어감
                rclcpp::spin_some(this->get_node_base_interface());
                rate.sleep();
            }
        }

    private:
        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr toggle_server_;
        rclcpp::Service<my_srv::srv::Changerate>::SharedPtr change_rate_server_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
        rclcpp::TimerBase::SharedPtr timer_;
        bool forward_;
        int cnt_;
        double frequency_;

        
        // server는 어떠한 bool값 등을 반환하지않고 해당 함수에서 사용하는 인터페이스에 따라 response값을 채워주기만 하면 된다. 
        void change_rate_callback(
            const std::shared_ptr<my_srv::srv::Changerate::Request> request,
            std::shared_ptr<my_srv::srv::Changerate::Response> response)
        {
            RCLCPP_INFO(this->get_logger(),"Changing rate to %f", request->newrate);
            frequency_ = request->newrate;
            response->ret = true;
        }
        
        void toggle_forward_callback(
            const std::shared_ptr<std_srvs::srv::Empty::Request> request,
            std::shared_ptr<std_srvs::srv::Empty::Response> response)
        {
            cnt_++;
            forward_= !forward_;
            RCLCPP_INFO(this->get_logger(), "Now sending %s commands.", forward_ ? "forward" : "rotate");
        }
        // loop함수에서 호출되기는하지만 public에 포함시키지 않아도 된다.
        void publish_velocity()
        {   
            if(cnt_ == 0){
                return;
            }
            auto msg = std::make_shared<geometry_msgs::msg::Twist>();  
            msg->linear.x = forward_ ? 1.0 : 0.0;
            msg->angular.z = forward_ ? 0.0: 1.0;
            pub_->publish(*msg);
            RCLCPP_INFO(this->get_logger(), "Published velocity command - linear.x: %.2f, angular.z: %.2f", msg->linear.x, msg->angular.z);
            RCLCPP_INFO(this->get_logger(), "Frequency: %.2f Hz", frequency_);
        }
        
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PubVelToggleRate>();
    node->loop();
    rclcpp::shutdown();
    return 0;
}