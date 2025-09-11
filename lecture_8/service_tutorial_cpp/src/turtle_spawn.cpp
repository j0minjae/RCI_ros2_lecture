#include "rclcpp/rclcpp.hpp"
#include "turtlesim/srv/spawn.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>

class SpawnTurtleNode: public rclcpp::Node
{
    public:
        SpawnTurtleNode() : Node("spawn_turtle_client")
        {
            client_ = this -> create_client<turtlesim::srv::Spawn>("spawn");
            // 요청 메시지 객체
            request_ = std::make_shared<turtlesim::srv::Spawn::Request>();
        }
        void send_request()
        {  
            request_ -> x = 5.0;
            request_ -> y = 2.0;
            request_ -> name = "new_turtle";
            
            // 서비스 서버가 준비될 때까지 대기
            while(!client_ -> wait_for_service(std::chrono::seconds(1))) //1초간 서버가 실행 중인지 확인
            {
                if(!rclcpp::ok())
                {
                    RCLCPP_INFO(this -> get_logger(), "Interrupted while waiting for the service. Exiting ...");
                    return;
                }
                RCLCPP_INFO(this -> get_logger(),"Service not available, waiting again ...");

            }
            // 작성한 요청(request)을 비동기적으로 서버에 보낸 후 진동벨(result)를 받아서 기다림
            auto result = client_->async_send_request(request_);
            
            // 서비스가 완료되면 result에 결과가 담기기때문에 해당 결과가 담기기를 result를 가지고 기다린다.
            if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
                rclcpp::FutureReturnCode::SUCCESS)  //응답이 성공적으로 도착하면
            {
                auto response =result.get();    //result가 담겼다고 알림이 울렸으니 result.get()을 통해 응답(response)메시지를 꺼내 로그를 띄움
                RCLCPP_INFO(this->get_logger(), "Successfully spawned a new turtle: %s", response -> name.c_str());
            }
            else{
                RCLCPP_ERROR(this->get_logger(), "Failed to spawn a new turtle");
            }
        }
    private:
        rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr client_;
        turtlesim::srv::Spawn::Request::SharedPtr request_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SpawnTurtleNode>();
    node -> send_request();
    rclcpp::shutdown();
}