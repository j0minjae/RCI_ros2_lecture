#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"
#include <memory>

using std::placeholders::_1;
class SubPoseNode : public rclcpp::Node
{
    public:
        SubPoseNode() : Node("subscribe_pose_node")
        {
            //구독자 생성, 토픽에서 메시지가 수신되었을 떄 호출할 함수pose_callback, 메시지가 도착하면 그 메시지를 _1에 넣어 pose_callback 함수의 인자로 전달
            subscription_ = this -> create_subscription<turtlesim::msg::Pose>
                ("turtle1/pose", 10, std::bind(&SubPoseNode::pose_callback, this, _1));
        }
    private:
        void pose_callback(turtlesim::msg::Pose::SharedPtr msg)
        {
            double x = msg -> x;
            double y = msg -> y;
            double theta = msg ->  theta;

            RCLCPP_INFO(this -> get_logger(), "Turtlebot Pose -X: %f, Y: %f, Theta: %f", x, y, theta);

        }

        rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscription_;
};

int main(int argv, char *argc[])
{
    rclcpp::init(argv, argc);
    auto node = std::make_shared<SubPoseNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
}