#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "my_action/action/add_until_goal.hpp" 
#include <chrono>
#include <thread>

class AddUntilGoalActionServer : public rclcpp::Node
{
    public:
    using AddUntilGoal = my_action::action::AddUntilGoal;
    using GoalHandleAddUntilGoal = rclcpp_action::ServerGoalHandle<AddUntilGoal>;

    AddUntilGoalActionServer() : Node("add_until_goal_action_server")
    {
        this->action_server_ = rclcpp_action::create_server<AddUntilGoal>(
            this,
            "add_until_goal",
            std::bind(&AddUntilGoalActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&AddUntilGoalActionServer::handle_cancel, this, std::placeholders::_1),
            std::bind(&AddUntilGoalActionServer::handle_accepted, this, std::placeholders::_1));
    }

    private:
    rclcpp_action::Server<AddUntilGoal>::SharedPtr action_server_;

    //새로운 목표 요청이 들어왔을시, handle_goal은 목표 수락/거절을 경정
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const AddUntilGoal::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received goal request: reach %d", goal->target_number);
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }
    //기존 목표의 취소 요청시, 취소요청 수락/거절을 결정
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleAddUntilGoal> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        return rclcpp_action::CancelResponse::ACCEPT;
    }
    //서버가 목표를 수락한 직후 호출, 실제 작업 시작
    void handle_accepted(const std::shared_ptr<GoalHandleAddUntilGoal> goal_handle)
    {
        std::thread{std::bind(&AddUntilGoalActionServer::execute, this, std::placeholders::_1), goal_handle}.detach();
    }
    // 실제 작업 수행, accepted에서 생성된 별도의 쓰레드 함수에서 실행
    void execute(const std::shared_ptr<GoalHandleAddUntilGoal> goal_handle)
    {
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<AddUntilGoal::Feedback>();
        auto result = std::make_shared<AddUntilGoal::Result>();

        int current_number = 0;
        int target_number = goal->target_number;

        while (current_number < target_number)
        {
            if (goal_handle->is_canceling()) {
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "Goal canceled");
                return;
            }

            current_number += 1;

            feedback->current_number = current_number;
            goal_handle->publish_feedback(feedback);
            RCLCPP_INFO(this->get_logger(), "Current number: %d", current_number);

            std::this_thread::sleep_for(std::chrono::seconds(1));
        }

        result->success = true;
        result->final_number = current_number;
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "Goal succeeded, final number: %d", current_number);
    }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AddUntilGoalActionServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}