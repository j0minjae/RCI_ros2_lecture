#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "my_action/action/add_until_goal.hpp"

class AddUntilGoalActionClient : public rclcpp::Node
{
    public:
    using AddUntilGoal = my_action::action::AddUntilGoal;
    using GoalHandleAddUntilGoal = rclcpp_action::ClientGoalHandle<AddUntilGoal>;

    AddUntilGoalActionClient() : Node("add_until_goal_action_client")
    {
        this->client_ptr_ = rclcpp_action::create_client<AddUntilGoal>(this, "add_until_goal");
        this->declare_parameter("target_number", 10);
    }

    void send_goal(int32_t target_number)
    {
        if (!this->client_ptr_->wait_for_action_server()) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            return;
        }

        auto goal_msg = AddUntilGoal::Goal();
        goal_msg.target_number = target_number;

        RCLCPP_INFO(this->get_logger(), "Sending goal to reach: %d", target_number);

        auto send_goal_options = rclcpp_action::Client<AddUntilGoal>::SendGoalOptions();
        send_goal_options.goal_response_callback = std::bind(&AddUntilGoalActionClient::goal_response_callback, this, std::placeholders::_1);
        send_goal_options.feedback_callback = std::bind(&AddUntilGoalActionClient::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
        send_goal_options.result_callback = std::bind(&AddUntilGoalActionClient::result_callback, this, std::placeholders::_1);

        this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
    }

    private:
    rclcpp_action::Client<AddUntilGoal>::SharedPtr client_ptr_;

    void goal_response_callback(std::shared_ptr<GoalHandleAddUntilGoal> goal_handle)
    {
        if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        } else {
        RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
        }
    }

    void feedback_callback(GoalHandleAddUntilGoal::SharedPtr, const std::shared_ptr<const AddUntilGoal::Feedback> feedback)
    {
        RCLCPP_INFO(this->get_logger(), "Current number: %d", feedback->current_number);
    }

    void result_callback(const GoalHandleAddUntilGoal::WrappedResult &result)
    {
        switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(), "Goal succeeded!");
            RCLCPP_INFO(this->get_logger(), "Final number: %d", result.result->final_number);
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
            break;
        default:
            RCLCPP_ERROR(this->get_logger(), "Unknown result code");
            break;
        }
        rclcpp::shutdown();
    }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto action_client = std::make_shared<AddUntilGoalActionClient>();

  int32_t target_number = action_client->get_parameter("target_number").as_int();

  action_client->send_goal(target_number);
  rclcpp::spin(action_client);
  rclcpp::shutdown();
  return 0;
}