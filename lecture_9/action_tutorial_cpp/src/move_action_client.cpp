#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "my_action/action/move_turtle.hpp"

class TurtleActionClient : public rclcpp::Node
{
    public:
    using MoveTurtle = my_action::action::MoveTurtle;
    using GoalHandleMoveTurtle = rclcpp_action::ClientGoalHandle<MoveTurtle>;

    TurtleActionClient() : Node("turtle_action_client")
    {
        this->client_ptr_ = rclcpp_action::create_client<MoveTurtle>(this, "move_turtle");
        this->declare_parameter("x", 1.0);
        this->declare_parameter("y", 1.0);
        this->declare_parameter("theta", 1.57);
        this->declare_parameter("duration", 3.0);
    }

    void send_goal(float x, float y, float theta, float duration)
    {
        if (!this->client_ptr_->wait_for_action_server()) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            return;
        }

        auto goal_msg = MoveTurtle::Goal();
        goal_msg.x = x;
        goal_msg.y = y;
        goal_msg.theta = theta;
        goal_msg.duration = duration;

        RCLCPP_INFO(this->get_logger(), "Sending goal: x=%f, y=%f, theta=%f", x, y, theta);

        auto send_goal_options = rclcpp_action::Client<MoveTurtle>::SendGoalOptions();
        send_goal_options.goal_response_callback = std::bind(&TurtleActionClient::goal_response_callback, this, std::placeholders::_1);
        send_goal_options.feedback_callback = std::bind(&TurtleActionClient::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
        send_goal_options.result_callback = std::bind(&TurtleActionClient::result_callback, this, std::placeholders::_1);

        this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
    }

    private:
    rclcpp_action::Client<MoveTurtle>::SharedPtr client_ptr_;

    void goal_response_callback(std::shared_ptr<GoalHandleMoveTurtle> goal_handle)
    {
        if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        } else {
        RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
        }
    }

    void feedback_callback(GoalHandleMoveTurtle::SharedPtr, const std::shared_ptr<const MoveTurtle::Feedback> feedback)
    {
        RCLCPP_INFO(this->get_logger(), "Current distance to goal: %f", feedback->distance_to_goal);
    }

    void result_callback(const GoalHandleMoveTurtle::WrappedResult &result)
    {
        switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(), "Goal succeeded!");
            RCLCPP_INFO(this->get_logger(), "Final position: (%f, %f, %f)", result.result->final_x, result.result->final_y, result.result->final_theta);
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
  auto action_client = std::make_shared<TurtleActionClient>();

  float x = action_client->get_parameter("x").as_double();
  float y = action_client->get_parameter("y").as_double();
  float theta = action_client->get_parameter("theta").as_double();
  float duration = action_client->get_parameter("duration").as_double();

  action_client->send_goal(x, y, theta, duration);
  rclcpp::spin(action_client);
  rclcpp::shutdown();
  return 0;
}