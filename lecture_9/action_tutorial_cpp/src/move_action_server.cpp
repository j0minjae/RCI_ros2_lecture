#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <turtlesim/srv/teleport_absolute.hpp>
#include <turtlesim/msg/pose.hpp>
#include "my_action/action/move_turtle.hpp"  

#include <cmath>
#include <thread>

class TurtleActionServer : public rclcpp::Node
{
public:
  using MoveTurtle = my_action::action::MoveTurtle;
  using GoalHandleMoveTurtle = rclcpp_action::ServerGoalHandle<MoveTurtle>;

  TurtleActionServer() : Node("turtle_action_server")
  {
    pose_subscriber_ = this->create_subscription<turtlesim::msg::Pose>(
        "turtle1/pose", 10, std::bind(&TurtleActionServer::pose_callback, this, std::placeholders::_1));

    teleport_client_ = this->create_client<turtlesim::srv::TeleportAbsolute>("/turtle1/teleport_absolute");
    while (!teleport_client_->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_INFO(this->get_logger(), "Waiting for TeleportAbsolute service...");
    }

    action_server_ = rclcpp_action::create_server<MoveTurtle>(
        this,
        "move_turtle",
        std::bind(&TurtleActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&TurtleActionServer::handle_cancel, this, std::placeholders::_1),
        std::bind(&TurtleActionServer::handle_accepted, this, std::placeholders::_1));
  }

private:
  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_subscriber_;
  rclcpp::Client<turtlesim::srv::TeleportAbsolute>::SharedPtr teleport_client_;
  rclcpp_action::Server<MoveTurtle>::SharedPtr action_server_;

  turtlesim::msg::Pose current_pose_;

  void pose_callback(const turtlesim::msg::Pose::SharedPtr msg)
  {
    current_pose_ = *msg;
  }

  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID &uuid,
      std::shared_ptr<const MoveTurtle::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Goal accepted: x=%f, y=%f, theta=%f", goal->x, goal->y, goal->theta);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleMoveTurtle> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Goal canceled");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleMoveTurtle> goal_handle)
  {
    std::thread{std::bind(&TurtleActionServer::execute, this, std::placeholders::_1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleMoveTurtle> goal_handle)
  {
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<MoveTurtle::Feedback>();
    auto result = std::make_shared<MoveTurtle::Result>();

    double current_x = current_pose_.x;
    double current_y = current_pose_.y;
    double current_theta = current_pose_.theta;

    int total_steps = static_cast<int>(goal->duration / 0.1);
    double x_step = (goal->x - current_x) / total_steps;
    double y_step = (goal->y - current_y) / total_steps;
    double theta_step = (goal->theta - current_theta) / total_steps;

    for (int step = 0; step < total_steps; ++step)
    {
      if (goal_handle->is_canceling()) {
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }

      current_x += x_step;
      current_y += y_step;
      current_theta += theta_step;

      double distance_to_goal = std::sqrt(std::pow(goal->x - current_x, 2) + std::pow(goal->y - current_y, 2));

      auto request = std::make_shared<turtlesim::srv::TeleportAbsolute::Request>();
      request->x = current_x;
      request->y = current_y;
      request->theta = current_theta;

      auto future = teleport_client_->async_send_request(request);

      feedback->distance_to_goal = distance_to_goal;
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(this->get_logger(), "Remaining distance: %f", distance_to_goal);

      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    result->success = true;
    result->final_x = goal->x;
    result->final_y = goal->y;
    result->final_theta = goal->theta;
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Final result: x=%f, y=%f, theta=%f", result->final_x, result->final_y, result->final_theta);
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TurtleActionServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}