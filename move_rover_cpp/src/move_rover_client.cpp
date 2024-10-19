#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>

#include "action_move_rover/action/move_rover.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace move_rover_cpp
{
class MoveRoverAction : public rclcpp::Node
{
public:
  using MoveRover = action_move_rover::action::MoveRover;
  using GoalHandleMoveRover = rclcpp_action::ClientGoalHandle<MoveRover>;
  int distance_inp;

  explicit MoveRoverAction(const rclcpp::NodeOptions & options)
  : Node("move_rover_client", options)
  {
    this->client_ptr_ = rclcpp_action::create_client<MoveRover>(
      this,
      "move_rover");

    this->timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&MoveRoverAction::send_goal, this));
  }

  void send_goal()
  {
    using namespace std::placeholders;

    this->timer_->cancel();

    if (!this->client_ptr_->wait_for_action_server()) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      rclcpp::shutdown();
    }

    auto goal_msg = MoveRover::Goal();
    std::cout << "Enter the distance you want the rover to move (in meters): ";

    std::cin >> distance_inp;
    
    goal_msg.distance_to_move = distance_inp;

    RCLCPP_INFO(this->get_logger(), "Sending distance goal to rover..");

    auto send_goal_options = rclcpp_action::Client<MoveRover>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      std::bind(&MoveRoverAction::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
      std::bind(&MoveRoverAction::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
      std::bind(&MoveRoverAction::result_callback, this, _1);
    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  rclcpp_action::Client<MoveRover>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;

  void goal_response_callback(const GoalHandleMoveRover::SharedPtr & goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
  }

  void feedback_callback(
    GoalHandleMoveRover::SharedPtr,
    const std::shared_ptr<const MoveRover::Feedback> feedback)
  {
    std::stringstream ss;
    for (auto number : feedback->distance_moved) {
      ss << number << " meter(s) out of " << distance_inp << " covered. \n\n";
    }
    RCLCPP_INFO(this->get_logger(), ss.str().c_str());
  }

  void result_callback(const GoalHandleMoveRover::WrappedResult & result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        return;
    }
    std::stringstream ss;
    ss << "Rover Control says: ";
    
    for (auto number : result.result->distance_result) {
        
        ss << number << " meters";
    }
    RCLCPP_INFO(this->get_logger(), ss.str().c_str());
    rclcpp::shutdown();
  }
};  // class MoveRoverAction

}  // namespace move_rover_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(move_rover_cpp::MoveRoverAction)