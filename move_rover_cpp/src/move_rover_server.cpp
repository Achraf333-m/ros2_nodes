#include <functional>
#include <memory>
#include <thread>

#include "action_move_rover/action/move_rover.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "move_rover_cpp/visibility_control.h"



namespace move_rover_cpp
{
class MoveRoverAction : public rclcpp::Node
{
public:
  using MoveRover = action_move_rover::action::MoveRover;
  using GoalHandleMoveRover = rclcpp_action::ServerGoalHandle<MoveRover>;

  MOVE_ROVER_CPP_PUBLIC
  explicit MoveRoverAction(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("move_rover_server", options)
  {
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<MoveRover>(
      this,
      "move_rover",
      std::bind(&MoveRoverAction::handle_goal, this, _1, _2),
      std::bind(&MoveRoverAction::handle_cancel, this, _1),
      std::bind(&MoveRoverAction::handle_accepted, this, _1));
  }

private:
  rclcpp_action::Server<MoveRover>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const MoveRover::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request with distance to move: %d meters", goal->distance_to_move);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleMoveRover> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal, stopping rover...");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleMoveRover> goal_handle)
  {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&MoveRoverAction::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleMoveRover> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "ROVER IS MOVING..MAKE WAY");
    rclcpp::Rate loop_rate(1);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<MoveRover::Feedback>();
    auto & sequence = feedback->distance_moved;
    // herechange this, you do not need an array
    sequence.push_back(0);

    // make sure you are handling strings correctly
    auto result = std::make_shared<MoveRover::Result>();

    for (int i = 1; (i < goal->distance_to_move) && rclcpp::ok(); ++i) {
      // Check if there is a cancel request
      if (goal_handle->is_canceling()) {
        result->distance_result = sequence;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled, rover has stopped");
        return;
      }
      // Update sequence
    //   handle non arrays
      sequence.push_back(i);
      // Publish feedback
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(this->get_logger(), "Publish distance covered by rover");

      loop_rate.sleep();
    }

    // Check if goal is done
    if (rclcpp::ok()) {
      result->distance_result.push_back(sequence[19]);
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded. Rover has moved given distance!");
    }
  }
};  // class MoveRoverAction

}  // namespace move_rover_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(move_rover_cpp::MoveRoverAction)