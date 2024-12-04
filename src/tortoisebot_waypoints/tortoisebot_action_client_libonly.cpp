#include "tortoisebot_action_client.h"

using WaypointAction = tortoisebot_waypoints::action::WaypointAction;
using GoalHandleWaypointAction = rclcpp_action::ClientGoalHandle<WaypointAction>;


WaypointActionClient::WaypointActionClient(double goto_x, double goto_y,
      const rclcpp::NodeOptions &node_options)
      : Node("waypoint_action_client", node_options), goal_done_(false) {
    this->client_ptr_ = rclcpp_action::create_client<WaypointAction>(
        this->get_node_base_interface(), this->get_node_graph_interface(),
        this->get_node_logging_interface(),
        this->get_node_waitables_interface(), "tortoisebot_as_2");

    send_goal(goto_x, goto_y);
}

bool WaypointActionClient::is_goal_done() const { return this->goal_done_; }

void WaypointActionClient::send_goal(double goto_x, double goto_y) {
    using namespace std::placeholders;
    rclcpp::Rate loop_rate(1);

    //this->timer_->cancel();

    this->goal_done_ = false;

    if (!this->client_ptr_) {
      RCLCPP_ERROR(this->get_logger(), "Action client not initialized");
    }

    if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
      RCLCPP_ERROR(this->get_logger(),
                   "Action server not available after waiting");
      //this->goal_done_ = true;

    }
    auto goal_msg = WaypointAction::Goal();
    goal_msg.position.x = goto_x;
    goal_msg.position.y = goto_y;

    RCLCPP_INFO(this->get_logger(), "Sending goal x:%f y:%f",
                goal_msg.position.x, goal_msg.position.y);
    auto send_goal_options = rclcpp_action::Client<WaypointAction>::SendGoalOptions();
    send_goal_options.goal_response_callback = std::bind(&WaypointActionClient::goal_response_callback, this, _1);
    send_goal_options.feedback_callback = std::bind(&WaypointActionClient::feedback_callback, this, _1, _2);
    send_goal_options.result_callback = std::bind(&WaypointActionClient::result_callback, this, _1);
    // real goal sending is here
    auto goal_handle_future = this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
    return;
}

void WaypointActionClient::goal_response_callback(
      const GoalHandleWaypointAction::SharedPtr &goal_handle) {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(),
                  "Goal accepted by server, waiting for result");
    }
}

void WaypointActionClient::feedback_callback(
      GoalHandleWaypointAction::SharedPtr,
      const std::shared_ptr<const GoalHandleWaypointAction::Feedback>
          feedback) {

    RCLCPP_INFO(this->get_logger(), "Feedback received: x %f, y %f",
                feedback->position.x, feedback->position.y);
    current_pos_.x = feedback->position.x;
    current_pos_.y = feedback->position.y;
}

void WaypointActionClient::result_callback(const GoalHandleWaypointAction::WrappedResult &result) {
    this->goal_done_ = true;
    switch (result.code) {

    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_ERROR(this->get_logger(), "Goal was success? %s",
                   result.result->success ? "yes" : "no");

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

    RCLCPP_INFO(this->get_logger(), "Result received:");
}


