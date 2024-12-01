#include "geometry_msgs/msg/detail/point__struct.hpp"
#include "geometry_msgs/msg/detail/quaternion__struct.hpp"
#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "nav_msgs/msg/detail/odometry__struct.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/utilities.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"
#include "tortoisebot_waypoints/action/waypoint_action.hpp"
#include <geometry_msgs/msg/point.h>
#include <inttypes.h>
#include <iostream>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <string>
#include <vector>

//#include "geometry_msgs/Point.h"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <cmath>

#define pi 3.14

int action_counter = 0;

float degree_to_radian(float degree) { return degree / 180 * pi; }
float radian_to_degree(float rad) { return rad / pi * 180; }

double yaw_theta_from_quaternion(float qx, float qy, float qz, float qw) {
  double roll_rad, pitch_rad, yaw_rad;
  tf2::Quaternion odom_quat(qx, qy, qz, qw);
  tf2::Matrix3x3 matrix_tf(odom_quat);
  matrix_tf.getRPY(roll_rad, pitch_rad, yaw_rad);
  return yaw_rad; // In radian
}

float theta_from_arctan(float x_target, float x_current, float y_target,
                        float y_current) {
  // float atan_ans = std::atan((y_target - y_current) / (x_target -
  // x_current));
  float ret;
  if (x_target < x_current && y_target < y_current)
    ret = -pi / 2 - std::atan((x_target - x_current) / (y_target - y_current));
  else if (x_target < x_current && y_target >= y_current)
    ret = pi / 2 - std::atan((x_target - x_current) / (y_target - y_current));
  else
    ret = std::atan((y_target - y_current) / (x_target - x_current));
  return ret;
}

class WaypointActionClient : public rclcpp::Node {
public:
  using WaypointAction = tortoisebot_waypoints::action::WaypointAction;
  using GoalHandleWaypointAction =
      rclcpp_action::ClientGoalHandle<WaypointAction>;

  explicit WaypointActionClient(
      const rclcpp::NodeOptions &node_options = rclcpp::NodeOptions())
      : Node("waypoint_action_client", node_options), goal_done_(false) {
    this->client_ptr_ = rclcpp_action::create_client<WaypointAction>(
        this->get_node_base_interface(), this->get_node_graph_interface(),
        this->get_node_logging_interface(),
        this->get_node_waitables_interface(), "tortoisebot_as_2");

    this->timer_ = this->create_wall_timer(
        std::chrono::milliseconds(0),
        std::bind(&WaypointActionClient::send_goal, this));

    start_position.x = -0.068263;
    start_position.y = 0.50008;
    // start_position.theta = theta_from_arctan(0.737, -0.068263, 0.576,
    // 0.50008);

    corner_bottom_left.x = 0.737;
    corner_bottom_left.y = 0.576;
    // corner_bottom_left.theta = theta_from_arctan(0.88435, 0.737, -0.57935,
    // 0.576);

    corner_bottom_right.x = 0.88435;
    corner_bottom_right.y = -0.57935;
    // corner_bottom_right.theta =  theta_from_arctan(-0.53170, 0.88435,
    // -0.47636, -0.57935);

    corner_top_left.x = -0.53170;
    corner_top_left.y = -0.47636;
    // corner_top_left.theta =
    // theta_from_arctan(-0.55024, -0.53170, 0.381047, -0.47636);

    corner_top_right.x = -0.63024;
    corner_top_right.y = 0.5;
    // corner_top_right.theta =
    // theta_from_arctan(0.737, -0.55024, 0.50008, 0.381047);
    corner_goal_Pose.push_back(start_position);
    corner_goal_Pose.push_back(corner_bottom_left);
    corner_goal_Pose.push_back(corner_bottom_right);
    corner_goal_Pose.push_back(corner_top_left);
    corner_goal_Pose.push_back(corner_top_right);
    for (int i = 0; i < action_counter; i++) {
      std::rotate(corner_goal_Pose.begin(), corner_goal_Pose.begin() + 1,
                  corner_goal_Pose.end());
    }
    current_pos_.x = corner_goal_Pose.back().x;
    current_pos_.y = corner_goal_Pose.back().y;
  }

  bool is_goal_done() const { return this->goal_done_; }

  void send_goal() {
    using namespace std::placeholders;

    this->timer_->cancel();

    this->goal_done_ = false;

    if (!this->client_ptr_) {
      RCLCPP_ERROR(this->get_logger(), "Action client not initialized");
    }

    if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
      RCLCPP_ERROR(this->get_logger(),
                   "Action server not available after waiting");
      this->goal_done_ = true;
      return;
    }

    auto goal_msg = WaypointAction::Goal();

    goal_msg.position.x = corner_goal_Pose[0].x;
    goal_msg.position.y = corner_goal_Pose[0].y;

    // theta_from_arctan(goal_msg.position.x, current_pos_.x,
    //                   goal_msg.position.y, current_pos_.y);

    RCLCPP_INFO(this->get_logger(), "Sending goal x:%f y:%f",
                goal_msg.position.x, goal_msg.position.y);

    auto send_goal_options =
        rclcpp_action::Client<WaypointAction>::SendGoalOptions();

    send_goal_options.goal_response_callback =
        std::bind(&WaypointActionClient::goal_response_callback, this, _1);

    send_goal_options.feedback_callback =
        std::bind(&WaypointActionClient::feedback_callback, this, _1, _2);

    send_goal_options.result_callback =
        std::bind(&WaypointActionClient::result_callback, this, _1);

    auto goal_handle_future =
        this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  rclcpp_action::Client<WaypointAction>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool goal_done_;
  std::vector<geometry_msgs::msg::Point> corner_goal_Pose;
  geometry_msgs::msg::Point start_position;
  geometry_msgs::msg::Point corner_bottom_right;
  geometry_msgs::msg::Point corner_bottom_left;
  geometry_msgs::msg::Point corner_top_left;
  geometry_msgs::msg::Point corner_top_right;
  geometry_msgs::msg::Point current_pos_;

  void goal_response_callback(
      const GoalHandleWaypointAction::SharedPtr &goal_handle) {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(),
                  "Goal accepted by server, waiting for result");
    }
  }

  void feedback_callback(
      GoalHandleWaypointAction::SharedPtr,
      const std::shared_ptr<const GoalHandleWaypointAction::Feedback>
          feedback) {

    RCLCPP_INFO(this->get_logger(), "Feedback received: x %f, y %f",
                feedback->position.x, feedback->position.y);
    current_pos_.x = feedback->position.x;
    current_pos_.y = feedback->position.y;
  }

  void result_callback(const GoalHandleWaypointAction::WrappedResult &result) {
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
}; // class WaypointActionClient

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  rclcpp::executors::MultiThreadedExecutor executor;

  while (rclcpp::ok()) {
    auto action_client = std::make_shared<WaypointActionClient>();
    executor.add_node(action_client);
    while (!action_client->is_goal_done()) {
      executor.spin_some();
    }
    executor.remove_node(action_client);
    action_counter++;
  }
  rclcpp::shutdown();
  return 0;
}