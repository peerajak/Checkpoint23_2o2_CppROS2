#include <functional>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/detail/odometry__struct.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"
#include "tortoisebot_waypoints/action/waypoint_action.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/empty.hpp>

class WaypointActionClass : public rclcpp::Node {
public:
  using WaypointAction = tortoisebot_waypoints::action::WaypointAction;
  using GoalHandleWaypointAction =
      rclcpp_action::ServerGoalHandle<WaypointAction>;

  explicit WaypointActionClass(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("waypoint_action_server", options) {
    using namespace std::placeholders;

    // Odom
    odom1_callback_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions options1;
    options1.callback_group = odom1_callback_group_;
    subscription1_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10,
        std::bind(&WaypointActionClass::odom_callback, this,
                  std::placeholders::_1),
        options1);

    // Action Server
    this->action_server_ = rclcpp_action::create_server<WaypointAction>(
        this, "tortoisebot_as_2",
        std::bind(&WaypointActionClass::handle_goal, this, _1, _2),
        std::bind(&WaypointActionClass::handle_cancel, this, _1),
        std::bind(&WaypointActionClass::handle_accepted, this, _1));

    // Cmd_vel
    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  }

private:
  // Odom
  rclcpp::CallbackGroup::SharedPtr odom1_callback_group_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription1_;
  geometry_msgs::msg::Point desire_pos_, current_pos_;
  float desire_pos_angle;
  geometry_msgs::msg::Quaternion desire_angle_, current_angle_;
  double target_yaw_rad_, current_yaw_rad_;
  // Action Server
  rclcpp_action::Server<WaypointAction>::SharedPtr action_server_;
  // Cmd_vel
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;

  rclcpp_action::GoalResponse
  handle_goal(const rclcpp_action::GoalUUID &uuid,
              std::shared_ptr<const WaypointAction::Goal> goal) {
    RCLCPP_INFO(this->get_logger(), "Received goal position %f,%f",
                goal->position.x, goal->position.y);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse
  handle_cancel(const std::shared_ptr<GoalHandleWaypointAction> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void
  handle_accepted(const std::shared_ptr<GoalHandleWaypointAction> goal_handle) {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a
    // new thread
    std::thread{std::bind(&WaypointActionClass::execute, this, _1), goal_handle}
        .detach();
  }

  void execute(const std::shared_ptr<GoalHandleWaypointAction> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<WaypointAction::Feedback>();
    auto &message = feedback->state;
    message = "Starting movement...";
    auto result = std::make_shared<WaypointAction::Result>();
    auto move = geometry_msgs::msg::Twist();
    rclcpp::Rate loop_rate(1);
    // int tiemout = 10;//seconds

    for (int i = 0; (i < 10) && rclcpp::ok(); ++i) {
      RCLCPP_INFO(this->get_logger(), "looping");
      // Check if there is a cancel request
      if (goal_handle->is_canceling()) {
        result->success = false; // message;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }
      // WaypointAction robot forward and send feedback
      message = "Moving forward...";
      move.linear.x = 0.3;
      publisher_->publish(move);
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(this->get_logger(), "Publish feedback");

      loop_rate.sleep();
    }

    // Check if goal is done
    if (rclcpp::ok()) {
      result->success =
          true; // "Finished action server. Robot moved during 5 seconds";
      move.linear.x = 0.0;
      publisher_->publish(move);
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_pos_ = msg->pose.pose.position;
    current_angle_ = msg->pose.pose.orientation;
    current_yaw_rad_ = yaw_theta_from_quaternion(
        msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);

    RCLCPP_DEBUG(this->get_logger(), "current pos=['%f','%f','%f'",
                 current_pos_.x, current_pos_.y, current_yaw_rad_);
  }

  double yaw_theta_from_quaternion(float qx, float qy, float qz, float qw) {
    double roll_rad, pitch_rad, yaw_rad;
    tf2::Quaternion odom_quat(qx, qy, qz, qw);
    tf2::Matrix3x3 matrix_tf(odom_quat);
    matrix_tf.getRPY(roll_rad, pitch_rad, yaw_rad);
    return yaw_rad; // In radian
  }

}; // class WaypointActionClass

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto action_server = std::make_shared<WaypointActionClass>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(action_server);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}