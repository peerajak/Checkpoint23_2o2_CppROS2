#include "geometry_msgs/msg/detail/point__struct.hpp"
#include "geometry_msgs/msg/detail/quaternion__struct.hpp"
#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "nav_msgs/msg/detail/odometry__struct.hpp"
#include "rclcpp/executors.hpp"
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

#ifndef TORTOISEBOT_ACTION_CLIENT
#define TORTOISEBOT_ACTION_CLIENT
class WaypointActionClient : public rclcpp::Node {
public:
  using WaypointAction = tortoisebot_waypoints::action::WaypointAction;
  using GoalHandleWaypointAction =
      rclcpp_action::ClientGoalHandle<WaypointAction>;

  explicit WaypointActionClient(double goto_x, double goto_y,
      const rclcpp::NodeOptions &node_options = rclcpp::NodeOptions());
  bool is_goal_done() const;
  void send_goal(double goto_x, double goto_y);

private:
  rclcpp_action::Client<WaypointAction>::SharedPtr client_ptr_;
  //rclcpp::TimerBase::SharedPtr timer_;
  bool goal_done_;
  std::vector<geometry_msgs::msg::Point> corner_goal_Pose;
  geometry_msgs::msg::Point start_position;
  geometry_msgs::msg::Point corner_bottom_right;
  geometry_msgs::msg::Point corner_bottom_left;
  geometry_msgs::msg::Point corner_top_left;
  geometry_msgs::msg::Point corner_top_right;
  geometry_msgs::msg::Point current_pos_;

  void goal_response_callback(const GoalHandleWaypointAction::SharedPtr &goal_handle);

  void feedback_callback( GoalHandleWaypointAction::SharedPtr,
      const std::shared_ptr<const GoalHandleWaypointAction::Feedback>feedback);

  void result_callback(const GoalHandleWaypointAction::WrappedResult &result);
}; // class WaypointActionClient



#endif