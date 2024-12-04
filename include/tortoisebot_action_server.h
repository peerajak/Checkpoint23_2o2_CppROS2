#include "geometry_msgs/msg/twist.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/empty.hpp>
#include "nav_msgs/msg/detail/odometry__struct.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tortoisebot_waypoints/action/waypoint_action.hpp"
#include <math.h>

#ifndef TORTOISEBOT_ACTION_SERVER
#define TORTOISEBOT_ACTION_SERVER
#define PI 3.1416
class WaypointActionClass : public rclcpp::Node {
public:
  using WaypointAction = tortoisebot_waypoints::action::WaypointAction;
  using GoalHandleWaypointAction =  rclcpp_action::ServerGoalHandle<WaypointAction>;

  explicit WaypointActionClass(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  std::tuple<geometry_msgs::msg::Point, double, double> get_desire_current_robot_position_and_yawrad();
  double get_desire_pos_angle_yawrad();

private:
  // Odom
  rclcpp::CallbackGroup::SharedPtr odom1_callback_group_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription1_;
  geometry_msgs::msg::Point desire_pos_, current_pos_;
  double desire_pos_angle_yawrad;
  geometry_msgs::msg::Quaternion desire_angle_, current_angle_;
  double target_yaw_rad_, current_yaw_rad_;
  // Action Server
  rclcpp_action::Server<WaypointAction>::SharedPtr action_server_;
  // Cmd_vel
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  double _dist_precision = 0.25,_yaw_precision= PI/30; // +/- 2 degree allowed;
  std::string _state;

  //methods
  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid,
              std::shared_ptr<const WaypointAction::Goal> goal) ;
  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleWaypointAction> goal_handle);
  void handle_accepted(const std::shared_ptr<GoalHandleWaypointAction> goal_handle) ;
  void execute(const std::shared_ptr<GoalHandleWaypointAction> goal_handle);
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  double yaw_theta_from_quaternion(float qx, float qy, float qz, float qw);
}; // class WaypointActionClass

#endif