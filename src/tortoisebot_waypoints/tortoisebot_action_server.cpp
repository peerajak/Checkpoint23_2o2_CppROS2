#include <functional>
#include <memory>
#include <thread>
#include "tortoisebot_action_server.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"
#include <math.h>

#define PI 3.1416
using WaypointAction = tortoisebot_waypoints::action::WaypointAction;
using GoalHandleWaypointAction =  rclcpp_action::ServerGoalHandle<WaypointAction>;
WaypointActionClass::WaypointActionClass(const rclcpp::NodeOptions &options )
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

std::tuple<geometry_msgs::msg::Point, double> WaypointActionClass::get_current_robot_position_and_yawrad(){
      return {current_pos_, current_yaw_rad_};
}

double WaypointActionClass::get_desire_pos_angle_yawrad(){
    return  this->desire_pos_angle_yawrad;
}

rclcpp_action::GoalResponse
  WaypointActionClass::handle_goal(const rclcpp_action::GoalUUID &uuid,
              std::shared_ptr<const WaypointAction::Goal> goal) {
    RCLCPP_INFO(this->get_logger(), "Received goal position %f,%f",
                goal->position.x, goal->position.y);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse
  WaypointActionClass::handle_cancel(const std::shared_ptr<GoalHandleWaypointAction> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void
  WaypointActionClass::handle_accepted(const std::shared_ptr<GoalHandleWaypointAction> goal_handle) {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a
    // new thread
    std::thread{std::bind(&WaypointActionClass::execute, this, _1), goal_handle}
        .detach();
}

void WaypointActionClass::execute(const std::shared_ptr<GoalHandleWaypointAction> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<WaypointAction::Feedback>();
    auto &message = feedback->state;
    message = "Starting movement...";
    auto result = std::make_shared<WaypointAction::Result>();
    auto move = geometry_msgs::msg::Twist();
    rclcpp::Rate loop_rate(1);
    double cnst_speed =0.3;
    // int tiemout = 10;//seconds
    // Check if there is a cancel request

    // WaypointAction robot forward and send feedback
    RCLCPP_INFO(this->get_logger(),"goal %f,%f received", goal->position.x, goal->position.y);
    bool success = true;

    // define desired position and errors
    desire_pos_ = goal->position;
    this->desire_pos_angle_yawrad = atan2( desire_pos_.y - current_pos_.y, desire_pos_.x -  current_pos_.x);
    double err_pos = sqrt((( desire_pos_.y - current_pos_.y)*( desire_pos_.y - current_pos_.y)) +((desire_pos_.x -  current_pos_.x)*(desire_pos_.x -  current_pos_.x)));
    double err_yaw = this->desire_pos_angle_yawrad - current_yaw_rad_;


    // perform task
    while (err_pos > _dist_precision && success){
        // update vars
        this->desire_pos_angle_yawrad = atan2( desire_pos_.y - current_pos_.y, desire_pos_.x -  current_pos_.x);
        err_yaw = this->desire_pos_angle_yawrad - current_yaw_rad_;
        err_pos = sqrt((( desire_pos_.y - current_pos_.y)*( desire_pos_.y - current_pos_.y)) +((desire_pos_.x -  current_pos_.x)*(desire_pos_.x -  current_pos_.x)));
        RCLCPP_INFO(this->get_logger(),"Current Yaw: %f", current_yaw_rad_);
        RCLCPP_INFO(this->get_logger(),"Error pos: %f, threshold %f", err_pos, _dist_precision);
        RCLCPP_INFO(this->get_logger(),"Desired Yaw: %f", this->desire_pos_angle_yawrad );
        RCLCPP_INFO(this->get_logger(),"Error Yaw: %f", err_yaw);
        // logic goes here
        if (goal_handle->is_canceling()) {
        success = false;
        result->success = false; // message;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
        } else if (abs(err_yaw) > _yaw_precision){
            // fix yaw
            RCLCPP_INFO(this->get_logger(),"fix yaw");
            this->_state = "fix yaw";           
            if (err_yaw > 0) 
                move.angular.z = cnst_speed;
            else 
                move.angular.z = -cnst_speed;
            publisher_->publish(move);
        }else{
            // go to point
            RCLCPP_INFO(this->get_logger(),"go to point");
            this->_state = "go to point";          
            move.linear.x = cnst_speed;
            move.angular.z = 0;
            // move.angular.z = 0.1 if err_yaw > 0 else -0.1
            publisher_->publish(move);
        }
        // loop rate
        loop_rate.sleep();
        // send feedback
        feedback->position = current_pos_;
        feedback->state = this->_state;
        goal_handle->publish_feedback(feedback);
        RCLCPP_INFO(this->get_logger(), "Publish feedback");
        
    }
    // stop
    move.linear.x = 0.0;
    move.angular.z = 0.0;
    publisher_->publish(move);
    // Check if goal is done
    if (success) {
      result->success = true; // "Finished action server. Robot moved during 5 seconds";
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
}

void WaypointActionClass::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_pos_ = msg->pose.pose.position;
    current_angle_ = msg->pose.pose.orientation;
    current_yaw_rad_ = yaw_theta_from_quaternion(
        msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);

    RCLCPP_DEBUG(this->get_logger(), "current pos=['%f','%f','%f'",
                 current_pos_.x, current_pos_.y, current_yaw_rad_);
}


double WaypointActionClass::yaw_theta_from_quaternion(float qx, float qy, float qz, float qw) {
    double roll_rad, pitch_rad, yaw_rad;
    tf2::Quaternion odom_quat(qx, qy, qz, qw);
    tf2::Matrix3x3 matrix_tf(odom_quat);
    matrix_tf.getRPY(roll_rad, pitch_rad, yaw_rad);
    return yaw_rad; // In radian
}



int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto action_server = std::make_shared<WaypointActionClass>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(action_server);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}