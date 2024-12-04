#include "geometry_msgs/msg/detail/point__struct.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "gtest/gtest.h"
#include <memory>
#include "tortoisebot_action_client.h"
#include "tortoisebot_action_server.h"
#define PI 3.1416
class RclCppFixture {
public:
  RclCppFixture() { rclcpp::init(0, nullptr); }
  ~RclCppFixture() { rclcpp::shutdown(); }
};
RclCppFixture g_rclcppfixture;

class PubSubscriberTestFixture : public ::testing::Test {
public:
  PubSubscriberTestFixture() {
    callback_success_ = false;

    // 1 create server node
    this->action_server = std::make_shared<WaypointActionClass>();
  
    executor_server.add_node(action_server);
    executor_server.spin();

    // 2 create client node
 //action_client = std::make_shared<WaypointActionClient>(goal_x,goal_y);
    //3 bind publisherSubscriberTest() to client send() 
            //use executor inside publisherSubscriberTest()
    //4 bind received_data_callback() callback to client node 
            //use get_current_pos from server node
        

  }

 std::tuple<double, double, double, double> publisherSubscriberTest(float ,float);

protected:
  //server node   //client node
 std::shared_ptr<WaypointActionClass> action_server;
 std::shared_ptr<WaypointActionClient> action_client;
 rclcpp::executors::MultiThreadedExecutor executor_server;
 rclcpp::executors::MultiThreadedExecutor executor_client;

private:
  double received_data_;
  bool callback_success_;
};

  std::tuple<double, double, double, double> PubSubscriberTestFixture::publisherSubscriberTest(float goal_x, float goal_y) {  
    std::tuple<geometry_msgs::msg::Point, double>current_robot_position_and_yawrad;  
    this->action_client = std::make_shared<WaypointActionClient>(goal_x,goal_y);
    executor_client.add_node(this->action_client);
    while (!action_client->is_goal_done()) {
        executor_client.spin_some();
    }
    executor_client.remove_node(action_client);
    current_robot_position_and_yawrad = this->action_server->get_current_robot_position_and_yawrad();  
    geometry_msgs::msg::Point result_position = std::get<0>(current_robot_position_and_yawrad);
    double result_yawrad = std::get<1>(current_robot_position_and_yawrad);
    double desire_yawrad = this->action_server->get_desire_pos_angle_yawrad();
    
    return {result_position.x,result_position.y,result_yawrad,desire_yawrad};
}

TEST_F(PubSubscriberTestFixture, SimpleTest) {
  std::tuple<double, double, double, double> answer;
  double epsilon = 0.1;
  answer = publisherSubscriberTest(0.0,2.0);
  double result_position_x = std::get<0>(answer);
  double result_position_y = std::get<1>(answer);
  double result_yawrad = std::get<2>(answer);
  double desire_yawrad = std::get<3>(answer);
  EXPECT_NEAR(0.0, result_position_x, epsilon);
  EXPECT_NEAR(2.0, result_position_y, epsilon);
  EXPECT_NEAR(desire_yawrad, result_yawrad, epsilon);
}