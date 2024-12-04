#include "geometry_msgs/msg/detail/point__struct.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "gtest/gtest.h"
#include <memory>
#include "tortoisebot_action_client.h"
#include "tortoisebot_action_server.h"
#define PI 3.1416
#define GOAL_X 1.8
#define GOAL_Y 2
class RclCppFixture {
public:
  RclCppFixture() { rclcpp::init(0, nullptr); }
  ~RclCppFixture() { rclcpp::shutdown(); }
};
RclCppFixture g_rclcppfixture;

class PubSubscriberTestFixture : public ::testing::Test {
public:
    PubSubscriberTestFixture() {
        std::cerr << "PubSubscriberTestFixture()" << std::endl;
        // 1 create server node
        this->action_server = std::make_shared<WaypointActionClass>();       
        executor_server.add_node(action_server);
 
        std::cerr << "publisherSubscriberTest()" << std::endl;
        std::tuple<geometry_msgs::msg::Point, double, double>desire_current_robot_position_and_yawrad;  
        this->action_client = std::make_shared<WaypointActionClient>(this->goal_x,this->goal_y);
        executor_client.add_node(this->action_client);

        desire_current_robot_position_and_yawrad = this->action_server->get_desire_current_robot_position_and_yawrad();  
        geometry_msgs::msg::Point start_position = std::get<0>(desire_current_robot_position_and_yawrad);
        this->start_position_x = start_position.x;
        this->start_position_y = start_position.y;
        this->start_yawrad = std::get<1>(desire_current_robot_position_and_yawrad);
        this->desire_yawrad = atan2( GOAL_Y - this->start_position_y , GOAL_X -  this->start_position_x );

        while (!action_client->is_goal_done()) {
            executor_server.spin_some();
            executor_client.spin_some();
        }
        //executor_client.remove_node(action_client);
        desire_current_robot_position_and_yawrad = this->action_server->get_desire_current_robot_position_and_yawrad();  
        geometry_msgs::msg::Point result_position = std::get<0>(desire_current_robot_position_and_yawrad);
        this->result_position_x = result_position.x;
        this->result_position_y = result_position.y;
        this->result_yawrad = std::get<1>(desire_current_robot_position_and_yawrad);
        //this->desire_yawrad = std::get<2>(desire_current_robot_position_and_yawrad);
        
        //return {result_position.x,result_position.y,result_yawrad,desire_yawrad};
    }

    void TestBody(){
    std::cerr << "TestBody()" << std::endl;
    }
 double goal_x = GOAL_X, goal_y = GOAL_Y;
 double result_position_x, result_position_y, result_yawrad, desire_yawrad;
 double start_position_x, start_position_y, start_yawrad;
protected:
  //server node   //client node
 std::shared_ptr<WaypointActionClass> action_server;
 std::shared_ptr<WaypointActionClient> action_client;
 rclcpp::executors::MultiThreadedExecutor executor_server;
 rclcpp::executors::MultiThreadedExecutor executor_client;



};


TEST_F(PubSubscriberTestFixture, SimpleTest) {
  double epsilon = 0.25;
  auto fixture_instance = std::make_shared<PubSubscriberTestFixture>();
  double result_position_x = fixture_instance->result_position_x;
  double result_position_y = fixture_instance->result_position_y;
  double result_yawrad =  fixture_instance->result_yawrad;
  double desire_yawrad =  fixture_instance->desire_yawrad;
  std::cerr<<'goal x'<<GOAL_X <<'result x'<<result_position_x<<'epsilon'<<epsilon<<std::endl;
  EXPECT_NEAR(GOAL_X, result_position_x, epsilon);
  std::cerr<<'goal y'<<GOAL_Y <<'result y'<<result_position_y<<'epsilon'<<epsilon<<std::endl;
  EXPECT_NEAR(GOAL_Y, result_position_y, epsilon);
  std::cerr<<'desire_yawrad'<<desire_yawrad <<'result_yawrad'<<result_yawrad<<'epsilon'<<epsilon<<std::endl;
  EXPECT_NEAR(desire_yawrad, result_yawrad, epsilon);
}