#include <memory>
#include <vector>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

class MockX0Node : public rclcpp::Node
{
public:
  MockX0Node()
  : Node("mock_x0_node")
  {
    // Parameters
    this->declare_parameter<int>("N_STATES", 13);
    N_STATES_ = this->get_parameter("N_STATES").as_int();
    
    // Publisher for x0
    x0_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/x0", 10);
    
    // Subscriber for reference trajectory
    ref_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/reference_trajectory", 10,
      std::bind(&MockX0Node::referenceCallback, this, std::placeholders::_1));
    
    // Publish initial x0 after a short delay to ensure subscribers are ready
    initial_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      [this]() {
        publishInitialX0();
        initial_timer_->cancel();
      });
    
    RCLCPP_INFO(get_logger(), "MockX0Node started (N_STATES=%d)", N_STATES_);
  }

private:
  void publishInitialX0()
  {
    std_msgs::msg::Float64MultiArray msg;
    msg.data.resize(N_STATES_, 0.0);
    
    // Set initial position: x=0, y=0, z=0.3
    msg.data[3] = 0.0;  // x
    msg.data[4] = 0.0;  // y
    msg.data[5] = 0.3;  // z
    
    // Set gravity (typically 9.81)
    msg.data[12] = 9.81;  // g
    
    x0_pub_->publish(msg);
    RCLCPP_INFO(get_logger(), "Published initial x0: [x=%.3f, y=%.3f, z=%.3f]",
                msg.data[3], msg.data[4], msg.data[5]);
  }
  
  void referenceCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    RCLCPP_INFO(get_logger(), "Received reference trajectory with %zu values", msg->data.size());
    
    // Extract first state from reference trajectory (first N_STATES values)
    if (static_cast<int>(msg->data.size()) < N_STATES_) {
      RCLCPP_WARN(get_logger(), "Reference trajectory too short: %zu < %d",
                  msg->data.size(), N_STATES_);
      return;
    }
    
    // Log first state values from reference
    RCLCPP_INFO(get_logger(), "First state in reference: [x=%.3f, y=%.3f, z=%.3f, yaw=%.3f]",
                msg->data[3], msg->data[4], msg->data[5], msg->data[2]);
    
    // Create new x0 message with first state from reference
    std_msgs::msg::Float64MultiArray x0_msg;
    x0_msg.data.assign(msg->data.begin(), msg->data.begin() + N_STATES_);
    
    x0_pub_->publish(x0_msg);
    update_count_++;
    
    RCLCPP_INFO(get_logger(), "Published x0[%d] from reference (first state): [x=%.3f, y=%.3f, z=%.3f, yaw=%.3f]",
                update_count_, x0_msg.data[3], x0_msg.data[4], x0_msg.data[5], x0_msg.data[2]);
  }
  
  int N_STATES_;
  int update_count_{0};
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr x0_pub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr ref_sub_;
  rclcpp::TimerBase::SharedPtr initial_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MockX0Node>());
  rclcpp::shutdown();
  return 0;
}

