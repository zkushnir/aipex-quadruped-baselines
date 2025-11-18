#include <memory>
#include <vector>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

class ReferenceTrajectoryNode : public rclcpp::Node
{
public:
  ReferenceTrajectoryNode()
  : Node("reference_trajectory_node")
  {
    // Same as MPC params, or read from YAML
    this->declare_parameter<int>("N_STATES", 13);
    this->declare_parameter<int>("N_MPC", 10);
    this->declare_parameter<double>("dt", 0.05);
    this->declare_parameter<double>("vx_ref", 0.5);   // m/s forward
    this->declare_parameter<double>("vy_ref", 0.0);   // m/s lateral
    this->declare_parameter<double>("wz_ref", 0.0);   // rad/s yaw

    N_STATES_ = this->get_parameter("N_STATES").as_int();
    N_MPC_    = this->get_parameter("N_MPC").as_int();
    dt_       = this->get_parameter("dt").as_double();
    vx_ref_   = this->get_parameter("vx_ref").as_double();
    vy_ref_   = this->get_parameter("vy_ref").as_double();
    wz_ref_   = this->get_parameter("wz_ref").as_double();

    x0_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/x0", 10,
      std::bind(&ReferenceTrajectoryNode::x0Callback, this, std::placeholders::_1));

    ref_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/reference_trajectory", 10);

    RCLCPP_INFO(get_logger(),
      "ReferenceTrajectoryNode started (N_STATES=%d, N_MPC=%d, dt=%.3f, vx=%.2f, vy=%.2f, wz=%.2f)",
      N_STATES_, N_MPC_, dt_, vx_ref_, vy_ref_, wz_ref_);
  }

private:
  void x0Callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (static_cast<int>(msg->data.size()) != N_STATES_) {
      RCLCPP_ERROR(get_logger(), "x0 size %zu != N_STATES %d",
                   msg->data.size(), N_STATES_);
      return;
    }

    // Copy x0
    std::vector<double> x(msg->data.begin(), msg->data.end());

    // Indices from your state definition:
    const int IDX_ROLL  = 0;
    const int IDX_PITCH = 1;
    const int IDX_YAW   = 2;
    const int IDX_X     = 3;
    const int IDX_Y     = 4;
    const int IDX_Z     = 5;
    const int IDX_VX    = 9;
    const int IDX_VY    = 10;
    const int IDX_VZ    = 11;

    std::vector<double> X_ref_flat;
    X_ref_flat.reserve(N_STATES_ * N_MPC_);

    for (int k = 0; k < N_MPC_; ++k) {
      // Append current state as step k
      for (int i = 0; i < N_STATES_; ++i)
        X_ref_flat.push_back(x[i]);

      // Simple “go straight” reference from current pose
      double yaw = x[IDX_YAW];

      double world_vx =  vx_ref_ * std::cos(yaw) - vy_ref_ * std::sin(yaw);
      double world_vy =  vx_ref_ * std::sin(yaw) + vy_ref_ * std::cos(yaw);

      x[IDX_X]  += world_vx * dt_;
      x[IDX_Y]  += world_vy * dt_;
      x[IDX_Z]   = x[IDX_Z];  // keep height (or set to fixed)
      x[IDX_YAW] += wz_ref_ * dt_;

      x[IDX_VX] = world_vx;
      x[IDX_VY] = world_vy;
      x[IDX_VZ] = 0.0;
    }

    std_msgs::msg::Float64MultiArray out;
    out.data = std::move(X_ref_flat);
    ref_pub_->publish(out);
  }

  int N_STATES_;
  int N_MPC_;
  double dt_;
  double vx_ref_, vy_ref_, wz_ref_;

  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr x0_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr ref_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ReferenceTrajectoryNode>());
  rclcpp::shutdown();
  return 0;
}
