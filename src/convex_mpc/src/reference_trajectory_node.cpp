#include <memory>
#include <vector>
#include <cmath>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

enum class TrajectoryType {
  STRAIGHT,
  CIRCLE,
  SQUARE
};

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

    // Trajectory pattern parameters

    this->declare_parameter<std::string>("trajectory_type", "square");  // "straight", "circle", "square"
    this->declare_parameter<double>("circle_radius", 1.0);  // meters
    this->declare_parameter<double>("square_side_length", 2.0);  // meters

    N_STATES_ = this->get_parameter("N_STATES").as_int();
    N_MPC_    = this->get_parameter("N_MPC").as_int();
    dt_       = this->get_parameter("dt").as_double();
    vx_ref_   = this->get_parameter("vx_ref").as_double();
    vy_ref_   = this->get_parameter("vy_ref").as_double();
    wz_ref_   = this->get_parameter("wz_ref").as_double();
    
    // Parse trajectory type
    std::string traj_type_str = this->get_parameter("trajectory_type").as_string();
    if (traj_type_str == "circle") {
      trajectory_type_ = TrajectoryType::CIRCLE;
    } else if (traj_type_str == "square") {
      trajectory_type_ = TrajectoryType::SQUARE;
    } else {
      trajectory_type_ = TrajectoryType::STRAIGHT;
    }
    
    circle_radius_ = this->get_parameter("circle_radius").as_double();
    square_side_length_ = this->get_parameter("square_side_length").as_double();

    x0_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/x0", 10,
      std::bind(&ReferenceTrajectoryNode::x0Callback, this, std::placeholders::_1));

    ref_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/reference_trajectory", 10);

    std::string traj_name = (trajectory_type_ == TrajectoryType::CIRCLE) ? "CIRCLE" :
                            (trajectory_type_ == TrajectoryType::SQUARE) ? "SQUARE" : "STRAIGHT";
    RCLCPP_INFO(get_logger(),
      "ReferenceTrajectoryNode started (N_STATES=%d, N_MPC=%d, dt=%.3f, type=%s, vx=%.2f, vy=%.2f, wz=%.2f)",
      N_STATES_, N_MPC_, dt_, traj_name.c_str(), vx_ref_, vy_ref_, wz_ref_);
    if (trajectory_type_ == TrajectoryType::CIRCLE) {
      RCLCPP_INFO(get_logger(), "  Circle radius: %.2f m", circle_radius_);
    } else if (trajectory_type_ == TrajectoryType::SQUARE) {
      RCLCPP_INFO(get_logger(), "  Square side length: %.2f m", square_side_length_);
    }
  }

private:
  void x0Callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (static_cast<int>(msg->data.size()) != N_STATES_) {
      RCLCPP_ERROR(get_logger(), "x0 size %zu != N_STATES %d",
                   msg->data.size(), N_STATES_);
      return;
    }

    std::vector<double> current_x0(msg->data.begin(), msg->data.end());

    RCLCPP_INFO(get_logger(), "Received x0: [x=%.3f, y=%.3f, z=%.3f, yaw=%.3f]",
                current_x0[3], current_x0[4], current_x0[5], current_x0[2]);
    
    // Store initial circle center/radius on first x0 for circle trajectories
    if (!initial_state_received_ && trajectory_type_ == TrajectoryType::CIRCLE) {
      const int IDX_YAW = 2;
      const int IDX_X   = 3;
      const int IDX_Y   = 4;
      double initial_yaw = current_x0[IDX_YAW];
      double initial_x = current_x0[IDX_X];
      double initial_y = current_x0[IDX_Y];
      
      // Store circle center based on initial position
      circle_center_x_ = initial_x - std::sin(initial_yaw) * circle_radius_;
      circle_center_y_ = initial_y + std::cos(initial_yaw) * circle_radius_;
      RCLCPP_INFO(get_logger(), "Initial circle center set: (%.3f, %.3f), radius: %.3f",
                  circle_center_x_, circle_center_y_, circle_radius_);
    }
    
    // Store initial square corner on first x0 for square trajectories
    if (!initial_state_received_ && trajectory_type_ == TrajectoryType::SQUARE) {
      const int IDX_X   = 3;
      const int IDX_Y   = 4;
      double initial_x = current_x0[IDX_X];
      double initial_y = current_x0[IDX_Y];
      
      // Store square corner (bottom-left) based on initial position
      square_corner_x_ = initial_x;
      square_corner_y_ = initial_y;
      RCLCPP_INFO(get_logger(), "Initial square corner set: (%.3f, %.3f), side length: %.3f",
                  square_corner_x_, square_corner_y_, square_side_length_);
    }
    
    initial_state_received_ = true;
    generateAndPublishReference(current_x0);
  }

  void generateAndPublishReference(const std::vector<double>& x)
  {
    if (static_cast<int>(x.size()) != N_STATES_) {
      RCLCPP_ERROR(get_logger(), "Stored x0 size %zu != N_STATES %d",
                   x.size(), N_STATES_);
      return;
    }

    const int IDX_YAW = 2;
    const int IDX_X   = 3;
    const int IDX_Y   = 4;
    const int IDX_Z   = 5;

    std::vector<double> X_ref_flat;
    X_ref_flat.reserve(N_STATES_ * N_MPC_);

    // Get current state
    double current_yaw = x[IDX_YAW];
    double current_x = x[IDX_X];
    double current_y = x[IDX_Y];
    double current_z = x[IDX_Z];

    // Generate N_MPC states based on trajectory type
    switch (trajectory_type_) {
      case TrajectoryType::STRAIGHT:
        generateStraightTrajectory(X_ref_flat, x, current_x, current_y, current_z, current_yaw);
        break;
      case TrajectoryType::CIRCLE:
        generateCircleTrajectory(X_ref_flat, x, current_x, current_y, current_z, current_yaw);
        break;
      case TrajectoryType::SQUARE:
        generateSquareTrajectory(X_ref_flat, x, current_x, current_y, current_z, current_yaw);
        break;
    }

    // Publish immediately
    std_msgs::msg::Float64MultiArray msg;
    msg.data = std::move(X_ref_flat);
    ref_pub_->publish(msg);
    RCLCPP_INFO(get_logger(), "Published reference trajectory: %zu values (N_MPC=%d, N_STATES=%d)",
                msg.data.size(), N_MPC_, N_STATES_);
  }


  void generateStraightTrajectory(std::vector<double>& X_ref_flat, const std::vector<double>& x,
                                   double current_x, double current_y, double current_z, double current_yaw)
  {
    const int IDX_ROLL   = 0;
    const int IDX_PITCH  = 1;
    const int IDX_OMEGA_X = 6;
    const int IDX_OMEGA_Y = 7;
    const int IDX_G      = 12;

    for (int k = 1; k <= N_MPC_; ++k) {
      double t_k = k * dt_;
      
      // Update yaw based on yaw rate
      double yaw_k = current_yaw + wz_ref_ * t_k;
      
      // Transform body-frame velocities to world frame
      double cos_yaw_k = std::cos(yaw_k);
      double sin_yaw_k = std::sin(yaw_k);
      
      double world_vx = vx_ref_ * cos_yaw_k - vy_ref_ * sin_yaw_k;
      double world_vy = vx_ref_ * sin_yaw_k + vy_ref_ * cos_yaw_k;
      
      // Integrate position forward
      double x_k = current_x + world_vx * t_k;
      double y_k = current_y + world_vy * t_k;
      double z_k = current_z;

      appendState(X_ref_flat, x_k, y_k, z_k, yaw_k, world_vx, world_vy,
                  x[IDX_ROLL], x[IDX_PITCH], x[IDX_OMEGA_X], x[IDX_OMEGA_Y], wz_ref_, x[IDX_G]);
    }
  }

  void generateCircleTrajectory(std::vector<double>& X_ref_flat, const std::vector<double>& x,
                                 double current_x, double current_y, double current_z, double current_yaw)
  {
    const int IDX_ROLL   = 0;
    const int IDX_PITCH  = 1;
    const int IDX_OMEGA_X = 6;
    const int IDX_OMEGA_Y = 7;
    const int IDX_G      = 12;

    // Calculate linear speed
    double linear_speed = std::sqrt(vx_ref_ * vx_ref_ + vy_ref_ * vy_ref_);
    if (linear_speed < 0.01) {
      linear_speed = 0.5;  // Default speed if too small
    }
    
    // Calculate angular velocity from linear velocity: omega = v / r
    double angular_velocity = linear_speed / circle_radius_;
    
    // Use stored circle center (from initial position) but calculate angle from current position
    // Initial angle from the stored center to the current position
    double initial_angle = std::atan2(current_y - circle_center_y_,
                                      current_x - circle_center_x_);

    for (int k = 1; k <= N_MPC_; ++k) {
      double t_k = k * dt_;
      
      // Update angle around circle (counter-clockwise)
      double angle_k = initial_angle + angular_velocity * t_k;
      
      // Calculate position on circle using stored center
      double x_k = circle_center_x_ + circle_radius_ * std::cos(angle_k);
      double y_k = circle_center_y_ + circle_radius_ * std::sin(angle_k);
      double z_k = current_z;
      
      // Yaw is tangent to the circle (perpendicular to radius, pointing counter-clockwise)
      double yaw_k = angle_k + M_PI / 2.0;  // Tangent direction
      
      // Velocity is tangent to the circle (counter-clockwise)
      double world_vx = -linear_speed * std::sin(angle_k);
      double world_vy = linear_speed * std::cos(angle_k);
      
      // Yaw rate for circular motion
      double wz_k = angular_velocity;

      appendState(X_ref_flat, x_k, y_k, z_k, yaw_k, world_vx, world_vy,
                  x[IDX_ROLL], x[IDX_PITCH], x[IDX_OMEGA_X], x[IDX_OMEGA_Y], wz_k, x[IDX_G]);
    }
  }

  void generateSquareTrajectory(std::vector<double>& X_ref_flat, const std::vector<double>& x,
                                  double current_x, double current_y, double current_z, double current_yaw)
  {
    const int IDX_ROLL   = 0;
    const int IDX_PITCH  = 1;
    const int IDX_OMEGA_X = 6;
    const int IDX_OMEGA_Y = 7;
    const int IDX_G      = 12;

    // Calculate linear speed
    double linear_speed = std::sqrt(vx_ref_ * vx_ref_ + vy_ref_ * vy_ref_);
    if (linear_speed < 0.01) {
      linear_speed = 0.5;  // Default speed
    }
    
    // Use stored square corner (fixed from initial position)
    double corner_x = square_corner_x_;
    double corner_y = square_corner_y_;
    
    // Find closest point on square perimeter to current position and calculate initial distance
    double dx = current_x - corner_x;
    double dy = current_y - corner_y;
    double perimeter = 4.0 * square_side_length_;
    double initial_distance = 0.0;
    
    // Find which side of the square the current position is closest to
    // Side 0: bottom (y = corner_y, x from corner_x to corner_x + side_length) - moving right
    // Side 1: right (x = corner_x + side_length, y from corner_y to corner_y + side_length) - moving up
    // Side 2: top (y = corner_y + side_length, x from corner_x + side_length to corner_x) - moving left
    // Side 3: left (x = corner_x, y from corner_y + side_length to corner_y) - moving down
    
    // Calculate distances to each side
    double dist_to_bottom = std::abs(dy);
    double dist_to_right = std::abs(dx - square_side_length_);
    double dist_to_top = std::abs(dy - square_side_length_);
    double dist_to_left = std::abs(dx);
    
    // Find minimum distance
    double min_dist = std::min({dist_to_bottom, dist_to_right, dist_to_top, dist_to_left});
    
    // Project current position onto the closest side and calculate distance along perimeter
    if (min_dist == dist_to_bottom && dx >= -0.1 && dx <= square_side_length_ + 0.1) {
      // Closest to bottom side (side 0, moving right)
      double clamped_dx = std::max(0.0, std::min(square_side_length_, dx));
      initial_distance = clamped_dx;
    } else if (min_dist == dist_to_right && dy >= -0.1 && dy <= square_side_length_ + 0.1) {
      // Closest to right side (side 1, moving up)
      double clamped_dy = std::max(0.0, std::min(square_side_length_, dy));
      initial_distance = square_side_length_ + clamped_dy;
    } else if (min_dist == dist_to_top && dx >= -0.1 && dx <= square_side_length_ + 0.1) {
      // Closest to top side (side 2, moving left)
      double clamped_dx = std::max(0.0, std::min(square_side_length_, dx));
      initial_distance = 2.0 * square_side_length_ + (square_side_length_ - clamped_dx);
    } else if (min_dist == dist_to_left && dy >= -0.1 && dy <= square_side_length_ + 0.1) {
      // Closest to left side (side 3, moving down)
      double clamped_dy = std::max(0.0, std::min(square_side_length_, dy));
      initial_distance = 3.0 * square_side_length_ + (square_side_length_ - clamped_dy);
    } else {
      // Default to corner if ambiguous
      initial_distance = 0.0;
    }

    for (int k = 1; k <= N_MPC_; ++k) {
      double t_k = k * dt_;
      
      // Calculate total distance traveled along the square perimeter starting from current position
      double total_distance = initial_distance + linear_speed * t_k;
      
      // Normalize to one square perimeter
      double normalized_distance = std::fmod(total_distance, perimeter);
      
      // Determine which side we're on (0-3) and distance along that side
      int side_index = static_cast<int>(normalized_distance / square_side_length_);
      double side_distance = std::fmod(normalized_distance, square_side_length_);
      
      double x_k, y_k, yaw_k, world_vx, world_vy;
      
      // Calculate position and orientation based on current side
      // Square goes: right (+x) -> up (+y) -> left (-x) -> down (-y)
      switch (side_index) {
        case 0:  // Moving right (+x)
          x_k = corner_x + side_distance;
          y_k = corner_y;
          yaw_k = 0.0;  // Pointing east
          world_vx = linear_speed;
          world_vy = 0.0;
          break;
        case 1:  // Moving up (+y)
          x_k = corner_x + square_side_length_;
          y_k = corner_y + side_distance;
          yaw_k = M_PI / 2.0;  // Pointing north
          world_vx = 0.0;
          world_vy = linear_speed;
          break;
        case 2:  // Moving left (-x)
          x_k = corner_x + square_side_length_ - side_distance;
          y_k = corner_y + square_side_length_;
          yaw_k = M_PI;  // Pointing west
          world_vx = -linear_speed;
          world_vy = 0.0;
          break;
        case 3:  // Moving down (-y)
          x_k = corner_x;
          y_k = corner_y + square_side_length_ - side_distance;
          yaw_k = -M_PI / 2.0;  // Pointing south
          world_vx = 0.0;
          world_vy = -linear_speed;
          break;
        default:
          x_k = current_x;
          y_k = current_y;
          yaw_k = current_yaw;
          world_vx = 0.0;
          world_vy = 0.0;
      }
      
      double z_k = current_z;
      double wz_k = 0.0;  // No yaw rate during straight segments

      appendState(X_ref_flat, x_k, y_k, z_k, yaw_k, world_vx, world_vy,
                  x[IDX_ROLL], x[IDX_PITCH], x[IDX_OMEGA_X], x[IDX_OMEGA_Y], wz_k, x[IDX_G]);
    }
  }

  void appendState(std::vector<double>& X_ref_flat,
                   double x_pos, double y_pos, double z_pos, double yaw,
                   double vx, double vy, double roll, double pitch,
                   double omega_x, double omega_y, double omega_z, double g)
  {
    // State: [roll, pitch, yaw, x, y, z, omega_x, omega_y, omega_z, vx, vy, vz, g]
    X_ref_flat.push_back(roll);
    X_ref_flat.push_back(pitch);
    X_ref_flat.push_back(yaw);
    X_ref_flat.push_back(x_pos);
    X_ref_flat.push_back(y_pos);
    X_ref_flat.push_back(z_pos);
    X_ref_flat.push_back(omega_x);
    X_ref_flat.push_back(omega_y);
    X_ref_flat.push_back(omega_z);
    X_ref_flat.push_back(vx);
    X_ref_flat.push_back(vy);
    X_ref_flat.push_back(0.0);  // vz (zero)
    X_ref_flat.push_back(g);
  }

  int N_STATES_;
  int N_MPC_;
  double dt_;
  double vx_ref_, vy_ref_, wz_ref_;
  TrajectoryType trajectory_type_;
  double circle_radius_;
  double square_side_length_;
  
  // Circle center stored from initial position
  double circle_center_x_{0.0};
  double circle_center_y_{0.0};
  
  // Square corner stored from initial position
  double square_corner_x_{0.0};
  double square_corner_y_{0.0};

  bool initial_state_received_{false};
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
