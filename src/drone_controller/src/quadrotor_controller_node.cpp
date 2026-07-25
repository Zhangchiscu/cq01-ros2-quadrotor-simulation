#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

class QuadrotorControllerNode : public rclcpp::Node
{
public:
  QuadrotorControllerNode()
  : Node("quadrotor_controller")
  {
    declareParameters();
    loadParameters();

    inertia_ = Eigen::Matrix3d::Zero();
    inertia_(0, 0) = inertia_xx_;
    inertia_(1, 1) = inertia_yy_;
    inertia_(2, 2) = inertia_zz_;

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/drone/odom",
      rclcpp::QoS(20),
      std::bind(
        &QuadrotorControllerNode::odomCallback,
        this,
        std::placeholders::_1));

    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/drone/goal",
      rclcpp::QoS(10),
      std::bind(
        &QuadrotorControllerNode::goalCallback,
        this,
        std::placeholders::_1));

    motor_command_pub_ =
      create_publisher<std_msgs::msg::Float32MultiArray>(
      "/drone/motor_rpm_cmd",
      rclcpp::QoS(20));

    error_pub_ =
      create_publisher<geometry_msgs::msg::Vector3Stamped>(
      "/drone/position_error",
      rclcpp::QoS(20));

    debug_pub_ =
      create_publisher<std_msgs::msg::Float32MultiArray>(
      "/drone/control_debug",
      rclcpp::QoS(20));

    const auto timer_period =
      std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / control_rate_hz_));

    last_control_time_ = now();

    control_timer_ = create_wall_timer(
      timer_period,
      std::bind(&QuadrotorControllerNode::controlStep, this));

    const double hover_omega =
      std::sqrt(mass_ * gravity_ / (4.0 * thrust_coefficient_));

    const double hover_rpm =
      hover_omega * 60.0 / (2.0 * kPi);

    RCLCPP_INFO(get_logger(), "Quadrotor controller node started");
    RCLCPP_INFO(
      get_logger(),
      "Controller rate: %.1f Hz",
      control_rate_hz_);
    RCLCPP_INFO(
      get_logger(),
      "Waiting for /drone/odom and /drone/goal");
    RCLCPP_INFO(
      get_logger(),
      "Nominal hover RPM: %.2f",
      hover_rpm);
  }

private:
  static constexpr double kPi =
    3.14159265358979323846;

  void declareParameters()
  {
    declare_parameter<double>("control_rate_hz", 100.0);
    declare_parameter<double>("odom_timeout_s", 0.30);

    declare_parameter<double>("mass", 1.36);
    declare_parameter<double>("gravity", 9.81);
    declare_parameter<double>("arm_length", 0.225);

    declare_parameter<double>("inertia_xx", 0.030);
    declare_parameter<double>("inertia_yy", 0.030);
    declare_parameter<double>("inertia_zz", 0.050);

    declare_parameter<double>("thrust_coefficient", 8.54858e-6);
    declare_parameter<double>("torque_coefficient", 1.60e-7);

    declare_parameter<double>("minimum_rpm", 0.0);
    declare_parameter<double>("maximum_rpm", 10000.0);

    declare_parameter<double>("kp_x", 2.0);
    declare_parameter<double>("kp_y", 2.0);
    declare_parameter<double>("kp_z", 5.0);

    declare_parameter<double>("kd_x", 2.4);
    declare_parameter<double>("kd_y", 2.4);
    declare_parameter<double>("kd_z", 3.5);

    declare_parameter<double>("ki_x", 0.05);
    declare_parameter<double>("ki_y", 0.05);
    declare_parameter<double>("ki_z", 0.25);

    declare_parameter<double>("integral_limit_x", 1.0);
    declare_parameter<double>("integral_limit_y", 1.0);
    declare_parameter<double>("integral_limit_z", 1.0);

    declare_parameter<double>("attitude_gain_roll", 1.2);
    declare_parameter<double>("attitude_gain_pitch", 1.2);
    declare_parameter<double>("attitude_gain_yaw", 0.40);

    declare_parameter<double>("angular_rate_gain_roll", 0.25);
    declare_parameter<double>("angular_rate_gain_pitch", 0.25);
    declare_parameter<double>("angular_rate_gain_yaw", 0.12);

    declare_parameter<double>("maximum_horizontal_acceleration", 3.0);
    declare_parameter<double>("maximum_upward_acceleration", 4.0);
    declare_parameter<double>("maximum_downward_acceleration", 4.0);
    declare_parameter<double>("maximum_tilt_deg", 30.0);

    declare_parameter<double>("maximum_roll_torque", 0.80);
    declare_parameter<double>("maximum_pitch_torque", 0.80);
    declare_parameter<double>("maximum_yaw_torque", 0.08);

    declare_parameter<double>("minimum_goal_z", 0.0);
    declare_parameter<double>("maximum_goal_z", 300.0);
  }

  void loadParameters()
  {
    control_rate_hz_ =
      get_parameter("control_rate_hz").as_double();

    odom_timeout_s_ =
      get_parameter("odom_timeout_s").as_double();

    mass_ = get_parameter("mass").as_double();
    gravity_ = get_parameter("gravity").as_double();
    arm_length_ = get_parameter("arm_length").as_double();

    inertia_xx_ = get_parameter("inertia_xx").as_double();
    inertia_yy_ = get_parameter("inertia_yy").as_double();
    inertia_zz_ = get_parameter("inertia_zz").as_double();

    thrust_coefficient_ =
      get_parameter("thrust_coefficient").as_double();

    torque_coefficient_ =
      get_parameter("torque_coefficient").as_double();

    minimum_rpm_ = get_parameter("minimum_rpm").as_double();
    maximum_rpm_ = get_parameter("maximum_rpm").as_double();

    kp_position_ = Eigen::Vector3d(
      get_parameter("kp_x").as_double(),
      get_parameter("kp_y").as_double(),
      get_parameter("kp_z").as_double());

    kd_position_ = Eigen::Vector3d(
      get_parameter("kd_x").as_double(),
      get_parameter("kd_y").as_double(),
      get_parameter("kd_z").as_double());

    ki_position_ = Eigen::Vector3d(
      get_parameter("ki_x").as_double(),
      get_parameter("ki_y").as_double(),
      get_parameter("ki_z").as_double());

    integral_limit_ = Eigen::Vector3d(
      get_parameter("integral_limit_x").as_double(),
      get_parameter("integral_limit_y").as_double(),
      get_parameter("integral_limit_z").as_double());

    attitude_gain_ = Eigen::Vector3d(
      get_parameter("attitude_gain_roll").as_double(),
      get_parameter("attitude_gain_pitch").as_double(),
      get_parameter("attitude_gain_yaw").as_double());

    angular_rate_gain_ = Eigen::Vector3d(
      get_parameter("angular_rate_gain_roll").as_double(),
      get_parameter("angular_rate_gain_pitch").as_double(),
      get_parameter("angular_rate_gain_yaw").as_double());

    maximum_horizontal_acceleration_ =
      get_parameter("maximum_horizontal_acceleration").as_double();

    maximum_upward_acceleration_ =
      get_parameter("maximum_upward_acceleration").as_double();

    maximum_downward_acceleration_ =
      get_parameter("maximum_downward_acceleration").as_double();

    maximum_tilt_rad_ =
      get_parameter("maximum_tilt_deg").as_double() *
      kPi / 180.0;

    maximum_torque_ = Eigen::Vector3d(
      get_parameter("maximum_roll_torque").as_double(),
      get_parameter("maximum_pitch_torque").as_double(),
      get_parameter("maximum_yaw_torque").as_double());

    minimum_goal_z_ =
      get_parameter("minimum_goal_z").as_double();

    maximum_goal_z_ =
      get_parameter("maximum_goal_z").as_double();

    if (control_rate_hz_ <= 0.0 ||
      mass_ <= 0.0 ||
      gravity_ <= 0.0 ||
      arm_length_ <= 0.0 ||
      inertia_xx_ <= 0.0 ||
      inertia_yy_ <= 0.0 ||
      inertia_zz_ <= 0.0 ||
      thrust_coefficient_ <= 0.0 ||
      torque_coefficient_ <= 0.0 ||
      maximum_rpm_ <= minimum_rpm_)
    {
      throw std::runtime_error(
              "Controller parameters contain invalid values");
    }
  }

  void odomCallback(
    const nav_msgs::msg::Odometry::SharedPtr message)
  {
    position_world_ = Eigen::Vector3d(
      message->pose.pose.position.x,
      message->pose.pose.position.y,
      message->pose.pose.position.z);

    Eigen::Quaterniond orientation(
      message->pose.pose.orientation.w,
      message->pose.pose.orientation.x,
      message->pose.pose.orientation.y,
      message->pose.pose.orientation.z);

    if (orientation.norm() < 1.0e-8) {
      return;
    }

    orientation.normalize();
    rotation_body_to_world_ = orientation.toRotationMatrix();

    const Eigen::Vector3d velocity_body(
      message->twist.twist.linear.x,
      message->twist.twist.linear.y,
      message->twist.twist.linear.z);

    velocity_world_ =
      rotation_body_to_world_ * velocity_body;

    angular_velocity_body_ = Eigen::Vector3d(
      message->twist.twist.angular.x,
      message->twist.twist.angular.y,
      message->twist.twist.angular.z);

    odom_received_ = true;
    last_odom_time_ = now();
  }

  void goalCallback(
    const geometry_msgs::msg::PoseStamped::SharedPtr message)
  {
    Eigen::Vector3d requested_goal(
      message->pose.position.x,
      message->pose.position.y,
      message->pose.position.z);

    if (!requested_goal.allFinite()) {
      RCLCPP_WARN(
        get_logger(),
        "Rejected goal containing NaN or Inf");
      return;
    }

    requested_goal.z() =
      std::clamp(
      requested_goal.z(),
      minimum_goal_z_,
      maximum_goal_z_);

    Eigen::Quaterniond goal_orientation(
      message->pose.orientation.w,
      message->pose.orientation.x,
      message->pose.orientation.y,
      message->pose.orientation.z);

    double requested_yaw = 0.0;

    if (goal_orientation.norm() > 1.0e-8) {
      goal_orientation.normalize();

      const double w = goal_orientation.w();
      const double x = goal_orientation.x();
      const double y = goal_orientation.y();
      const double z = goal_orientation.z();

      requested_yaw = std::atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z));
    }

    const double yaw_difference =
      std::atan2(
      std::sin(requested_yaw - goal_yaw_),
      std::cos(requested_yaw - goal_yaw_));

    const bool goal_changed =
      !goal_received_ ||
      (requested_goal - goal_position_world_).norm() > 1.0e-4 ||
      std::abs(yaw_difference) > 1.0e-4;

    goal_position_world_ = requested_goal;
    goal_yaw_ = requested_yaw;
    goal_received_ = true;

    if (goal_changed) {
      position_integral_.setZero();

      RCLCPP_INFO(
        get_logger(),
        "New goal: x=%.2f, y=%.2f, z=%.2f, yaw=%.1f deg",
        goal_position_world_.x(),
        goal_position_world_.y(),
        goal_position_world_.z(),
        goal_yaw_ * 180.0 / kPi);
    }
  }

  void publishZeroMotorCommand()
  {
    std_msgs::msg::Float32MultiArray command;
    command.data = {0.0F, 0.0F, 0.0F, 0.0F};
    motor_command_pub_->publish(command);
  }

  void controlStep()
  {
    const rclcpp::Time current_time = now();

    double dt =
      (current_time - last_control_time_).seconds();

    last_control_time_ = current_time;

    dt = std::clamp(
      dt,
      0.001,
      0.05);

    if (!odom_received_) {
      publishZeroMotorCommand();
      return;
    }

    if ((current_time - last_odom_time_).seconds() >
      odom_timeout_s_)
    {
      publishZeroMotorCommand();
      return;
    }

    if (!goal_received_) {
      publishZeroMotorCommand();
      return;
    }

    const Eigen::Vector3d position_error =
      goal_position_world_ - position_world_;

    position_integral_ += position_error * dt;

    for (int axis = 0; axis < 3; ++axis) {
      position_integral_(axis) =
        std::clamp(
        position_integral_(axis),
        -integral_limit_(axis),
        integral_limit_(axis));
    }

    Eigen::Vector3d desired_acceleration =
      kp_position_.cwiseProduct(position_error) +
      ki_position_.cwiseProduct(position_integral_) -
      kd_position_.cwiseProduct(velocity_world_);

    Eigen::Vector2d horizontal_acceleration(
      desired_acceleration.x(),
      desired_acceleration.y());

    if (horizontal_acceleration.norm() >
      maximum_horizontal_acceleration_)
    {
      horizontal_acceleration =
        horizontal_acceleration.normalized() *
        maximum_horizontal_acceleration_;

      desired_acceleration.x() =
        horizontal_acceleration.x();

      desired_acceleration.y() =
        horizontal_acceleration.y();
    }

    desired_acceleration.z() =
      std::clamp(
      desired_acceleration.z(),
      -maximum_downward_acceleration_,
      maximum_upward_acceleration_);

    Eigen::Vector3d desired_force_world =
      mass_ *
      (
      desired_acceleration +
      Eigen::Vector3d(0.0, 0.0, gravity_));

    const double horizontal_force =
      desired_force_world.head<2>().norm();

    const double maximum_horizontal_force_from_tilt =
      std::max(0.0, desired_force_world.z()) *
      std::tan(maximum_tilt_rad_);

    if (horizontal_force >
      maximum_horizontal_force_from_tilt &&
      horizontal_force > 1.0e-8)
    {
      desired_force_world.head<2>() *=
        maximum_horizontal_force_from_tilt /
        horizontal_force;
    }

    if (desired_force_world.norm() < 1.0e-8) {
      publishZeroMotorCommand();
      return;
    }

    const Eigen::Vector3d desired_body_z_world =
      desired_force_world.normalized();

    const Eigen::Vector3d heading_reference(
      std::cos(goal_yaw_),
      std::sin(goal_yaw_),
      0.0);

    Eigen::Vector3d desired_body_y_world =
      desired_body_z_world.cross(heading_reference);

    if (desired_body_y_world.norm() < 1.0e-8) {
      desired_body_y_world =
        Eigen::Vector3d(
        -std::sin(goal_yaw_),
        std::cos(goal_yaw_),
        0.0);
    }

    desired_body_y_world.normalize();

    const Eigen::Vector3d desired_body_x_world =
      desired_body_y_world.cross(
      desired_body_z_world).normalized();

    Eigen::Matrix3d desired_rotation;
    desired_rotation.col(0) = desired_body_x_world;
    desired_rotation.col(1) = desired_body_y_world;
    desired_rotation.col(2) = desired_body_z_world;

    const Eigen::Matrix3d attitude_error_matrix =
      0.5 *
      (
      desired_rotation.transpose() *
      rotation_body_to_world_ -
      rotation_body_to_world_.transpose() *
      desired_rotation);

    const Eigen::Vector3d attitude_error(
      attitude_error_matrix(2, 1),
      attitude_error_matrix(0, 2),
      attitude_error_matrix(1, 0));

    Eigen::Vector3d desired_torque_body =
      -attitude_gain_.cwiseProduct(attitude_error) -
      angular_rate_gain_.cwiseProduct(
      angular_velocity_body_) +
      angular_velocity_body_.cross(
      inertia_ * angular_velocity_body_);

    for (int axis = 0; axis < 3; ++axis) {
      desired_torque_body(axis) =
        std::clamp(
        desired_torque_body(axis),
        -maximum_torque_(axis),
        maximum_torque_(axis));
    }

    double total_thrust =
      desired_force_world.dot(
      rotation_body_to_world_.col(2));

    const double maximum_omega =
      maximum_rpm_ * 2.0 * kPi / 60.0;

    const double maximum_total_thrust =
      4.0 *
      thrust_coefficient_ *
      maximum_omega *
      maximum_omega;

    total_thrust =
      std::clamp(
      total_thrust,
      0.0,
      maximum_total_thrust);

    const double mixer_arm =
      arm_length_ / std::sqrt(2.0);

    const double yaw_force_ratio =
      torque_coefficient_ /
      thrust_coefficient_;

    const double roll_mix =
      desired_torque_body.x() / mixer_arm;

    const double pitch_mix =
      desired_torque_body.y() / mixer_arm;

    const double yaw_mix =
      desired_torque_body.z() / yaw_force_ratio;

    std::array<double, 4> motor_thrust{
      0.25 * (
        total_thrust +
        roll_mix -
        pitch_mix +
        yaw_mix),

      0.25 * (
        total_thrust -
        roll_mix -
        pitch_mix -
        yaw_mix),

      0.25 * (
        total_thrust -
        roll_mix +
        pitch_mix +
        yaw_mix),

      0.25 * (
        total_thrust +
        roll_mix +
        pitch_mix -
        yaw_mix)
    };

    std_msgs::msg::Float32MultiArray motor_command;
    motor_command.data.resize(4);

    for (std::size_t motor = 0; motor < 4; ++motor) {
      motor_thrust[motor] =
        std::max(0.0, motor_thrust[motor]);

      const double motor_omega =
        std::sqrt(
        motor_thrust[motor] /
        thrust_coefficient_);

      const double motor_rpm =
        motor_omega * 60.0 / (2.0 * kPi);

      motor_command.data[motor] =
        static_cast<float>(
        std::clamp(
          motor_rpm,
          minimum_rpm_,
          maximum_rpm_));
    }

    motor_command_pub_->publish(motor_command);

    geometry_msgs::msg::Vector3Stamped error_message;
    error_message.header.stamp = current_time;
    error_message.header.frame_id = "map";
    error_message.vector.x = position_error.x();
    error_message.vector.y = position_error.y();
    error_message.vector.z = position_error.z();
    error_pub_->publish(error_message);

    std_msgs::msg::Float32MultiArray debug_message;
    debug_message.data = {
      static_cast<float>(position_error.x()),
      static_cast<float>(position_error.y()),
      static_cast<float>(position_error.z()),
      static_cast<float>(total_thrust),
      static_cast<float>(desired_torque_body.x()),
      static_cast<float>(desired_torque_body.y()),
      static_cast<float>(desired_torque_body.z())
    };
    debug_pub_->publish(debug_message);
  }

  double control_rate_hz_{100.0};
  double odom_timeout_s_{0.30};

  double mass_{1.36};
  double gravity_{9.81};
  double arm_length_{0.225};

  double inertia_xx_{0.030};
  double inertia_yy_{0.030};
  double inertia_zz_{0.050};

  double thrust_coefficient_{8.54858e-6};
  double torque_coefficient_{1.60e-7};

  double minimum_rpm_{0.0};
  double maximum_rpm_{10000.0};

  Eigen::Vector3d kp_position_{2.0, 2.0, 5.0};
  Eigen::Vector3d kd_position_{2.4, 2.4, 3.5};
  Eigen::Vector3d ki_position_{0.05, 0.05, 0.25};
  Eigen::Vector3d integral_limit_{1.0, 1.0, 1.0};

  Eigen::Vector3d attitude_gain_{1.2, 1.2, 0.40};
  Eigen::Vector3d angular_rate_gain_{0.25, 0.25, 0.12};

  double maximum_horizontal_acceleration_{3.0};
  double maximum_upward_acceleration_{4.0};
  double maximum_downward_acceleration_{4.0};
  double maximum_tilt_rad_{30.0 * kPi / 180.0};

  Eigen::Vector3d maximum_torque_{0.80, 0.80, 0.08};

  double minimum_goal_z_{0.0};
  double maximum_goal_z_{300.0};

  Eigen::Matrix3d inertia_{Eigen::Matrix3d::Identity()};

  Eigen::Vector3d position_world_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity_world_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d angular_velocity_body_{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d rotation_body_to_world_{
    Eigen::Matrix3d::Identity()};

  Eigen::Vector3d goal_position_world_{
    Eigen::Vector3d::Zero()};

  double goal_yaw_{0.0};

  Eigen::Vector3d position_integral_{
    Eigen::Vector3d::Zero()};

  bool odom_received_{false};
  bool goal_received_{false};

  rclcpp::Time last_odom_time_;
  rclcpp::Time last_control_time_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
  odom_sub_;

  rclcpp::Subscription<
    geometry_msgs::msg::PoseStamped>::SharedPtr
  goal_sub_;

  rclcpp::Publisher<
    std_msgs::msg::Float32MultiArray>::SharedPtr
  motor_command_pub_;

  rclcpp::Publisher<
    geometry_msgs::msg::Vector3Stamped>::SharedPtr
  error_pub_;

  rclcpp::Publisher<
    std_msgs::msg::Float32MultiArray>::SharedPtr
  debug_pub_;

  rclcpp::TimerBase::SharedPtr control_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    rclcpp::spin(
      std::make_shared<QuadrotorControllerNode>());
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(
      rclcpp::get_logger("quadrotor_controller"),
      "Fatal error: %s",
      exception.what());

    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
