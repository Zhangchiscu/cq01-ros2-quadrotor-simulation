#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "tf2_ros/transform_broadcaster.h"

class QuadrotorDynamicsNode : public rclcpp::Node
{
public:
  QuadrotorDynamicsNode()
  : Node("quadrotor_dynamics"),
    position_world_(Eigen::Vector3d::Zero()),
    velocity_world_(Eigen::Vector3d::Zero()),
    orientation_body_to_world_(1.0, 0.0, 0.0, 0.0),
    angular_velocity_body_(Eigen::Vector3d::Zero()),
    acceleration_world_(Eigen::Vector3d::Zero()),
    external_force_command_(Eigen::Vector3d::Zero()),
    external_torque_command_(Eigen::Vector3d::Zero())
  {
    declareParameters();
    loadParameters();

    position_world_ = Eigen::Vector3d(
      initial_position_x_,
      initial_position_y_,
      std::max(0.0, initial_position_z_));
    velocity_world_ = Eigen::Vector3d(
      initial_velocity_x_,
      initial_velocity_y_,
      initial_velocity_z_);
    orientation_body_to_world_ = Eigen::Quaterniond(
      Eigen::AngleAxisd(
        initial_yaw_deg_ * M_PI / 180.0,
        Eigen::Vector3d::UnitZ()));
    orientation_body_to_world_.normalize();

    motor_rpm_command_.fill(0.0);
    motor_rpm_actual_.fill(0.0);

    inertia_ = Eigen::Matrix3d::Zero();
    inertia_(0, 0) = inertia_xx_;
    inertia_(1, 1) = inertia_yy_;
    inertia_(2, 2) = inertia_zz_;
    inertia_inverse_ = inertia_.inverse();

    external_wrench_sub_ =
      create_subscription<geometry_msgs::msg::WrenchStamped>(
      "drone/external_wrench",
      rclcpp::QoS(20),
      std::bind(
        &QuadrotorDynamicsNode::externalWrenchCallback,
        this,
        std::placeholders::_1));

    motor_command_sub_ =
      create_subscription<std_msgs::msg::Float32MultiArray>(
      "drone/motor_rpm_cmd",
      rclcpp::QoS(20),
      std::bind(
        &QuadrotorDynamicsNode::motorCommandCallback,
        this,
        std::placeholders::_1));

    odom_pub_ =
      create_publisher<nav_msgs::msg::Odometry>(
      "drone/odom", rclcpp::QoS(20));

    imu_pub_ =
      create_publisher<sensor_msgs::msg::Imu>(
      "drone/imu", rclcpp::QoS(20));

    path_pub_ =
      create_publisher<nav_msgs::msg::Path>(
      "drone/path", rclcpp::QoS(10));

    actual_rpm_pub_ =
      create_publisher<std_msgs::msg::Float32MultiArray>(
      "drone/motor_rpm", rclcpp::QoS(20));

    applied_wrench_pub_ =
      create_publisher<geometry_msgs::msg::WrenchStamped>(
      "drone/external_wrench_applied", rclcpp::QoS(20));

    tf_broadcaster_ =
      std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    last_command_time_ = now();
    last_external_wrench_time_ = now();
    last_path_time_ = now();

    const auto timer_period =
      std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(time_step_));

    simulation_timer_ =
      create_wall_timer(
      timer_period,
      std::bind(&QuadrotorDynamicsNode::simulationStep, this));

    const double hover_omega =
      std::sqrt(mass_ * gravity_ / (4.0 * thrust_coefficient_));

    const double hover_rpm =
      hover_omega * 60.0 / (2.0 * M_PI);

    RCLCPP_INFO(get_logger(), "Quadrotor dynamics node started");
    RCLCPP_INFO(
      get_logger(),
      "Simulation rate: %.1f Hz, dt: %.6f s",
      simulation_rate_hz_,
      time_step_);
    RCLCPP_INFO(
      get_logger(),
      "Motor order: [front_left, front_right, rear_right, rear_left]");
    RCLCPP_INFO(
      get_logger(),
      "Calculated hover RPM: %.2f",
      hover_rpm);
    RCLCPP_INFO(
      get_logger(),
      "Initial state: position=(%.2f, %.2f, %.2f) m, yaw=%.1f deg, body_frame=%s",
      position_world_.x(),
      position_world_.y(),
      position_world_.z(),
      initial_yaw_deg_,
      body_frame_id_.c_str());
  }

private:
  void declareParameters()
  {
    declare_parameter<double>("simulation_rate_hz", 200.0);
    declare_parameter<double>("mass", 1.36);
    declare_parameter<double>("gravity", 9.81);
    declare_parameter<double>("arm_length", 0.225);
    declare_parameter<double>("inertia_xx", 0.030);
    declare_parameter<double>("inertia_yy", 0.030);
    declare_parameter<double>("inertia_zz", 0.050);
    declare_parameter<double>("thrust_coefficient", 8.54858e-6);
    declare_parameter<double>("torque_coefficient", 1.60e-7);
    declare_parameter<double>("motor_time_constant_up", 0.050);
    declare_parameter<double>("motor_time_constant_down", 0.080);
    declare_parameter<double>("minimum_rpm", 0.0);
    declare_parameter<double>("maximum_rpm", 10000.0);
    declare_parameter<double>("linear_drag_coefficient", 0.10);
    declare_parameter<double>("angular_drag_x", 0.020);
    declare_parameter<double>("angular_drag_y", 0.020);
    declare_parameter<double>("angular_drag_z", 0.030);
    declare_parameter<double>("command_timeout_s", 0.50);
    declare_parameter<double>("path_publish_period_s", 0.05);
    declare_parameter<int>("path_max_points", 3000);
    declare_parameter<double>("external_wrench_timeout_s", 0.25);
    declare_parameter<double>("maximum_external_force_n", 25.0);
    declare_parameter<double>("maximum_external_torque_nm", 2.0);
    declare_parameter<std::string>("world_frame_id", "map");
    declare_parameter<std::string>("body_frame_id", "base_link");
    declare_parameter<double>("initial_position_x", 0.0);
    declare_parameter<double>("initial_position_y", 0.0);
    declare_parameter<double>("initial_position_z", 0.0);
    declare_parameter<double>("initial_velocity_x", 0.0);
    declare_parameter<double>("initial_velocity_y", 0.0);
    declare_parameter<double>("initial_velocity_z", 0.0);
    declare_parameter<double>("initial_yaw_deg", 0.0);
  }

  void loadParameters()
  {
    simulation_rate_hz_ =
      get_parameter("simulation_rate_hz").as_double();

    if (simulation_rate_hz_ <= 0.0) {
      throw std::runtime_error(
              "simulation_rate_hz must be greater than zero");
    }

    time_step_ = 1.0 / simulation_rate_hz_;

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

    motor_time_constant_up_ =
      get_parameter("motor_time_constant_up").as_double();

    motor_time_constant_down_ =
      get_parameter("motor_time_constant_down").as_double();

    minimum_rpm_ = get_parameter("minimum_rpm").as_double();
    maximum_rpm_ = get_parameter("maximum_rpm").as_double();

    linear_drag_coefficient_ =
      get_parameter("linear_drag_coefficient").as_double();

    angular_drag_coefficients_.x() =
      get_parameter("angular_drag_x").as_double();
    angular_drag_coefficients_.y() =
      get_parameter("angular_drag_y").as_double();
    angular_drag_coefficients_.z() =
      get_parameter("angular_drag_z").as_double();

    command_timeout_s_ =
      get_parameter("command_timeout_s").as_double();

    path_publish_period_s_ =
      get_parameter("path_publish_period_s").as_double();

    path_max_points_ =
      static_cast<std::size_t>(
      get_parameter("path_max_points").as_int());

    external_wrench_timeout_s_ =
      get_parameter("external_wrench_timeout_s").as_double();

    maximum_external_force_n_ =
      get_parameter("maximum_external_force_n").as_double();

    maximum_external_torque_nm_ =
      get_parameter("maximum_external_torque_nm").as_double();

    world_frame_id_ =
      get_parameter("world_frame_id").as_string();

    body_frame_id_ =
      get_parameter("body_frame_id").as_string();

    initial_position_x_ =
      get_parameter("initial_position_x").as_double();
    initial_position_y_ =
      get_parameter("initial_position_y").as_double();
    initial_position_z_ =
      get_parameter("initial_position_z").as_double();
    initial_velocity_x_ =
      get_parameter("initial_velocity_x").as_double();
    initial_velocity_y_ =
      get_parameter("initial_velocity_y").as_double();
    initial_velocity_z_ =
      get_parameter("initial_velocity_z").as_double();
    initial_yaw_deg_ =
      get_parameter("initial_yaw_deg").as_double();

    if (mass_ <= 0.0 ||
      thrust_coefficient_ <= 0.0 ||
      inertia_xx_ <= 0.0 ||
      inertia_yy_ <= 0.0 ||
      inertia_zz_ <= 0.0 ||
      external_wrench_timeout_s_ <= 0.0 ||
      maximum_external_force_n_ <= 0.0 ||
      maximum_external_torque_nm_ <= 0.0)
    {
      throw std::runtime_error(
              "Mass, inertia and thrust coefficient must be positive");
    }
  }

  void motorCommandCallback(
    const std_msgs::msg::Float32MultiArray::SharedPtr message)
  {
    if (message->data.size() != 4) {
      RCLCPP_WARN(
        get_logger(),
        "Rejected RPM command: expected 4 values, received %zu",
        message->data.size());
      return;
    }

    for (std::size_t i = 0; i < 4; ++i) {
      if (!std::isfinite(message->data[i])) {
        RCLCPP_WARN(
          get_logger(),
          "Rejected RPM command containing NaN or Inf");
        return;
      }

      motor_rpm_command_[i] =
        std::clamp(
        static_cast<double>(message->data[i]),
        minimum_rpm_,
        maximum_rpm_);
    }

    last_command_time_ = now();
  }


  static Eigen::Vector3d clampVectorNorm(
    const Eigen::Vector3d & value,
    const double maximum_norm)
  {
    const double norm = value.norm();
    if (norm <= maximum_norm || norm < 1.0e-12) {
      return value;
    }
    return value * (maximum_norm / norm);
  }

  void externalWrenchCallback(
    const geometry_msgs::msg::WrenchStamped::SharedPtr message)
  {
    const Eigen::Vector3d force(
      message->wrench.force.x,
      message->wrench.force.y,
      message->wrench.force.z);

    const Eigen::Vector3d torque(
      message->wrench.torque.x,
      message->wrench.torque.y,
      message->wrench.torque.z);

    if (!force.allFinite() || !torque.allFinite()) {
      RCLCPP_WARN(
        get_logger(),
        "Rejected external wrench containing NaN or Inf");
      return;
    }

    const std::string frame =
      message->header.frame_id.empty() ? world_frame_id_ :
      message->header.frame_id;

    if (frame != world_frame_id_ && frame != body_frame_id_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Rejected external wrench frame '%s'; expected '%s' or '%s'",
        frame.c_str(),
        world_frame_id_.c_str(),
        body_frame_id_.c_str());
      return;
    }

    external_force_command_ =
      clampVectorNorm(force, maximum_external_force_n_);
    external_torque_command_ =
      clampVectorNorm(torque, maximum_external_torque_nm_);
    external_wrench_is_body_frame_ = (frame == body_frame_id_);
    last_external_wrench_time_ = now();
  }

  void updateMotorDynamics()
  {
    for (std::size_t i = 0; i < 4; ++i) {
      const double error =
        motor_rpm_command_[i] - motor_rpm_actual_[i];

      const double time_constant =
        error >= 0.0 ?
        motor_time_constant_up_ :
        motor_time_constant_down_;

      const double interpolation_factor =
        std::clamp(
        time_step_ / std::max(time_constant, time_step_),
        0.0,
        1.0);

      motor_rpm_actual_[i] +=
        interpolation_factor * error;

      motor_rpm_actual_[i] =
        std::clamp(
        motor_rpm_actual_[i],
        minimum_rpm_,
        maximum_rpm_);
    }
  }

  void simulationStep()
  {
    const rclcpp::Time current_time = now();

    if ((current_time - last_command_time_).seconds() >
      command_timeout_s_)
    {
      motor_rpm_command_.fill(0.0);
    }

    updateMotorDynamics();

    std::array<double, 4> motor_omega{};
    std::array<double, 4> motor_thrust{};

    for (std::size_t i = 0; i < 4; ++i) {
      motor_omega[i] =
        motor_rpm_actual_[i] * 2.0 * M_PI / 60.0;

      motor_thrust[i] =
        thrust_coefficient_ *
        motor_omega[i] *
        motor_omega[i];
    }

    const double total_thrust =
      motor_thrust[0] +
      motor_thrust[1] +
      motor_thrust[2] +
      motor_thrust[3];

    const double x_configuration_arm =
      arm_length_ / std::sqrt(2.0);

    const double roll_torque =
      x_configuration_arm *
      (
      motor_thrust[0] -
      motor_thrust[1] -
      motor_thrust[2] +
      motor_thrust[3]);

    const double pitch_torque =
      x_configuration_arm *
      (
      -motor_thrust[0] -
      motor_thrust[1] +
      motor_thrust[2] +
      motor_thrust[3]);

    const double yaw_torque =
      torque_coefficient_ *
      (
      motor_omega[0] * motor_omega[0] -
      motor_omega[1] * motor_omega[1] +
      motor_omega[2] * motor_omega[2] -
      motor_omega[3] * motor_omega[3]);

    const Eigen::Vector3d torque_body(
      roll_torque,
      pitch_torque,
      yaw_torque);

    const Eigen::Matrix3d rotation_body_to_world =
      orientation_body_to_world_.toRotationMatrix();

    Eigen::Vector3d external_force_world =
      Eigen::Vector3d::Zero();
    Eigen::Vector3d external_torque_body =
      Eigen::Vector3d::Zero();

    if ((current_time - last_external_wrench_time_).seconds() <=
      external_wrench_timeout_s_)
    {
      if (external_wrench_is_body_frame_) {
        external_force_world =
          rotation_body_to_world * external_force_command_;
        external_torque_body = external_torque_command_;
      } else {
        external_force_world = external_force_command_;
        external_torque_body =
          rotation_body_to_world.transpose() *
          external_torque_command_;
      }
    }

    const Eigen::Vector3d thrust_body(
      0.0,
      0.0,
      total_thrust);

    const Eigen::Vector3d gravity_world(
      0.0,
      0.0,
      -gravity_);

    const Eigen::Vector3d drag_force_world =
      -linear_drag_coefficient_ * velocity_world_;

    acceleration_world_ =
      rotation_body_to_world * thrust_body / mass_ +
      gravity_world +
      drag_force_world / mass_ +
      external_force_world / mass_;

    const Eigen::Vector3d gyroscopic_term =
      angular_velocity_body_.cross(
      inertia_ * angular_velocity_body_);

    const Eigen::Vector3d angular_drag_torque =
      -angular_drag_coefficients_.cwiseProduct(
      angular_velocity_body_);

    const Eigen::Vector3d angular_acceleration_body =
      inertia_inverse_ *
      (
      torque_body +
      external_torque_body +
      angular_drag_torque -
      gyroscopic_term);

    velocity_world_ +=
      acceleration_world_ * time_step_;

    position_world_ +=
      velocity_world_ * time_step_;

    angular_velocity_body_ +=
      angular_acceleration_body * time_step_;

    const Eigen::Quaterniond angular_velocity_quaternion(
      0.0,
      angular_velocity_body_.x(),
      angular_velocity_body_.y(),
      angular_velocity_body_.z());

    const Eigen::Quaterniond quaternion_derivative =
      orientation_body_to_world_ *
      angular_velocity_quaternion;

    orientation_body_to_world_.coeffs() +=
      0.5 *
      time_step_ *
      quaternion_derivative.coeffs();

    orientation_body_to_world_.normalize();

    if (position_world_.z() < 0.0) {
      position_world_.z() = 0.0;

      if (velocity_world_.z() < 0.0) {
        velocity_world_.z() = 0.0;
      }
    }

    publishAppliedWrench(
      current_time,
      external_force_world,
      rotation_body_to_world * external_torque_body);

    publishState(current_time);
  }


  void publishAppliedWrench(
    const rclcpp::Time & stamp,
    const Eigen::Vector3d & force_world,
    const Eigen::Vector3d & torque_world)
  {
    geometry_msgs::msg::WrenchStamped message;
    message.header.stamp = stamp;
    message.header.frame_id = world_frame_id_;
    message.wrench.force.x = force_world.x();
    message.wrench.force.y = force_world.y();
    message.wrench.force.z = force_world.z();
    message.wrench.torque.x = torque_world.x();
    message.wrench.torque.y = torque_world.y();
    message.wrench.torque.z = torque_world.z();
    applied_wrench_pub_->publish(message);
  }

  void publishState(const rclcpp::Time & stamp)
  {
    const Eigen::Matrix3d rotation_body_to_world =
      orientation_body_to_world_.toRotationMatrix();

    const Eigen::Vector3d velocity_body =
      rotation_body_to_world.transpose() *
      velocity_world_;

    nav_msgs::msg::Odometry odometry;
    odometry.header.stamp = stamp;
    odometry.header.frame_id = world_frame_id_;
    odometry.child_frame_id = body_frame_id_;

    odometry.pose.pose.position.x = position_world_.x();
    odometry.pose.pose.position.y = position_world_.y();
    odometry.pose.pose.position.z = position_world_.z();

    odometry.pose.pose.orientation.w =
      orientation_body_to_world_.w();
    odometry.pose.pose.orientation.x =
      orientation_body_to_world_.x();
    odometry.pose.pose.orientation.y =
      orientation_body_to_world_.y();
    odometry.pose.pose.orientation.z =
      orientation_body_to_world_.z();

    odometry.twist.twist.linear.x = velocity_body.x();
    odometry.twist.twist.linear.y = velocity_body.y();
    odometry.twist.twist.linear.z = velocity_body.z();

    odometry.twist.twist.angular.x =
      angular_velocity_body_.x();
    odometry.twist.twist.angular.y =
      angular_velocity_body_.y();
    odometry.twist.twist.angular.z =
      angular_velocity_body_.z();

    odom_pub_->publish(odometry);

    sensor_msgs::msg::Imu imu;
    imu.header.stamp = stamp;
    imu.header.frame_id = body_frame_id_;

    imu.orientation = odometry.pose.pose.orientation;
    imu.angular_velocity.x = angular_velocity_body_.x();
    imu.angular_velocity.y = angular_velocity_body_.y();
    imu.angular_velocity.z = angular_velocity_body_.z();

    const Eigen::Vector3d gravity_world(
      0.0,
      0.0,
      -gravity_);

    const Eigen::Vector3d specific_force_body =
      rotation_body_to_world.transpose() *
      (acceleration_world_ - gravity_world);

    imu.linear_acceleration.x = specific_force_body.x();
    imu.linear_acceleration.y = specific_force_body.y();
    imu.linear_acceleration.z = specific_force_body.z();

    imu_pub_->publish(imu);

    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = stamp;
    transform.header.frame_id = world_frame_id_;
    transform.child_frame_id = body_frame_id_;

    transform.transform.translation.x = position_world_.x();
    transform.transform.translation.y = position_world_.y();
    transform.transform.translation.z = position_world_.z();

    transform.transform.rotation =
      odometry.pose.pose.orientation;

    tf_broadcaster_->sendTransform(transform);

    std_msgs::msg::Float32MultiArray actual_rpm_message;
    actual_rpm_message.data.resize(4);

    for (std::size_t i = 0; i < 4; ++i) {
      actual_rpm_message.data[i] =
        static_cast<float>(motor_rpm_actual_[i]);
    }

    actual_rpm_pub_->publish(actual_rpm_message);

    if ((stamp - last_path_time_).seconds() >=
      path_publish_period_s_)
    {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = odometry.header;
      pose.pose = odometry.pose.pose;

      path_message_.header = odometry.header;
      path_message_.poses.push_back(pose);

      if (path_message_.poses.size() > path_max_points_) {
        path_message_.poses.erase(
          path_message_.poses.begin());
      }

      path_pub_->publish(path_message_);
      last_path_time_ = stamp;
    }
  }

  double simulation_rate_hz_{200.0};
  double time_step_{0.005};

  double mass_{1.36};
  double gravity_{9.81};
  double arm_length_{0.225};

  double inertia_xx_{0.030};
  double inertia_yy_{0.030};
  double inertia_zz_{0.050};

  double thrust_coefficient_{8.54858e-6};
  double torque_coefficient_{1.60e-7};

  double motor_time_constant_up_{0.050};
  double motor_time_constant_down_{0.080};

  double minimum_rpm_{0.0};
  double maximum_rpm_{10000.0};

  double linear_drag_coefficient_{0.10};

  Eigen::Vector3d angular_drag_coefficients_{
    0.020,
    0.020,
    0.030};

  double command_timeout_s_{0.50};
  double path_publish_period_s_{0.05};
  std::size_t path_max_points_{3000};

  double external_wrench_timeout_s_{0.25};
  double maximum_external_force_n_{25.0};
  double maximum_external_torque_nm_{2.0};
  std::string world_frame_id_{"map"};
  std::string body_frame_id_{"base_link"};
  double initial_position_x_{0.0};
  double initial_position_y_{0.0};
  double initial_position_z_{0.0};
  double initial_velocity_x_{0.0};
  double initial_velocity_y_{0.0};
  double initial_velocity_z_{0.0};
  double initial_yaw_deg_{0.0};

  Eigen::Matrix3d inertia_;
  Eigen::Matrix3d inertia_inverse_;

  Eigen::Vector3d position_world_;
  Eigen::Vector3d velocity_world_;
  Eigen::Quaterniond orientation_body_to_world_;
  Eigen::Vector3d angular_velocity_body_;
  Eigen::Vector3d acceleration_world_;
  Eigen::Vector3d external_force_command_;
  Eigen::Vector3d external_torque_command_;
  bool external_wrench_is_body_frame_{false};

  std::array<double, 4> motor_rpm_command_;
  std::array<double, 4> motor_rpm_actual_;

  rclcpp::Time last_command_time_;
  rclcpp::Time last_path_time_;
  rclcpp::Time last_external_wrench_time_;

  nav_msgs::msg::Path path_message_;

  rclcpp::Subscription<
    std_msgs::msg::Float32MultiArray>::SharedPtr
  motor_command_sub_;

  rclcpp::Subscription<
    geometry_msgs::msg::WrenchStamped>::SharedPtr
  external_wrench_sub_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr
  odom_pub_;

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr
  imu_pub_;

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr
  path_pub_;

  rclcpp::Publisher<
    std_msgs::msg::Float32MultiArray>::SharedPtr
  actual_rpm_pub_;

  rclcpp::Publisher<
    geometry_msgs::msg::WrenchStamped>::SharedPtr
  applied_wrench_pub_;

  std::unique_ptr<tf2_ros::TransformBroadcaster>
  tf_broadcaster_;

  rclcpp::TimerBase::SharedPtr simulation_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    rclcpp::spin(
      std::make_shared<QuadrotorDynamicsNode>());
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(
      rclcpp::get_logger("quadrotor_dynamics"),
      "Fatal error: %s",
      exception.what());

    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
