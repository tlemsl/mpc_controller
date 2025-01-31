#include "Controller.hpp"

#include <tf2/utils.h>

#include <chrono>

#include "Ackermann.hpp"

MPCController::MPCController(ros::NodeHandle& nh) : nh_(nh) {
  double dt = 0.05;  // 20Hz
  double WB = 0.26;  // Wheelbase
  Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(3, 3);
  Q(0, 0) = 10;
  Q(1, 1) = 10;
  Q(2, 2) = 5;
  Eigen::MatrixXd R = Eigen::MatrixXd::Identity(2, 2);
  R(0, 0) = 5;
  R(1, 1) = 5;
  Eigen::MatrixXd Qf = Q;
  int N = 20;
  steering_angle_limit_ = 0.34;
  speed_limit_ = 4.0;
  Eigen::VectorXd u_reference = Eigen::VectorXd::Zero(2);
  // u_reference(0) = 1.0;
  Eigen::VectorXd u_max = Eigen::VectorXd::Zero(2);
  Eigen::VectorXd u_min = Eigen::VectorXd::Zero(2);
  u_max(0) = speed_limit_;
  u_max(1) = steering_angle_limit_;
  u_min(0) = -speed_limit_;
  u_min(1) = -steering_angle_limit_;

  std::shared_ptr<MJMPC::Dynamics::ContinuousDynamics> dynamics =
      std::make_shared<AckermannContinuousDynamics>(speed_limit_,
                                                    steering_angle_limit_, WB);
  auto dynamics_discrete = std::make_shared<MJMPC::Dynamics::DiscreteDynamics>(
      dynamics, MJMPC::Dynamics::DiscretizationMethod::ForwardEuler, dt);
  // auto dynamics_discrete = std::make_shared<AckermannDiscreteDynamics>(
  //     speed_limit_, steering_angle_limit_, WB, dt);
  // controller_ = std::make_unique<MJMPC::Control::SQPMPC>(
  //     dynamics_discrete, Q, R, Qf, N, u_max, u_min, u_reference);
  controller_ =
      std::make_unique<MJMPC::Control::iLQR>(dynamics_discrete, Q, R, Qf, N);

  // controller_ =
  //     std::make_unique<MJMPC::Control::iLQR>(dynamics_discrete, Q, R, Qf, N);
  controller_->setState(Eigen::VectorXd::Zero(3));
  controller_->setReference(Eigen::VectorXd::Zero(3));
  sub_state_ = nh_.subscribe("/mushr_mujoco_ros/buddy/pose", 1,
                             &MPCController::stateCallback, this);
  sub_reference_ = nh_.subscribe("/move_base_simple/goal", 1,
                                 &MPCController::referenceCallback, this);
  pub_control_ = nh_.advertise<ackermann_msgs::AckermannDriveStamped>(
      "/mushr_mujoco_ros/buddy/control", 1);
  pub_path_ = nh.advertise<nav_msgs::Path>("/predicted_trajectory", 1);

  running_ = true;
  control_thread_ = std::thread(&MPCController::controlThread, this);
}

MPCController::~MPCController() {
  running_ = false;
  control_thread_.join();
}

void MPCController::stateCallback(const geometry_msgs::PoseStamped& msg) {
  double x = msg.pose.position.x;
  double y = msg.pose.position.y;
  tf2::Quaternion q;
  tf2::fromMsg(msg.pose.orientation, q);
  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  state_ << x, y, yaw;
  controller_->setState(state_);
}

void MPCController::referenceCallback(const geometry_msgs::PoseStamped& msg) {
  double x = msg.pose.position.x;
  double y = msg.pose.position.y;
  tf2::Quaternion q;
  tf2::fromMsg(msg.pose.orientation, q);
  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  reference_ << x, y, yaw;
  controller_->setReference(reference_);
}

void MPCController::controlThread() {
  ros::Rate rate(20);
  nav_msgs::Path path;
  ackermann_msgs::AckermannDriveStamped msg;

  while (running_) {
    auto start = std::chrono::high_resolution_clock::now();
    Eigen::VectorXd control = controller_->getControl();
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "map";
    msg.drive.speed = std::clamp(control[0], -speed_limit_, speed_limit_);
    msg.drive.steering_angle =
        std::clamp(control[1], -steering_angle_limit_, steering_angle_limit_);
    pub_control_.publish(msg);
    // std::cout << "Control: " << control.transpose()(1) << std::endl;
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration = end - start;
    std::cout << "Control time: "
              << std::chrono::duration_cast<std::chrono::milliseconds>(duration)
                     .count()
              << "ms" << std::endl;

    path.header.stamp = ros::Time::now();
    path.header.frame_id = "map";
    path.poses.clear();
    for (const auto& state : controller_->getTrajectory()) {
      geometry_msgs::PoseStamped pose;
      pose.pose.position.x = state(0);
      pose.pose.position.y = state(1);
      pose.pose.orientation = tf2::toMsg(tf2::Quaternion(0, 0, state(2)));
      path.poses.push_back(pose);
    }
    pub_path_.publish(path);
    rate.sleep();
  }
}
