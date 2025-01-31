#include "Ackermann.hpp"

AckermannContinuousDynamics::AckermannContinuousDynamics(
    double speed_limit, double steering_angle_limit, double wheelbase,
    int state_dim, int control_dim)
    : ContinuousDynamics(state_dim, control_dim),
      speed_limit_(speed_limit),
      steering_angle_limit_(steering_angle_limit),
      wheelbase_(wheelbase) {}

Eigen::VectorXd AckermannContinuousDynamics::getValue(
    const Eigen::VectorXd& state, const Eigen::VectorXd& control) const {
  Eigen::VectorXd xdot = Eigen::VectorXd::Zero(state_dim_);

  double v = control(0);
  v = std::clamp(v, -speed_limit_, speed_limit_);
  double steering_angle = control(1);
  steering_angle =
      std::clamp(steering_angle, -steering_angle_limit_, steering_angle_limit_);
  double theta = state(2);
  xdot(0) = v * cos(theta);
  xdot(1) = v * sin(theta);
  xdot(2) = v / wheelbase_ * tan(steering_angle);
  return xdot;
}

AckermannDiscreteDynamics::AckermannDiscreteDynamics(
    double speed_limit, double steering_angle_limit, double wheelbase,
    double dt)
    : speed_limit_(speed_limit),
      steering_angle_limit_(steering_angle_limit),
      wheelbase_(wheelbase),
      DiscreteDynamics(3, 2, dt) {}

Eigen::VectorXd AckermannDiscreteDynamics::getValue(
    const Eigen::VectorXd& state, const Eigen::VectorXd& control) const {
  Eigen::VectorXd x_next = Eigen::VectorXd::Zero(state_dim_);
  Eigen::VectorXd croped_control = control;
  // croped_control(0) = std::clamp(control(0), -speed_limit_, speed_limit_);
  // croped_control(1) =
  //     std::clamp(control(1), -steering_angle_limit_, steering_angle_limit_);
  x_next(0) = state(0) + dt_ * croped_control(0) * cos(state(2));
  x_next(1) = state(1) + dt_ * croped_control(0) * sin(state(2));
  x_next(2) =
      state(2) + dt_ * croped_control(0) / wheelbase_ * tan(croped_control(1));
  return x_next;
}

Eigen::MatrixXd AckermannDiscreteDynamics::getStatePartialDerivative(
    const Eigen::VectorXd& state, const Eigen::VectorXd& control) const {
  Eigen::VectorXd croped_control = control;
  // croped_control(0) = std::clamp(control(0), -speed_limit_, speed_limit_);
  // croped_control(1) =
  //     std::clamp(control(1), -steering_angle_limit_, steering_angle_limit_);
  Eigen::MatrixXd A = Eigen::MatrixXd::Identity(3, 3);
  A(0, 2) = -croped_control(0) * sin(state(2)) * dt_;
  A(1, 2) = croped_control(0) * cos(state(2)) * dt_;
  return A;
}

Eigen::MatrixXd AckermannDiscreteDynamics::getControlPartialDerivative(
    const Eigen::VectorXd& state, const Eigen::VectorXd& control) const {
  Eigen::MatrixXd B = Eigen::MatrixXd::Zero(3, 2);
  B(0, 0) = cos(state(2)) * dt_;
  B(1, 0) = sin(state(2)) * dt_;
  B(2, 1) = control(0) / wheelbase_ * dt_;
  return B;
}
