#ifndef DYNAMICS_HPP
#define DYNAMICS_HPP

#include <ros/ros.h>

#include <MJMPC/MJMPC>

class AckermannContinuousDynamics : public MJMPC::Dynamics::ContinuousDynamics {
 public:
  AckermannContinuousDynamics(double speed_limit, double steering_angle_limit,
                              double wheelbase, int state_dim = 3,
                              int control_dim = 2);
  ~AckermannContinuousDynamics() {};
  Eigen::VectorXd getValue(const Eigen::VectorXd& state,
                           const Eigen::VectorXd& control) const override;

 private:
  double speed_limit_;
  double steering_angle_limit_;
  double wheelbase_;
};

class AckermannDiscreteDynamics : public MJMPC::Dynamics::DiscreteDynamics {
 public:
  AckermannDiscreteDynamics(double speed_limit, double steering_angle_limit,
                            double wheelbase, double dt);
  ~AckermannDiscreteDynamics() {};
  Eigen::VectorXd getValue(const Eigen::VectorXd& state,
                           const Eigen::VectorXd& control) const override;
  Eigen::MatrixXd getStatePartialDerivative(
      const Eigen::VectorXd& state,
      const Eigen::VectorXd& control) const override;
  Eigen::MatrixXd getControlPartialDerivative(
      const Eigen::VectorXd& state,
      const Eigen::VectorXd& control) const override;

 private:
  double speed_limit_;
  double steering_angle_limit_;
  double wheelbase_;
};

#endif
