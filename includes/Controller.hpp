#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#include <ackermann_msgs/AckermannDriveStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>

#include <MJMPC/MJMPC>
#include <memory>
#include <thread>

class MPCController {
 public:
  MPCController(ros::NodeHandle& nh);
  ~MPCController();
  void setState(const Eigen::VectorXd& state);
  void setReference(const Eigen::VectorXd& reference);
  void controlThread();

 private:
  std::shared_ptr<MJMPC::Dynamics::DiscreteDynamics> dynamics_;
  std::unique_ptr<MJMPC::Control::Controller> controller_;

  Eigen::Vector3d state_;
  Eigen::Vector3d reference_;

  double steering_angle_limit_;
  double speed_limit_;

  ros::NodeHandle nh_;
  ros::Subscriber sub_state_;
  ros::Subscriber sub_reference_;
  ros::Publisher pub_control_;
  ros::Publisher pub_path_;

  std::thread control_thread_;
  bool running_;

  void stateCallback(const geometry_msgs::PoseStamped& msg);
  void referenceCallback(const geometry_msgs::PoseStamped& msg);
};

#endif
