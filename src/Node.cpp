#include <ros/ros.h>

#include "Controller.hpp"

int main(int argc, char **argv) {
  ros::init(argc, argv, "node");
  ros::NodeHandle nh;
  MPCController controller(nh);
  ros::spin();
  return 0;
}
