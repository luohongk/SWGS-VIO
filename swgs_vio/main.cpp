#include <iostream>
#include <ros/ros.h>
#include "swgs_vio.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "swgs_vio_node");

  ros::NodeHandle swgs_vio_nh;

  image_transport::ImageTransport it(swgs_vio_nh);

  SlidingWindowGSVIO sliding_window_gsvio(swgs_vio_nh);

  mapper.initializeSubscribersAndPublishers(nh, it);
  mapper.run();
  return 0;
}