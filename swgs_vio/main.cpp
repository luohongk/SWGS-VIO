#include <iostream>
#include <ros/ros.h>
#include "swgs_vio.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "swgs_vio_node");

  ros::NodeHandle swgs_vio_nh;
      
  
  // 设置ROS日志级别为Info
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

  image_transport::ImageTransport it(swgs_vio_nh);
   
  // Path configuration
  std::string project_source_dir = PROJECT_SOURCE_DIR;

  loadParameters(swgs_vio_nh);
      
  subscribeToData(swgs_vio_nh, it, sliding_window_gsvio);
  
  preprocessData(sliding_window_gsvio);

  SlidingWindowGSVIO sliding_window_gsvio(swgs_vio_nh);

  sliding_window_gsvio.run();

  return 0;
}