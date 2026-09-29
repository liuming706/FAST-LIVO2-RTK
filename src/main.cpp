#include "optimization.h"
#include <memory>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "laserMapping");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  LIVMapper mapper(nh); 

  bool gps_enabled;
  nh.param<bool>("gps/gps_en", gps_enabled, true);
  std::unique_ptr<optimization> opti;
  if (gps_enabled) opti = std::make_unique<optimization>(nh);


  mapper.initializeSubscribersAndPublishers(nh, it);
  mapper.run();
  return 0;
}