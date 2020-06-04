#include "ros/ros.h"
#include "std_msgs/String.h"
#include "create_driver/create_driver.h"

#include <sstream>

class Roomba{
public:
  ros::NodeHandle n;

  ros::Publisher cmd_vel_pub;
  ros::Subscriber bumper_sub;
  ros::Subscriber cliff_sub;

  ca_msgs::Bumper bumper_values;
  ca_msgs::Cliff cliff_values;

  geometry_msgs::Twist data;

public:
  void bumperCallback(const ca_msgs::Bumper::ConstPtr& msg);
  void cliffCallback(const ca_msgs::Cliff::ConstPtr& msg);
  void run(void);
  Roomba(void);

};

