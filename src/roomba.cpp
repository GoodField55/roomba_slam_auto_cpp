#include "roomba_slam_auto_cpp/roomba.h"

Roomba::Roomba(void){
  cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  bumper_sub = n.subscribe("bumper", 1, &Roomba::bumperCallback, this);
  cliff_sub = n.subscribe("cliff", 1, &Roomba::cliffCallback, this);

  data.linear.x = 0;
  data.angular.z = 0;
}

void Roomba::bumperCallback(const ca_msgs::Bumper::ConstPtr& msg){
  bumper_values = *msg;
}

void Roomba::cliffCallback(const ca_msgs::Cliff::ConstPtr& msg){
  cliff_values = *msg;
}

void Roomba::run(void){
  ros::Rate loop_rate(10);
  data.linear.x = 0.1;

  while (ros::ok()){

    if ( bumper_values.is_left_pressed == true || bumper_values.is_right_pressed == true){
      data.linear.x = 0.0;
    }else{
      data.linear.x = 0.1;
    }
    cmd_vel_pub.publish(data);

    ros::spinOnce();
    loop_rate.sleep();
  }
}

int main(int argc, char **argv){

  ros::init(argc, argv, "roomba");

  Roomba roomba;
  roomba.run();

  return 0;
}
