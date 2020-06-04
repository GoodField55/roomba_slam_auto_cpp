#include "roomba_slam_auto_cpp/roomba.h"
/*
ros::Publisher Roomba::cmd_vel_pub = nullptr;
ros::Subscriber Roomba::bumper_sub = nullptr;
ros::Subscriber Roomba::cliff_sub = nullptr;
ca_msgs::Bumper Roomba::bumper_values = nullptr;
ca_msgs::Cliff Roomba::cliff_values = nullptr;
geometry_msgs::Twist Roomba::data = nullptr;
*/

Roomba::Roomba(void){
  cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
//  bumper_sub = n.subscribe("bumper", 1, Roomba::bumperCallback);
//  cliff_sub = n.subscribe("cliff", 1, Roomba::cliffCallback);

  data.linear.x = 0;
  data.angular.z = 0;
}

void Roomba::bumperCallback(const ca_msgs::Bumper::ConstPtr& msg){
  bumper_values = *msg;
  ROS_INFO("light_signal_right: [%d]", msg->light_signal_right);
}

void Roomba::cliffCallback(const ca_msgs::Cliff::ConstPtr& msg){
  cliff_values = *msg;
  ROS_INFO("cliff_signal_left: [%d]", msg->cliff_signal_left);
}

void Roomba::run(void){
  ros::Rate loop_rate(10);
  data.linear.x = 0.1;

  int count = 0;
  while (ros::ok()){
      /**
       * This is a message object. You stuff it with data, and then publish it.
       */
    std_msgs::String msg;

    std::stringstream ss;
    ss << "hello world " << count;
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());

      /**
       * The publish() function is how you send messages. The parameter
       * is the message object. The type of this object must agree with the type
       * given as a template parameter to the advertise<>() call, as was done
       * in the constructor above.
       */
      // chatter_pub.publish(msg);

    cmd_vel_pub.publish(data);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }
}

int main(int argc, char **argv){

  ros::init(argc, argv, "roomba");

  Roomba roomba;
  roomba.run();

  return 0;
}
