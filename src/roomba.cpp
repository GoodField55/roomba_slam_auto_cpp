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

bool Roomba::wall_detect ( void ){
  return ( bumper_values.light_signal_center_left > bumper_signal_center_threshold ||
           bumper_values.light_signal_center_right > bumper_signal_center_threshold ||
           bumper_values.light_signal_front_left > bumper_signal_front_threshold ||
           bumper_values.light_signal_front_right > bumper_signal_front_threshold ||
           bumper_values.light_signal_left > bumper_signal_threshold ||
           bumper_values.light_signal_right > bumper_signal_threshold );
}

bool Roomba::back_turn( int sensor, int position ){

  int x, z;
  int goal_back, goal_turn;
  bool sensor_value;

  switch ( sensor ){
    case ROOMBA_CLIFF : switch ( position ){
			  case ROOMBA_LEFT_FRONT 	: sensor_value = cliff_values.is_cliff_front_left;
								x = -1 * x_lo;
								z = -1 * z_hi;
								goal_back = goal_cliff_front_back;
                                                                goal_turn = goal_cliff_front_turn;
								 break;
                          case ROOMBA_RIGHT_FRONT 	: sensor_value = cliff_values.is_cliff_front_right;
                                                                x = -1 * x_lo;
                                                                z = z_hi;
                                                                goal_back = goal_cliff_front_back;
                                                                goal_turn = goal_cliff_front_turn;
                                                                 break;
                          case ROOMBA_LEFT 		: sensor_value = cliff_values.is_cliff_left;
                                                                x = -1 * x_lo;
                                                                z = -1 * z_hi;
                                                                goal_back = goal_cliff_back;
                                                                goal_turn = goal_cliff_turn;
                                                                 break;
                          case ROOMBA_RIGHT 		: sensor_value = cliff_values.is_cliff_right;
                                                                x = -1 * x_lo;
                                                                z = z_hi;
                                                                goal_back = goal_cliff_back;
                                                                goal_turn = goal_cliff_turn;
                                                                 break;
			  default			: sensor_value = false;
                                                                x = 0;
                                                                z = 0;
                                                                goal_back = 1;
                                                                goal_turn = 1;
                                                                 break;
                        }
    case ROOMBA_BUMPER : switch ( position ){
                          case ROOMBA_LEFT              : sensor_value = bumper_values.is_left_pressed;
                                                                x = -1 * x_lo;
                                                                z = -1 * z_hi;
                                                                goal_back = goal_bumper_back;
                                                                goal_turn = goal_bumper_turn;
                                                                 break;
                          case ROOMBA_RIGHT             : sensor_value = bumper_values.is_right_pressed;
                                                                x = -1 * x_lo;
                                                                z = z_hi;
                                                                goal_back = goal_bumper_back;
                                                                goal_turn = goal_bumper_turn;
                                                                 break;
                          default                       : sensor_value = false;
                                                                x = 0;
                                                                z = 0;
                                                                goal_back = 1;
                                                                goal_turn = 1;
                                                                 break;
                        }
    default : sensor_value = false;
		x = 0;
		z = 0;
                goal_back = 1;
                goal_turn = 1;
  }
 
  if ( sensor_value ||
       ( (roomba_status & sensor) && (roomba_status & position) ) ){           // under sensor process
    if ( (roomba_status & ROOMBA_PHASE) == 0 ){                                // under back process
      if ( !((roomba_status & sensor) && (roomba_status & position)) ){        // first time of sensor back
        counter = 0;
        roomba_status |= ( sensor | position );
        under_accel = false;
        roomba_status |= ROOMBA_BACK;         // set back flag
        roomba_status &= ~ROOMBA_TURN;        // clear turn flag
      }
      data.linear.x = x;
      data.angular.z = 0;
      counter++;
      if ( counter > goal_back ){ 			// stop backward
        data.linear.x = 0;
        data.angular.z = 0;
        roomba_status++;
      }
    }else{                                                                      // under turn process
      if ( !(roomba_status & ROOMBA_TURN) ){                                    // first time of cliff left front turn
        counter = 0;
        under_accel = false;
        roomba_status |= ROOMBA_TURN;         // set turn flag
        roomba_status &= ~ROOMBA_BACK;        // clear back flag
      }
      data.linear.x = 0;
      data.angular.z = z;
      counter++;
      if ( counter > goal_turn ){ 		// stop turn
        data.linear.x = 0;
        data.angular.z = 0;
        // roomba_status &= ~ROOMBA_PHASE;        // clear phase
        roomba_status = 0;			// clear roomba_status
      }
    }
    return true;
  }else{
    roomba_status = 0;				// clear roomba_status
    return false;
  }
}

void Roomba::run(void){
  boost = 10;
  loop_hz = 10 * boost;	//boost x10
  ros::Rate loop_rate(loop_hz);

  x_hi = 0.2;
  x_lo = 0.05;
  x_start = 0.005;
  accel_step = 0.001;

  data.linear.x = x_start;
  under_accel = false;
  
  z_hi = M_PI / 4.0;
  z_lo = M_PI / 8.0;
  data.angular.z = z_hi;
  
  goal_bumper_back = loop_hz * 0.02 / x_lo;		// counts to 0.02m
  goal_bumper_turn = loop_hz * M_PI / 4.5 / z_hi;	// counts to 80degree

  goal_cliff_back = loop_hz * 0.05 / x_lo;             // counts to 0.05m
  goal_cliff_turn = loop_hz * M_PI / 4.5 / z_hi;       // counts to 80degree

  goal_cliff_front_back = loop_hz * 0.05 / x_lo;             // counts to 0.05m
  goal_cliff_front_turn = loop_hz * M_PI / 3.3 / z_hi;       // counts to 110degree

  bumper_signal_threshold = 100;	// bumper signal comparate level
  bumper_signal_front_threshold = 150;	// bumper signal front comparate level
  bumper_signal_center_threshold = 200;	// bumper signal center comparate level

  roomba_status = 0;
  counter = 0;

  while (ros::ok()){

    if ( back_turn ( ROOMBA_CLIFF, ROOMBA_LEFT_FRONT )){

    }else if ( back_turn ( ROOMBA_CLIFF, ROOMBA_RIGHT_FRONT )){

    }else if ( back_turn ( ROOMBA_CLIFF, ROOMBA_LEFT )){

    }else if ( back_turn ( ROOMBA_CLIFF, ROOMBA_RIGHT )){

    }else if ( back_turn ( ROOMBA_BUMPER, ROOMBA_LEFT )){

    }else if ( back_turn ( ROOMBA_BUMPER, ROOMBA_RIGHT )){

    }else{
      if ( wall_detect() ){
        data.linear.x = x_lo;
        data.angular.z = 0;
        under_accel = false;
      }else{
        if ( ! under_accel ){
          data.linear.x = x_start;
          under_accel = true;
        }else{
          data.linear.x += accel_step;
          if ( data.linear.x > x_hi ){
            data.linear.x = x_hi;
          }
          data.angular.z = 0;
        }
      }
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
