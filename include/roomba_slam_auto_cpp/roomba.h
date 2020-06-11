#ifndef __ROOMBA__
#define __ROOMBA__

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "create_driver/create_driver.h"

#include <sstream>
#include <math.h>

// roomba_status
// 7654321076543210
//             **** -> phase
//            *     -> cliff
//           *      -> bumper
//          *       -> back
//         *        -> turn
//        *         -> left
//       *          -> right
//      *           -> front_left
//     *            -> front_right
//    *             -> center_left
//   *              -> center_right

#define ROOMBA_PHASE		0x000f
#define ROOMBA_CLIFF		0x0010
#define ROOMBA_BUMPER		0x0020
#define ROOMBA_BACK             0x0040
#define ROOMBA_TURN             0x0080
#define ROOMBA_LEFT		0x0100
#define ROOMBA_RIGHT		0x0200
#define ROOMBA_LEFT_FRONT	0x0400
#define ROOMBA_RIGHT_FRONT	0x0800
#define ROOMBA_LEFT_CENTER	0x1000
#define ROOMBA_RIGHT_CENTER	0x2000


class Roomba{
private:
  ros::NodeHandle n_;
  ros::NodeHandle pn_;

  ros::Publisher cmd_vel_pub;

  ros::Subscriber bumper_sub;
  ros::Subscriber cliff_sub;

  ca_msgs::Bumper bumper_values;
  ca_msgs::Cliff cliff_values;

  geometry_msgs::Twist data;

  unsigned int roomba_status;	// roomba proces status
  int counter;			// for process time count

  float x_lo;			// linear low speed(m/sec)
  float x_hi;			// linear high speed(m/sec)
  float x_start;                  // linear initial speed(m/sec)

  float accel_step;		// linear speed accel step(m/sec)
  bool under_accel;		// true : under accel

  float z_lo;			// angular low speed(rad/sec)
  float z_hi;			// angular high speed(rad/sec)
  float z_lo_degree;		// angular low speed(degree/sec)
  float z_hi_degree;            // angular high speed(degree/sec)

  float bumper_back_length;     	// bumper back length (m)
  float cliff_back_length;     		// cliff back length (m)
  float cliff_front_back_length;     	// cliff front back length (m)
  float bumper_turn_angle;             	// bumper turn angle (degree)
  float cliff_turn_angle;               // cliff turn angle (degree)
  float cliff_front_turn_angle;         // cliff front turn angle (degree)

  int goal_bumper_back;		// counts to goal(back)
  int goal_bumper_turn;		// counts to goal(turn)
  int goal_cliff_back;		// counts to goal(cliff back)
  int goal_cliff_turn;		// counts to goal(cliff turn)
  int goal_cliff_front_back;    // counts to goal(cliff_front back)
  int goal_cliff_front_turn;    // counts to goal(cliff_front turn)

  float bumper_signal_threshold;		// bumper signal comparate level
  float bumper_signal_front_threshold; 	// bumper signal front comparate level
  float bumper_signal_center_threshold; 	// bumper signal center comparate level

  float loop_hz_origin;
  float loop_hz;			// loop cycle(hz) = loop_hz_origin * boost
  float boost;			// boost loop cycle

public:
  void bumperCallback(const ca_msgs::Bumper::ConstPtr& msg);
  void cliffCallback(const ca_msgs::Cliff::ConstPtr& msg);
  bool wall_detect( void );
  bool back_turn( int sensor, int position );
  void run(void);
  explicit Roomba(ros::NodeHandle& n);

};

#endif  // __ROOMBA__
