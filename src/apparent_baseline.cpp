#include "ros/ros.h"
#include "chicago/MotorSpeed.h"
#include "nav_msgs/Odometry.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <math.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

float apparent_baseline_final = 0;

void callback(const chicago::MotorSpeed::ConstPtr& fr, const chicago::MotorSpeed::ConstPtr& fl,
              const nav_msgs::Odometry::ConstPtr& odom) {
  //constants
  float gear_rateo = 0.027027027;
  float wheel_radius = 0.1575;
  //ROS_INFO("rpm r: %f, rpm f: %f, gear_rateo: %f, wheel_radius. %f", fr->rpm, fl->rpm, gear_rateo, wheel_radius);
  // get linear velocities of the wheels
  float Vr = (fr->rpm) * gear_rateo * (M_PI/30) * wheel_radius;
  float Vl = (fl->rpm) * gear_rateo * (M_PI/30) * wheel_radius;
  // get angular velocity of the robot
  float Wz = odom->twist.twist.angular.z;
  // get apparent baseline
  float apparent_baseline = (Vr-Vl)/Wz;
  //ROS_INFO("apparent baseline: %f, Vr: %f, Vl: %f, Wz: %f", apparent_baseline, Vr, Vl, Wz);
  //ros::Duration(0.5).sleep();
  if(abs(apparent_baseline) < 2){
    apparent_baseline_final = (apparent_baseline_final + abs(apparent_baseline))/2;
    ROS_INFO("%f", apparent_baseline_final);
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "apparent_baseline");

  ros::NodeHandle n;

  message_filters::Subscriber<chicago::MotorSpeed> fr(n, "motor_speed_fr", 1);
  message_filters::Subscriber<chicago::MotorSpeed> fl(n, "motor_speed_fl", 1);
  message_filters::Subscriber<nav_msgs::Odometry> odom(n, "scout_odom", 1);
  message_filters::TimeSynchronizer<chicago::MotorSpeed, 
                                    chicago::MotorSpeed, nav_msgs::Odometry> sync(fr, fl, odom, 10);
  sync.registerCallback(boost::bind(&callback, _1, _2, _3));

  ros::spin();

  return 0;
}
