#include "ros/ros.h"
#include "chicago/MotorSpeed.h"
#include "chicago/motor_speeds.h"
#include "geometry_msgs/Vector3Stamped.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include "std_msgs/Float64.h"

class pub_sub
{


  public:
  ros::NodeHandle n;
  chicago::motor_speeds speeds;
  ros::Publisher pub;
  message_filters::Subscriber<chicago::MotorSpeed> fl;
  message_filters::Subscriber<chicago::MotorSpeed> rr;
  message_filters::Subscriber<chicago::MotorSpeed> rl;
    
  public:
    pub_sub(){
      message_filters::Subscriber<chicago::MotorSpeed> fr(n, "/motor_speed_fl", 1);
      message_filters::Subscriber<chicago::MotorSpeed> fl(n, "/motor_speed_fl", 1);
      message_filters::Subscriber<chicago::MotorSpeed> rr(n, "/motor_speed_rr", 1);
      message_filters::Subscriber<chicago::MotorSpeed> rl(n, "/motor_speed_rl", 1);    
      pub = n.advertise<chicago::motor_speeds>("/motor_speeds", 1);
      message_filters::TimeSynchronizer<chicago::MotorSpeed, 
                                      chicago::MotorSpeed,
                                      chicago::MotorSpeed,
                                      chicago::MotorSpeed> sync(fr, fl, rr, rl, 10);

      sync.registerCallback(boost::bind(callback, _1, _2, _3, _4));
  }

  void callback(chicago::MotorSpeed& fr, 
              chicago::MotorSpeed& fl,
              chicago::MotorSpeed& rr,
              chicago::MotorSpeed& rl) {
      speeds.header.stamp = ros::Time::now();
      speeds.header.frame_id = "";
      speeds.rpm_fr=fr.rpm;
      speeds.rpm_fl=fl.rpm;
      speeds.rpm_rr=rr.rpm;
      speeds.rpm_rl=rl.rpm;
      pub.publish(speeds);
  }
};

int main(int argc, char **argv)
{
  ros::NodeHandle n;
  ros::init(argc, argv, "speeds_pub_sub");
  pub_sub my_pub_sub;
  ros::spin();
  return 0;
}
