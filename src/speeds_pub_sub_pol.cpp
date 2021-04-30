#include "ros/ros.h"
#include "chicago/MotorSpeed.h"
#include "chicago/motor_speeds.h"
#include "geometry_msgs/Vector3Stamped.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include "std_msgs/Float64.h"
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>


//TODO: Is this data structure enough ?
typedef struct motor_speeds_input_data
{
    double speed_fr;
    double speed_fl;
    double speed_rr;
    double speed_rl;

    double time_fr;
    double time_fl;
    double time_rr;
    double time_rl;

} InputMotorSpeeds;


class pub_sub
{
  private:
  ros::NodeHandle n;
  ros::Publisher pub;

  public:
  pub_sub(){
    pub = n.advertise<chicago::motor_speeds>("/speeds", 1000);
    message_filters::Subscriber<chicago::MotorSpeed> fr(n, "/motor_speed_fr", 1);
    message_filters::Subscriber<chicago::MotorSpeed> fl(n, "/motor_speed_fl", 1);
    message_filters::Subscriber<chicago::MotorSpeed> rr(n, "/motor_speed_rr", 1);
    message_filters::Subscriber<chicago::MotorSpeed> rl(n, "/motor_speed_rl", 1);

    // policy
    typedef message_filters::sync_policies
    ::ApproximateTime<chicago::MotorSpeed, chicago::MotorSpeed, chicago::MotorSpeed,chicago::MotorSpeed> MySyncPolicy;

    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), fr, fl, rr, rl);

    sync.registerCallback(boost::bind(&pub_sub::callback, this, _1, _2, _3, _4));
    ros::spin();
  }

  void callback(const chicago::MotorSpeed::ConstPtr& fr, const chicago::MotorSpeed::ConstPtr& fl, 
                const chicago::MotorSpeed::ConstPtr& rr, const chicago::MotorSpeed::ConstPtr& rl) {

      InputMotorSpeeds inputMotorSpeeds;

      chicago::motor_speeds speeds;

      speeds.header.stamp = ros::Time::now();
      speeds.header.frame_id = "";
      /*if(fr->rpm != rr->rpm){
        ROS_INFO("non uguali di %f", abs(fr->rpm-rr->rpm));
        ros::Duration(1).sleep();
      }
      if(fl->rpm != rl-> rpm){
        ROS_INFO("non uguali di %f", abs(fl->rpm-rl->rpm));
        ros::Duration(1).sleep();
      }*/
      speeds.rpm_fr=fr->rpm;
      speeds.rpm_fl=fl->rpm;
      speeds.rpm_rr=rr->rpm;
      speeds.rpm_rl=rl->rpm;

      //TODO: verify that it actually takes the timestamps

      //Get the input data from the bag
      inputMotorSpeeds.speed_fr = fr->rpm;
      inputMotorSpeeds.speed_fl = fl->rpm;
      inputMotorSpeeds.speed_rr = rr->rpm;
      inputMotorSpeeds.speed_rl = rl->rpm;

      //Set the timestamp
      inputMotorSpeeds.time_fr = fr->header.stamp.toSec();
      inputMotorSpeeds.time_fl = fl->header.stamp.toSec();
      inputMotorSpeeds.time_rr = rr->header.stamp.toSec();
      inputMotorSpeeds.time_rl = rl->header.stamp.toSec();

      //Publish our own data structure with timestamps
      pub.publish(inputMotorSpeeds);
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "speeds_pub_sub");
  pub_sub my_pub_sub;
  ros::spin();
  return 0;
}
