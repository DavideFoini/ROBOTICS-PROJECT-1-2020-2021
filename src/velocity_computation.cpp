#include "ros/ros.h"
#include "chicago/MotorSpeed.h"
#include "chicago/motor_speeds.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "geometry_msgs/TwistStamped.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include "std_msgs/Float64.h"
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

class pub_sub
{
  private:
  ros::NodeHandle n;
  ros::Publisher pub;
  ros::Subscriber sub;

  public:
  pub_sub(){
    pub = n.advertise<geometry_msgs::TwistStamped>("/twist", 1000);
    sub = n.subscribe("/speeds", 1000, &pub_sub::callback, this);
    ros::spin();
  }

  void callback(const chicago::motor_speeds::ConstPtr& speeds){
      // parameters
      float gear_rateo;
      n.getParam("/gear_rateo", gear_rateo);
      float wheel_radius;
      n.getParam("/wheel_radius", wheel_radius);
      float real_baseline;
      n.getParam("/wheel_radius", real_baseline);
      // linear velocities
      float r_vel = (speeds->rpm_fr + speeds->rpm_rr) / (2*wheel_radius);
      float l_vel = (speeds->rpm_fl + speeds->rpm_rl) / (2*wheel_radius);
      // angular velocity
      float w = (r_vel + l_vel)/real_baseline;
      // rotation radius
      float rotation_radius = (real_baseline/2)*((r_vel+l_vel)/(r_vel-l_vel));

      float apparent_baseline = rotation_radius * (r_vel-l_vel)/(r_vel+l_vel);
      ROS_INFO("%f", apparent_baseline);
      //pub.publish();
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "velocity_computation");
  pub_sub my_pub_sub;
  ros::spin();
  return 0;
}
