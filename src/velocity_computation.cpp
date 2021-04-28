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
#include <math.h>

class pub_sub
{
  private:
  ros::NodeHandle n;
  ros::Publisher pub;
  ros::Subscriber sub;
  geometry_msgs::TwistStamped twist;

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
      float apparent_baseline;
      n.getParam("/apparent_baseline", apparent_baseline);
      // linear velocities
      float v_r = (((speeds->rpm_fr + speeds->rpm_rr) / 2) * gear_rateo) * (M_PI / 30) * wheel_radius;
      float v_l = (((speeds->rpm_fl + speeds->rpm_rl) / 2) * gear_rateo) * (M_PI / 30) * wheel_radius;
      float v_x = (v_r + v_l) / 2;
      // angular velocity of the robot
      float w = (v_r - v_l)/apparent_baseline;
      // generating twist message
      twist.header.stamp = speeds->header.stamp;
      twist.header.frame_id = "base_link";
      twist.twist.linear.x = v_x;
      twist.twist.linear.y = 0.0;
      twist.twist.linear.z = 0.0;
      twist.twist.angular.x = 0.0;
      twist.twist.angular.y = 0.0;
      twist.twist.angular.z = w;

      pub.publish(twist);
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "velocity_computation");
  pub_sub my_pub_sub;
  ros::spin();
  return 0;
}
