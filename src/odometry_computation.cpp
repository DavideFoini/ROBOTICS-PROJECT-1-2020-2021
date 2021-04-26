#include "ros/ros.h"
#include "chicago/odom.h"
#include "chicago/MotorSpeed.h"
#include "chicago/motor_speeds.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/String.h"
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <math.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf/transform_broadcaster.h>

class pub_sub
{
  private:
  ros::NodeHandle n;
  ros::Publisher pub;
  ros::Subscriber sub;
  chicago::odom odom;
  ros::Time last_time;
  bool first;

  public:
  pub_sub(){
    pub = n.advertise<chicago::odom>("/odometry", 1000);
    sub = n.subscribe("/twist", 1000, &pub_sub::callback, this);
    last_time = ros::Time::now();
    first = true;
    ros::spin();
  }

  void callback(const geometry_msgs::TwistStamped::ConstPtr& twist){
      float x, y, th, dt = 0;
      int method;
      std_msgs::String str_method;
      // get parameters
      n.getParam("/position.x", x);
      n.getParam("/position.y", y);
      n.getParam("/position.th", th);
      n.getParam("/method", method);
      odom.odom.header.stamp = ros::Time::now();
      odom.odom.header.frame_id = "odom";
      odom.odom.child_frame_id = "scout";

      if(!first)
       dt = (twist->header.stamp - last_time).toSec();

      if(first)
        first = false;

      last_time = twist->header.stamp;

      // compute new pose
      // if method 0 use euler
      if(method == 0){
        x = x + (twist->twist.linear.x * dt * cos(th));
        y = y + (twist->twist.linear.x * dt * sin(th));
        th = th + (twist->twist.angular.z * dt);
      }

      if(method == 1){
        // TO DO runge kutta
      }

      // updating parameters
      n.setParam("/position.x", x);
      n.setParam("/position.y", y);
      n.setParam("/position.th", th);

      // set pose
      odom.odom.pose.pose.position.x = x;
      odom.odom.pose.pose.position.y = y;
      odom.odom.pose.pose.position.z = 0;
      geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);
      odom.odom.pose.pose.orientation = odom_quat;

      // set twist
      odom.odom.twist.twist = twist->twist;

      //set method
      if(method == 0)
        str_method.data = "euler";
      else
        str_method.data = "rk";

      odom.method = str_method;

      // TO DO tf transform

      pub.publish(odom);
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "odometry_computation");
  pub_sub my_pub_sub;
  ros::spin();
  return 0;
}
