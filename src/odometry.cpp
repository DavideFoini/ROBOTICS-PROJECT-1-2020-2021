#include "ros/ros.h"
#include "chicago/MotorSpeed.h"
#include "chicago/custom_odom.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <math.h>

class pub_sub
{
  private:
    ros::NodeHandle n;
    // publishers
    ros::Publisher twist_pub;
    ros::Publisher custom_pub;
    // parameters
    float gear_rateo, wheel_radius, apparent_baseline;
    int method;
    // status
    float x, y, theta, v, w;
    // messages
    geometry_msgs::TwistStamped twist;
    chicago::custom_odom custom_odom;
    nav_msgs::Odometry odom;
    geometry_msgs::TransformStamped transformStamped;
    geometry_msgs::Quaternion quat_msg;
    // times
    ros::Time current_time;
    ros::Time last_time;
    // TF broadcaster
    tf2_ros::TransformBroadcaster br;
    // deltas
    float dx, dy, dth, dt;
    //others
    tf2::Quaternion odom_quat;
    std_msgs::String str_method;

  public:
    pub_sub(){
      // set publishers
      custom_pub = n.advertise<chicago::custom_odom>("/custom_odom", 1000);
      twist_pub = n.advertise<geometry_msgs::TwistStamped>("/twist", 1000);
      // message filters
      message_filters::Subscriber<chicago::MotorSpeed> fr(n, "/motor_speed_fr", 10);
      message_filters::Subscriber<chicago::MotorSpeed> fl(n, "/motor_speed_fl", 10);
      message_filters::Subscriber<chicago::MotorSpeed> rr(n, "/motor_speed_rr", 10);
      message_filters::Subscriber<chicago::MotorSpeed> rl(n, "/motor_speed_rl", 10);
      message_filters::TimeSynchronizer<chicago::MotorSpeed, chicago::MotorSpeed,
                                        chicago::MotorSpeed, chicago::MotorSpeed> sync(fr, fl, rr, rl, 10);
      sync.registerCallback(boost::bind(&pub_sub::callback, this, _1, _2, _3, _4));
      // set parameters
      n.getParam("/method", method);
      n.getParam("/gear_rateo", gear_rateo);
      n.getParam("/wheel_radius", wheel_radius);
      n.getParam("/apparent_baseline", apparent_baseline);
      // set starting status
      n.getParam("/x", x);
      n.getParam("/y", y);
      n.getParam("/th", theta);
      v = 0;
      w = 0;
      // set times
      current_time = ros::Time::now();
      last_time = ros::Time::now();
      // set deltas
      dx = 0;
      dy = 0;
      dth = 0;
      dt = 0;

      ros::spin();
    }

    void callback(const chicago::MotorSpeed::ConstPtr& fr, const chicago::MotorSpeed::ConstPtr& fl, 
                  const chicago::MotorSpeed::ConstPtr& rr, const chicago::MotorSpeed::ConstPtr& rl) {
      // update current_time
      current_time = ros::Time::now();
      // compute velocities
      comp_vels(fr->rpm, fl->rpm, rr->rpm, rl->rpm);
      // compute and publish twist
      comp_twist();
      // compute odometry
      comp_odom();
      // build and publish custom_message
      comp_custom_message();
      // build and publish TF
      comp_TF();
      // update last time
      last_time = current_time;
    }

    // compute linear and angular velocities of the robot from rpms
    void comp_vels(float fr, float fl, float rr, float rl){
      // from rpms to linear velocities
      float v_r = (((fr + rr) / 2) * gear_rateo) * (M_PI / 30) * wheel_radius;
      float v_l = (((fl + rl) / 2) * (-1) * gear_rateo) * (M_PI / 30) * wheel_radius;
      // set linear velocity
      v = (v_r + v_l) / 2;
      // set angular velocity
      w = (v_r - v_l)/apparent_baseline;
    }

    // build and publish twistStamped message
    void comp_twist(){
      twist.header.stamp = current_time;
      twist.header.frame_id = "base_link";
      twist.twist.linear.x = v;
      twist.twist.linear.y = 0.0;
      twist.twist.linear.z = 0.0;
      twist.twist.angular.x = 0.0;
      twist.twist.angular.y = 0.0;
      twist.twist.angular.z = w;
      twist_pub.publish(twist);  
    }

    // compute odometry from velocities
    void comp_odom(){
      dt = (current_time - last_time).toSec();
      dth = w * dt;
      // compute deltas based on the set integration method
      if(method == 0)
        comp_odom_euler();
      else if (method == 1)
        comp_odom_rk();
      else
        ROS_ERROR("fatal error: integration method not set");
      // update pose
      x += dx;
      y += dy;
      theta += dth;
      // update pose parameters
      n.setParam("/x", x);
      n.setParam("/y", y);
      n.setParam("/th", theta);
    }

    // odometry with euler method
    void comp_odom_euler(){
      dx = v * dt * cos(theta);
      dy = v * dt * sin(theta);
    }

    // odometry with runge-kutta method
    void comp_odom_rk(){
      dx = v * dt * cos (theta + ((w * dt) / 2));
      dy = v * dt * sin (theta + ((w * dt) / 2));
    }

    // build and publish custom_odom message
    void comp_custom_message(){
      // build message
      odom.header.stamp = current_time;
      odom.header.frame_id = "world";
      odom.child_frame_id = "base_link";
      odom.pose.pose.position.x = x;
      odom.pose.pose.position.y = y;
      odom.pose.pose.position.z = 0;
      odom_quat.setRPY(0, 0, theta);
      quat_msg = tf2::toMsg(odom_quat);
      odom.pose.pose.orientation = quat_msg;
      odom.twist.twist = twist.twist;
      if(method == 0)
        str_method.data = "euler";
      else if(method == 1)
        str_method.data = "rk";
      else
        ROS_ERROR("fatal error: integration method not set");
      custom_odom.odom = odom;
      custom_odom.method = str_method;
      //publish message
      custom_pub.publish(custom_odom);
    }

    // build and publish TF message
    void comp_TF(){
      // build TF message
      transformStamped.header.stamp = current_time;
      transformStamped.header.frame_id = "world";
      transformStamped.child_frame_id = "base_link";
      transformStamped.transform.translation.x = x;
      transformStamped.transform.translation.y = y;
      transformStamped.transform.translation.z = 0.0;
      transformStamped.transform.rotation = odom.pose.pose.orientation;
      // publish TF message
      br.sendTransform(transformStamped);
    }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "odometry");
  pub_sub my_pub_sub;
  ros::spin();
  return 0;
}
