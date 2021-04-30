#include "ros/ros.h"
#include "chicago/custom_odom.h"
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
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>



class pub_sub
{
private:
    ros::NodeHandle n;
    ros::Publisher pub_odom;
    ros::Publisher pub_custom;
    ros::Subscriber sub;
    chicago::custom_odom custom_odom;
    nav_msgs::Odometry odom;
    ros::Time last_time;
    bool first;
    tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;
    float x, y, th, dt = 0;



public:
    pub_sub(){
        pub_custom = n.advertise<chicago::custom_odom>("/custom_odometry", 1000);
        pub_odom = n.advertise<nav_msgs::Odometry>("/odometry", 1000);
        sub = n.subscribe("/twist", 1000, &pub_sub::callback, this);
        last_time;
        first = true;
        ros::spin();
    }



    void computeEuler(const geometry_msgs::TwistStamped::ConstPtr& twist){
        x = x + (twist->twist.linear.x * dt * cos(th));
        y = y + (twist->twist.linear.x * dt * sin(th));
        th = th + (twist->twist.angular.z * dt);
    }



    void computeRk(const geometry_msgs::TwistStamped::ConstPtr& twist){
        x = x + (twist->twist.linear.x * dt * cos(th + ((twist->twist.angular.z * dt) / 2)));
        y = y + (twist->twist.linear.x * dt * sin(th + ((twist->twist.angular.z * dt) / 2)));
        th = th + (twist->twist.angular.z * dt);
    }



    void callback(const geometry_msgs::TwistStamped::ConstPtr& twist){
        int method;
        std_msgs::String str_method;
// get parameters
        n.getParam("/x", x);
        n.getParam("/y", y);
        n.getParam("/th", th);
        n.getParam("/method", method);
        odom.header.stamp = twist->header.stamp;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link";



        if(!first)
            dt = (twist->header.stamp - last_time).toSec();



        if(first)
            first = false;



        last_time = twist->header.stamp;



//Compute the odometry with Euler or RK methods
//Method is defined through a class variable enum, odometryType
        if(method == 0)
            computeEuler(twist);
        else
            computeRk(twist);





// updating parameters
        n.setParam("/x", x);
        n.setParam("/y", y);
        n.setParam("/th", th);



// set pose
        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0;
        tf2::Quaternion odom_quat;
        odom_quat.setRPY(0, 0, th);
        geometry_msgs::Quaternion quat_msg = tf2::toMsg(odom_quat);
        odom.pose.pose.orientation = quat_msg;



// set twist
        odom.twist.twist = twist->twist;




//Do we need the code below ?




        if(method == 0)
            str_method.data = "euler";
        else
            str_method.data = "rk";



        custom_odom.odom = odom;
        custom_odom.method = str_method;



// publish messages
        pub_custom.publish(custom_odom);
        pub_odom.publish(odom);



// set TF
        transformStamped.header.stamp = twist->header.stamp;
        transformStamped.header.frame_id = "world";
        transformStamped.child_frame_id = "base_link";
        transformStamped.transform.translation.x = x;
        transformStamped.transform.translation.y = y;
        transformStamped.transform.translation.z = 0.0;
        transformStamped.transform.rotation.x = odom_quat.x();
        transformStamped.transform.rotation.y = odom_quat.y();
        transformStamped.transform.rotation.z = odom_quat.z();
        transformStamped.transform.rotation.w = odom_quat.w();



// send TF
        br.sendTransform(transformStamped);
    }
};



int main(int argc, char **argv)
{
    ros::init(argc, argv, "odometry_computation");
    pub_sub my_pub_sub;
    ros::spin();
    return 0;
}