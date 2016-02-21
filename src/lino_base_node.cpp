#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/Imu.h>
#include <math.h>

double raw_vel = 0.0;
double vel_dt = 0.0;
double imu_z = 0.0;

ros::Time current_time;
ros::Time vel_time(0.0);
ros::Time last_time(0.0);

void vel_callback( const geometry_msgs::Vector3Stamped& vel) {
  raw_vel = vel.vector.x;
  vel_dt = vel.vector.z;
  vel_time = vel.header.stamp;
}

void imu_callback( const sensor_msgs::Imu& imu){
  // this is to filter out imu noise
  if(imu.angular_velocity.z > -0.005 && imu.angular_velocity.z < 0) 
  {
    imu_z = 0.00;
  }
  else
  {
    imu_z = imu.angular_velocity.z;
  }
}

int main(int argc, char** argv){
  ros::init(argc, argv, "base_controller");
  ros::NodeHandle n;
  ros::NodeHandle nh_private_("~");
  ros::Subscriber sub = n.subscribe("raw_vel", 50, vel_callback);
  ros::Subscriber imu_sub = n.subscribe("imu/data", 50, imu_callback);
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
  tf::TransformBroadcaster odom_broadcaster;

  double rate = 10.0;
  double dt = 0.0;
  double x_pos = 0.0;
  double y_pos = 0.0;
  double theta = 0.0;

  ros::Rate r(rate);
  while(n.ok()){
    ros::spinOnce();
    current_time = ros::Time::now();
       
    dt = vel_dt;
    double dtt = (current_time - last_time).toSec();

    double linear_velocity = raw_vel;
    double angular_velocity = imu_z;
    
    double delta_theta = angular_velocity * dtt; //radians
    double delta_x = (linear_velocity * cos(theta)) * dt; //m
    double delta_y = (linear_velocity * sin(theta)) * dt; //m

    x_pos += delta_x;
    y_pos += delta_y;
    theta += delta_theta;

    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta);
    
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";
    odom_trans.transform.translation.x = x_pos;
    odom_trans.transform.translation.y = y_pos;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;
    odom_trans.header.stamp = current_time;
    odom_broadcaster.sendTransform(odom_trans);
    
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    odom.pose.pose.position.x = x_pos;
    odom.pose.pose.position.y = y_pos;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;
  
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = linear_velocity;
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.linear.z = 0.0;
    
    odom.twist.twist.angular.x = 0.0;
    odom.twist.twist.angular.y = 0.0;
    odom.twist.twist.angular.z = imu_z;

    odom_pub.publish(odom);
    
    last_time = current_time;
    r.sleep();
  }
}
