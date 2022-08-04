#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

nav_msgs::Odometry odom;
double vehicle_yaw, vehicle_pitch, vehicle_roll;

void odomcallback(const nav_msgs::Odometry& msg)
{
    odom = msg;
    tf::Quaternion car_orientation(odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w);
    tf::Matrix3x3 matrix(car_orientation);
    matrix.getRPY(vehicle_roll, vehicle_pitch, vehicle_yaw);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "tf_make");
  ros::NodeHandle n;
  ros::Subscriber pose_sub = n.subscribe("/odom",10,&odomcallback);
  ros::Rate r(100);


  tf::TransformBroadcaster map_odom;
  tf::TransformBroadcaster odom_lidar;
  tf::TransformBroadcaster odom_local;

  while(n.ok()){
     map_odom.sendTransform(
       tf::StampedTransform(
         tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(odom.pose.pose.position.x, odom.pose.pose.position.y, 0.0)),
         ros::Time::now(),"map", "odom"));

    odom_lidar.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, vehicle_yaw, 1), tf::Vector3(0.7, 0.0, 0.3)),
        ros::Time::now(),"odom", "lidar"));

    odom_local.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, vehicle_yaw, 1), tf::Vector3(0.7, 0.0, 0.3)),
        ros::Time::now(),"odom", "local"));
    r.sleep();
   }
 }
