#include<ros/ros.h>
#include<iostream>
#include<math.h>
#include<cmath>
#include<vector>

#include<nav_msgs/Path.h>
#include<nav_msgs/Odometry.h>
#include<nav_msgs/OccupancyGrid.h>
#include<nav_msgs/OccupancyGrid.h>

#include<geometry_msgs/Point.h>
#include<geometry_msgs/PointStamped.h>
#include<geometry_msgs/PoseWithCovarianceStamped.h>
#include<geometry_msgs/Polygon.h>
#include<geometry_msgs/Twist.h>

#include<visualization_msgs/MarkerArray.h>

#include<ackermann_msgs/AckermannDriveStamped.h>
#include<ackermann_msgs/AckermannDrive.h>

#include<morai_msgs/CtrlCmd.h>
#include<morai_msgs/ObjectStatusList.h>
#include<morai_msgs/GPSMessage.h>

#include<tf/tf.h>

#include<std_msgs/Int32.h>
#include<std_msgs/String.h>

#include<sensor_msgs/PointCloud.h>

#include<pcl_ros/point_cloud.h>

class local_path{
public:
    local_path();
    int speed;
    double L, VL;
    double vehicle_yaw;
    double vehicle_pitch;
    double vehicle_roll;
    double steering_out, prev_error;
    double STEER_KP,STEER_KD;
    double max_lfd, min_lfd;
    double rate = 10;
    bool flag;
    bool is_look_foward_point;
    bool school_zone, avoid, narrow, lane, lane_point;
    int num;

    nav_msgs::Path path;
    nav_msgs::Path trajectory_path;
    nav_msgs::Path tracking_path;
    nav_msgs::Odometry odometry;
    nav_msgs::Odometry odom_for_pub;
    nav_msgs::Odometry object_odom;
    nav_msgs::OccupancyGrid small_map;
    nav_msgs::OccupancyGrid global_map;

    sensor_msgs::PointCloud obstacle;
    sensor_msgs::PointCloud way_pt;

    geometry_msgs::PoseStamped pose;
    geometry_msgs::PoseStamped temp_pose;
    geometry_msgs::PoseStamped goal_pose;
    geometry_msgs::Polygon node_list;
    geometry_msgs::Point prev_point;
    geometry_msgs::Twist lane_twist;

    morai_msgs::CtrlCmd cmd_vel;
    morai_msgs::ObjectStatusList object_info;
    morai_msgs::GPSMessage gps_;

    std_msgs::String state;
    std_msgs::String traffic;

    std::vector<double> object_distance;
    std::vector<double> object_velocity;
    std::vector<double> close_car_dis;
    std::vector<double> close_car_vel;

    //Publish
    ros::Publisher cmd_pub;
    ros::Publisher path_pub;
    ros::Publisher pose_pub;
    ros::Publisher marker_lfd;
    ros::Publisher marker_vel;
    ros::Publisher small_map_pub;
    ros::Publisher local_goal_pub;
    ros::Publisher odom_pub;
    ros::Publisher object_pub;
    //Subscriber
    ros::Subscriber pose_sub;
    ros::Subscriber path_sub;
    ros::Subscriber trajectory;
    ros::Subscriber Odom;
    ros::Subscriber sim_pose;
    ros::Subscriber speed_sub;
    ros::Subscriber map_sub;
    ros::Subscriber obstacle_sub;
    ros::Subscriber state_sub;
    ros::Subscriber object_sub;
    ros::Subscriber traffic_sign;
    ros::Subscriber narrow_way;
    ros::Subscriber gps_sub;
    ros::Subscriber lane_sub;
    ros::Subscriber node_sub;
    //Callback
    void pathcallback(const nav_msgs::Path& msg);
    void trajectorycallback(const nav_msgs::Path& msg);
    void posecallback(const geometry_msgs::PoseStamped& msg);
    void sim_pose_callback(const nav_msgs::Odometry& msg);
    void speed_callback(const std_msgs::Int32 &msg);
    void map_callback(const nav_msgs::OccupancyGrid &msg);
    void obstacle_callback(const sensor_msgs::PointCloud &msg);
    void state_callback(const std_msgs::String &msg);
    void object_callback(const morai_msgs::ObjectStatusList &msg);
    void traffic_sign_callback(const std_msgs::String &msg);
    void narrow_callback(const sensor_msgs::PointCloud &msg);
    void gps_callback(const morai_msgs::GPSMessage &msg);
    void lane_callback(const geometry_msgs::Twist &msg);
    void node_callback(const geometry_msgs::Polygon &msg);
    //Function
    double PID(double desire);
    double pure_pursuit();
    double lidar_tracking();
    double stanley();
    double velocity();
    double nomalize_angle(double angle);
    double obstacle_distance();
    double node_distance();
    void about_object();
    void local_map();
    void path_tracking();
    void process();
};
