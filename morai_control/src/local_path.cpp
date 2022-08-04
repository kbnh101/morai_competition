#include "local_path.h"

local_path::local_path()
{
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    pnh.param("L", L, 0.8);
    pnh.param("VL",VL,1.2);
    pnh.param("max_lfd", max_lfd, 5.0);
    pnh.param("min_lfd", min_lfd, 1.0);
    pnh.param("STEER_KP",STEER_KP,1.0);
    pnh.param("STEER_KD",STEER_KD,0.0);
    pnh.param("num",num,30);

    //publish
    cmd_pub = nh.advertise<morai_msgs::CtrlCmd>("/ctrl_cmd",10);
    path_pub = nh.advertise<nav_msgs::Path>("tracking_path",10);
    pose_pub = nh.advertise<geometry_msgs::PoseStamped>("current_pose",10);
    marker_lfd = nh.advertise<visualization_msgs::MarkerArray>("/Look_Forward_Distance",10);
    marker_vel = nh.advertise<visualization_msgs::MarkerArray>("/marker_vel",1);
    small_map_pub = nh.advertise<nav_msgs::OccupancyGrid>("/local_map",10);
    odom_pub = nh.advertise<nav_msgs::Odometry>("/odom_",10);
    local_goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/local_goal",10);
    object_pub = nh.advertise<morai_msgs::ObjectStatusList>("object_position",10);
    //subscibe
    //pose_sub = nh.subscribe("/slam_out_pose",10,&local_path::posecallback,this);
    sim_pose = nh.subscribe("/odom",10,&local_path::sim_pose_callback,this);
    path_sub = nh.subscribe("/path",10,&local_path::pathcallback,this);
    trajectory = nh.subscribe("/trajectory",10,&local_path::trajectorycallback,this);
    speed_sub = nh.subscribe("/speed",10,&local_path::speed_callback,this);
    map_sub = nh.subscribe("map",10,&local_path::map_callback,this);
    obstacle_sub = nh.subscribe("center_points",10,&local_path::obstacle_callback,this);
    state_sub = nh.subscribe("state",10,&local_path::state_callback,this);
    object_sub = nh.subscribe("/Object_topic_adsc",10,&local_path::object_callback,this);
    traffic_sign = nh.subscribe("/perception/Detection/TrafficSign",100,&local_path::traffic_sign_callback,this);
    narrow_way = nh.subscribe("way_points",10,&local_path::narrow_callback,this);
    gps_sub = nh.subscribe("/gps",10,&local_path::gps_callback,this);
    lane_sub = nh.subscribe("lane",10,&local_path::lane_callback,this);
    node_sub = nh.subscribe("node_pose",10,&local_path::node_callback,this);
}

void local_path::state_callback(const std_msgs::String &msg)
{
    if(msg.data != "")
    {
        state = msg;
    }
}

void local_path::node_callback(const geometry_msgs::Polygon &msg)
{
    node_list = msg;
    int i =0;
}

void local_path::gps_callback(const morai_msgs::GPSMessage &msg)
{
    gps_ = msg;
    if(state.data == "narrow")
    {
        gps_.latitude = 0;
        gps_.longitude = 0;
        pose.pose.position.x = 0;
        pose.pose.position.y = 0;
        if(gps_.latitude == 0 && gps_.longitude == 0)
        {
            lane = false;
            narrow = true;
            lane_point = false;
        }
        else
        {
            lane = false;
            narrow = false;
            lane_point = false;
        }
    }

    else if(state.data == "lane")
    {
        gps_.latitude = 0;
        gps_.longitude = 0;
        pose.pose.position.x=0;
        pose.pose.position.y = 0;
        if(gps_.latitude == 0 && gps_.longitude == 0)
        {
            lane = true;
            lane_point = true;
            narrow = false;
        }
        else
        {
            lane = false;
            narrow = false;
            lane_point = false;
        }
    }
    else
    {
        narrow = false;
        lane = false;
    }
}

void local_path::lane_callback(const geometry_msgs::Twist &msg)
{
    lane_twist = msg;
}

void local_path::traffic_sign_callback(const std_msgs::String &msg)
{
    traffic = msg;
}

void local_path::narrow_callback(const sensor_msgs::PointCloud &msg)
{
    way_pt = msg;
}

void local_path::trajectorycallback(const nav_msgs::Path& msg)
{
    trajectory_path = msg;
}

void local_path::object_callback(const morai_msgs::ObjectStatusList &msg)
{
    object_info = msg;
    object_pub.publish(object_info);
}

void local_path::obstacle_callback(const sensor_msgs::PointCloud &msg)
{
    obstacle = msg;
}

void local_path::map_callback(const nav_msgs::OccupancyGrid &msg)
{
    global_map = msg;
}

void local_path::pathcallback(const nav_msgs::Path& msg)
{
    path = msg;
    flag = true;
}

void local_path::sim_pose_callback(const nav_msgs::Odometry &msg)
{
    odometry = msg;
    pose.pose = msg.pose.pose;
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "map";
    tf::Quaternion car_orientation(pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w);
    tf::Matrix3x3 matrix(car_orientation);
    matrix.getRPY(vehicle_roll, vehicle_pitch, vehicle_yaw);
    //lidar_debug
//    pose.pose.position.x = 0;
//    pose.pose.position.y = 0;
    //end

    temp_pose.header.stamp = ros::Time::now();
    temp_pose.header.frame_id = "odom";
    temp_pose.pose.position.x = 0;
    temp_pose.pose.position.y = 0;
    temp_pose.pose.position.z = 0;
//    temp_pose.pose.orientation.x = pose.pose.orientation.x;
//    temp_pose.pose.orientation.y = pose.pose.orientation.y;
//    temp_pose.pose.orientation.z = pose.pose.orientation.z;
    temp_pose.pose.orientation.x = 0;
    temp_pose.pose.orientation.y = 0;
    temp_pose.pose.orientation.z = 0;
    temp_pose.pose.orientation.w = 1;

    pose_pub.publish(temp_pose);
}

void local_path::speed_callback(const std_msgs::Int32 &msg)
{
    speed = msg.data;
}

void local_path::posecallback(const geometry_msgs::PoseStamped& msg)
{
    pose = msg;
    tf::Quaternion car_orientation(pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w);
    tf::Matrix3x3 matrix(car_orientation);
    matrix.getRPY(vehicle_roll, vehicle_pitch, vehicle_yaw);

    temp_pose.header.stamp = ros::Time::now();
    temp_pose.header.frame_id = "map";
    temp_pose.pose.position.x = (msg.pose.position.x);
    temp_pose.pose.position.y = (msg.pose.position.y);
    temp_pose.pose.position.z = 0;
    temp_pose.pose.orientation.x = msg.pose.orientation.x;
    temp_pose.pose.orientation.y = msg.pose.orientation.y;
    temp_pose.pose.orientation.z = msg.pose.orientation.z;
    temp_pose.pose.orientation.w = msg.pose.orientation.w;

    //std::cout<<vehicle_yaw<<std::endl;

    pose_pub.publish(temp_pose);
}

double local_path::node_distance()
{
    if(node_list.points.size() > 0)
    {
        std::vector<geometry_msgs::Point32> temp_point;

        temp_point.push_back(node_list.points.at(14));
        temp_point.push_back(node_list.points.at(16));
        temp_point.push_back(node_list.points.at(18));
        temp_point.push_back(node_list.points.at(20));
        temp_point.push_back(node_list.points.at(24));
        temp_point.push_back(node_list.points.at(27));
        temp_point.push_back(node_list.points.at(30));
        //temp_point.push_back(node_list.points.at(32));
        temp_point.push_back(node_list.points.at(35));
        temp_point.push_back(node_list.points.at(36));
        temp_point.push_back(node_list.points.at(37));

        double index = 0;
        double min_dis = 50;

        for(int i = 0; i<temp_point.size(); i++)
        {
            double dx = temp_point.at(i).x - pose.pose.position.x;
            double dy = temp_point.at(i).y - pose.pose.position.y;

            double dis = sqrt(dx*dx + dy*dy);

            if(min_dis > dis)
            {
                min_dis = dis;
                index = i;
            }
        }
        double tx = temp_point.at(index).x - pose.pose.position.x;
        double ty = temp_point.at(index).y - pose.pose.position.y;

        double dist = sqrt(tx*tx + ty*ty);

        return dist;
    }
}

void local_path::path_tracking()
{
    if(flag == true)
    {
        double least_dist = 10;
        int temp_num;
        int num = this->num;

        for(int i = 0; i<path.poses.size(); i++)
        {
            double dx = pose.pose.position.x - path.poses.at(i).pose.position.x;
            double dy = pose.pose.position.y - path.poses.at(i).pose.position.y;

            double dist = sqrt(dx*dx + dy*dy);
            if(dist<least_dist)
            {
                least_dist = dist;
                temp_num = i;
            }
        }

//        double avoid_index = 0;
//        double short_distance;
//        double center_path,right_path,left_path;

//        if(avoid == true)
//        {
//            for(int i = 0; i<tracking_path.poses.size(); i++)
//            {
//                double dx = obstacle.points.at(0).x - tracking_path.poses.at(i).pose.position.x;
//                double dy = obstacle.points.at(0).y - tracking_path.poses.at(i).pose.position.y;

//                double dis = sqrt(dx*dx + dy*dy);
//                if(short_distance > dis)
//                {
//                    short_distance = dis;
//                    avoid_index = i;
//                }
//            }
//        }

        tracking_path.header.stamp = ros::Time::now();
        tracking_path.header.frame_id = "map";
        tracking_path.poses.clear();

        geometry_msgs::PoseStamped temp_pose;
        temp_pose.header.stamp = ros::Time::now();
        temp_pose.header.frame_id = "map";

        if(temp_num + num <= path.poses.size())
        {
            for(int i = temp_num; i< temp_num + num; i++)
            {
                if(avoid == true)
                {
                    temp_pose.pose.position.x = path.poses.at(i).pose.position.x + 4*sin(vehicle_yaw);
                    temp_pose.pose.position.y = path.poses.at(i).pose.position.y - 4*cos(vehicle_yaw);
                    temp_pose.pose.position.z = 0;
                    tracking_path.poses.push_back(temp_pose);
                }
                else
                {
                    temp_pose.pose.position.x = path.poses.at(i).pose.position.x;
                    temp_pose.pose.position.y = path.poses.at(i).pose.position.y;
                    temp_pose.pose.position.z = 0;
                    tracking_path.poses.push_back(temp_pose);
                }

//                temp_pose.pose.position.x = path.poses.at(i).pose.position.x - 4*sin(vehicle_yaw);
//                temp_pose.pose.position.y = path.poses.at(i).pose.position.y + 4*cos(vehicle_yaw);
//                temp_pose.pose.position.z = 0;
//                tracking_path1.poses.push_back(temp_pose);
            }
        }
        else
        {
            flag = false;
        }
        path_pub.publish(tracking_path);
    }
    else
    {
        ROS_INFO("DONE");
    }
}

double local_path::nomalize_angle(double angle)
{
    while(angle > M_PI)
    {
        angle -= 2.0 * M_PI;
    }
    while(angle < M_PI)
    {
        angle += 2.0 * M_PI;
    }
    return angle;
}

double local_path::velocity()
{
   double vel = 1;

   return vel;
}

double local_path::PID(double desire)
{
    double steer_error;
    steer_error = desire;
    double steer_derivative = (steer_error - prev_error)*rate;
    steering_out = STEER_KP * steer_error + STEER_KD *steer_derivative; //+ STEER_KI*steer_integral;

    if (steering_out >= 0.35)
        steering_out = 0.35;
    if (steering_out <= -0.35)
        steering_out = -0.35;

    prev_error = steer_error;

    return steering_out;
}

double local_path::stanley()
{
    double px,py,cyaw;
    double min_dist = 1e9;
    double min_index = 0;
    double k = 0.5;

    geometry_msgs::PoseStamped path_pose = tracking_path.poses.at(0);
    geometry_msgs::PoseStamped next_pose = tracking_path.poses.at(1);

    px = next_pose.pose.position.x - path_pose.pose.position.x;
    py = next_pose.pose.position.y - path_pose.pose.position.y;

    cyaw = atan2(py,px);

    double front_x = pose.pose.position.x + 0.1*cos(vehicle_yaw);
    double front_y = pose.pose.position.y + 0.1*sin(vehicle_yaw);

    for(int i = 0; i<tracking_path.poses.size(); i++)
    {
        double dx = front_x - tracking_path.poses.at(i).pose.position.x;
        double dy = front_y - tracking_path.poses.at(i).pose.position.y;

        double dist = sqrt(dx*dx + dy*dy);
        if(dist<min_dist)
        {
            min_dist = dist;
            min_index = i;
        }
    }

    double map_x = tracking_path.poses.at(min_index).pose.position.x;
    double map_y = tracking_path.poses.at(min_index).pose.position.y;

    double dx = map_x - front_x;
    double dy = map_y - front_y;
    double vec_x = cos(vehicle_yaw + M_PI/2);
    double vec_y = sin(vehicle_yaw + M_PI/2);

    double cte = dx*vec_x + dy*vec_y;

    double yaw_term = cyaw - vehicle_yaw;
    double cte_term = atan2(k*cte, velocity());

    double steering = (yaw_term + cte_term);

    return steering;
}

double local_path::lidar_tracking()
{
    double min_dist = 10;
    double min_index = 0;
    double back_x = - 1.7;
    double back_y = 0;

    for(int i = 0; i<way_pt.points.size(); i++)
    {
        double dx = back_x - way_pt.points.at(i).x;
        double dy = back_y - way_pt.points.at(i).y;

        double dist = sqrt(dx*dx + dy*dy);
        if(dist<min_dist)
        {
            min_dist = dist;
            min_index = i;
        }
    }

    double x = back_x - way_pt.points.at(min_index).x;
    double y = back_y - way_pt.points.at(min_index).y;


    double distance = sqrt(pow(x,2) + pow(y,2));
    double dis = 0;

    double lfd = 1;
    double max_lfd = this->max_lfd;
    double min_lfd = 5;
    double rotated_x = 0;
    double rotated_y = 0;

    lfd = cmd_vel.velocity/5.0;

    if(lfd < min_lfd)
    {
        lfd = min_lfd;
    }
    else if(lfd > max_lfd)
    {
        lfd = max_lfd;
    }

    double dx,dy = 0;

    for(int i = 0; i<way_pt.points.size(); i++)
    {
        dx = way_pt.points.at(i).x - 1.7;
        dy = way_pt.points.at(i).y;

        if(dx > 0)
        {
            dis = sqrt(pow(dx,2) + pow(dy,2));
            if(dis>=lfd)
            {
                is_look_foward_point = true;
                break;
            }
        }
    }

    double theta = atan2(dy,dx);
    double steering = 0;
    if(is_look_foward_point == true)
    {
        double eta = atan2((2*VL*sin(theta)),lfd);
        steering = eta;
    }
    else
    {
        ROS_INFO("no found forwad point");
    }

    return steering;
}

double local_path::pure_pursuit()
{
    geometry_msgs::Pose index;
    is_look_foward_point = false;
    double min_dist = 10;
    double min_index = 0;
    double back_x = pose.pose.position.x - L*cos(vehicle_yaw);
    double back_y = pose.pose.position.y - L*sin(vehicle_yaw);

    for(int i = 0; i<path.poses.size(); i++)
    {
        double dx = back_x - path.poses.at(i).pose.position.x;
        double dy = back_y - path.poses.at(i).pose.position.y;

        double dist = sqrt(dx*dx + dy*dy);
        if(dist<min_dist)
        {
            min_dist = dist;
            min_index = i;
        }
    }

    double x = back_x - path.poses.at(min_index).pose.position.x;
    double y = back_y - path.poses.at(min_index).pose.position.y;

    double distance = sqrt(pow(x,2) + pow(y,2));
    double dis = 0;

    double lfd = 1;
    double max_lfd = this->max_lfd;
    double min_lfd = this->min_lfd;
    double rotated_x = 0;
    double rotated_y = 0;

    lfd = cmd_vel.velocity/8.0;

    if(lfd < min_lfd)
    {
        lfd = min_lfd;
    }
    else if(lfd > max_lfd)
    {
        lfd = max_lfd;
    }

    for(int i = 0; i<tracking_path.poses.size(); i++)
    {
        double dx = tracking_path.poses.at(i).pose.position.x - back_x;
        double dy = tracking_path.poses.at(i).pose.position.y - back_y;

        rotated_x = cos(vehicle_yaw)*dx + sin(vehicle_yaw)*dy;
        rotated_y = sin(vehicle_yaw)*dx - cos(vehicle_yaw)*dy;

        if(rotated_x > 0)
        {
            dis = sqrt(pow(rotated_x,2) + pow(rotated_y,2));
            if(dis>=lfd)
            {
                index.position.x = tracking_path.poses.at(i).pose.position.x;
                index.position.y = tracking_path.poses.at(i).pose.position.y;
                is_look_foward_point = true;
                break;
            }
        }
    }

    //ROS_INFO_STREAM( "Look_Forward_Distance = " << lfd);
    //lfd_marker
    visualization_msgs::MarkerArray node_arr;
    visualization_msgs::Marker node1;
    node1.header.frame_id = "/map"; // map frame 기준
    node1.header.stamp = ros::Time::now();
    node1.type = visualization_msgs::Marker::SPHERE;
    node1.id = 0;
    node1.action = visualization_msgs::Marker::ADD;
    node1.pose.orientation.w = 1.0;
    node1.pose.position.x = index.position.x; //노드의 x 좌표
    node1.pose.position.y = index.position.y; //노드의 y 좌표 // Points are green
    node1.color.g = 0.5;
    node1.color.a = 1.0;
    node1.scale.x = 1;
    node1.scale.y = 1;
    node_arr.markers.push_back(node1);
    marker_lfd.publish(node_arr);
    //end

    double theta = atan2(rotated_y,rotated_x);
    double steering = 0;
    if(is_look_foward_point == true)
    {
        double eta = atan2((2*VL*sin(theta)),lfd);
        steering = -eta;
    }
    else
    {
        ROS_INFO("no found forwad point");
    }

    return steering;
}

double local_path::obstacle_distance()
{
    if(school_zone = true && obstacle.points.size() > 0)
    {
        double min_dis = 20;
        int path_index = 0;
        int lidar_index = 0;

        for(int i = 0; i<obstacle.points.size(); i++)
        {
            for(int j = 0; j<tracking_path.poses.size(); j++)
            {
                double tx = obstacle.points.at(i).x;
                double ty = obstacle.points.at(i).y;
                double rotated_x = cos(vehicle_yaw)*tx - sin(vehicle_yaw)*ty;
                double rotated_y = sin(vehicle_yaw)*tx + cos(vehicle_yaw)*ty;

                double x = rotated_x + pose.pose.position.x;
                double y = rotated_y + pose.pose.position.y;

                double dx = x - tracking_path.poses.at(j).pose.position.x;
                double dy = y - tracking_path.poses.at(j).pose.position.y;
                double dis = sqrt(dx*dx + dy*dy);

                if(min_dis > dis)
                {
                    min_dis = dis;
                    path_index = j;
                    lidar_index = i;
                }
            }
        }

        double dx = obstacle.points.at(lidar_index).x;
        double dy = obstacle.points.at(lidar_index).y;

        double dist = sqrt(dx*dx + dy*dy);

        if(min_dis <= 4 && min_dis > 2)
        {
            cmd_vel.velocity = 18;
        }
        else if(min_dis <= 2 || dist <= 2)
        {
            cmd_vel.velocity = 0;
        }

        return cmd_vel.velocity;
    }

}

void local_path::local_map()
{
    double x = 60;
    double y = 20;
    small_map.header.stamp = ros::Time::now();
    small_map.header.frame_id = "local";
    small_map.info.map_load_time = ros::Time::now();
    small_map.info.resolution = 1;
    small_map.info.width = x;
    small_map.info.height = y;
    small_map.info.origin.position.x = -x/2;
    small_map.info.origin.position.y = -y/2;
    small_map.info.origin.orientation.w = 1;
    small_map.data.assign(x*y,0);

    double goal_x = goal_pose.pose.position.x - x/2;
    double goal_y = goal_pose.pose.position.y - y/2;
    //small_map.data.at(goal_x*goal_y) = 255;

    small_map_pub.publish(small_map);
}

void local_path::about_object()
{
    int index;

    for(int i = 0; i<object_info.npc_list.size(); i++)
    {
        double x = object_info.npc_list.at(i).position.x - odometry.pose.pose.position.x;
        double y = object_info.npc_list.at(i).position.y - odometry.pose.pose.position.y;

        double dis = sqrt(x*x + y*y);

        double vx = object_info.npc_list.at(i).velocity.x;
        double vy = object_info.npc_list.at(i).velocity.y;
        double vz = object_info.npc_list.at(i).velocity.z;

        double vel = sqrt(vx*vx + vy*vy + vz*vz);

        object_distance.push_back(dis);
        object_velocity.push_back(vel);
    }
}

void local_path::process()
{
    //experiments
    nav_msgs::Path temp_path;
    temp_path.header.stamp = ros::Time::now();
    temp_path.header.frame_id = "map";
    cmd_vel.longlCmdType = 2;

    path_tracking();
    local_map();
    about_object();

    if(state.data == "school_zone")
    {
        school_zone = true;
        avoid = false;
    }
    else if(state.data == "static")
    {
        school_zone = false;
        double min_dis = 20;
        int path_index = 0;
        int lidar_index = 0;

        for(int i = 0; i<obstacle.points.size(); i++)
        {
            for(int j = 0; j<tracking_path.poses.size(); j++)
            {
                double tx = obstacle.points.at(i).x;
                double ty = obstacle.points.at(i).y;
                double rotated_x = cos(vehicle_yaw)*tx - sin(vehicle_yaw)*ty;
                double rotated_y = sin(vehicle_yaw)*tx + cos(vehicle_yaw)*ty;

                double x = rotated_x + pose.pose.position.x;
                double y = rotated_y + pose.pose.position.y;

                double dx = x - tracking_path.poses.at(j).pose.position.x;
                double dy = y - tracking_path.poses.at(j).pose.position.y;
                double dis = sqrt(dx*dx + dy*dy);

                if(min_dis > dis)
                {
                    min_dis = dis;
                    path_index = j;
                    lidar_index = i;
                }
            }
        }

        if(obstacle.points.size() > 0 && min_dis <= 3)
        {
            avoid = true;
        }
        else
        {
//            avoid = false;
        }
    }
    else if(state.data == "end")
    {
        flag = true;
    }
    else
    {
        avoid = false;
        school_zone = false;
    }

    if(flag == true && narrow == false)
    {
        cmd_vel.steering = pure_pursuit();
        if(is_look_foward_point == true)
        {
            if(state.data == "no" || state.data == "end" || state.data == "narrow" || state.data == "highway" || state.data == "lane" || state.data == "rotate")
            {
//                if(traffic.data == "RED" && node_distance() < 10 && node_distance() >= 8)
//                {
//                    cmd_vel.velocity = 10;
//                }
                if(traffic.data == "RED" && node_distance() < 14 && node_distance() >= 8)
                {
                    cmd_vel.velocity = 10;
                }
                if(traffic.data == "RED" && node_distance() < 8)
                {
                    cmd_vel.velocity = 0;
                }
                else if(traffic.data != "RED")
                {
                    lane_point = false;
                    cmd_vel.velocity = speed;
                    double close_car_index = 0;
                    for(int i = 0; i<tracking_path.poses.size(); i++)
                    {
                        for(int j = 0; j<object_info.npc_list.size(); j++)
                        {
                            double dx = tracking_path.poses.at(i).pose.position.x - object_info.npc_list.at(j).position.x;
                            double dy = tracking_path.poses.at(i).pose.position.y - object_info.npc_list.at(j).position.y;

                            double dis = sqrt(dx*dx + dy*dy);
                            if(dis <= 2)
                            {
                                close_car_index = j;
                                close_car_dis.push_back(object_distance.at(j));
                                close_car_vel.push_back(object_velocity.at(j));
                                for(int k = 1; k<close_car_dis.size();k++)
                                {
                                    if(close_car_dis.at(k-1) == close_car_dis.at(k))
                                    {
                                        close_car_dis.pop_back();
                                        close_car_vel.pop_back();
                                    }
                                }
                            }
                        }
                    }
                    for(int i = 0; i<close_car_dis.size(); i++)
                    {
                        if(close_car_dis.at(i) <= 20 && close_car_dis.at(i) > 8)
                        {
                            cmd_vel.velocity = close_car_vel.at(0);
                        }
                        else if(close_car_dis.at(i) <= 8 && close_car_dis.at(i) >= 2)
                        {
                            cmd_vel.velocity = close_car_vel.at(0) - 10;
                            if(cmd_vel.velocity <= 0)
                            {
                                cmd_vel.velocity = 0;
                            }
                        }
                        else if(close_car_dis.at(i) <= 2)
                        {
                            cmd_vel.velocity = 0;
                        }
                    }
                }
            }
            else if(state.data == "static")
            {
                cmd_vel.velocity = speed;
                if(obstacle.points.size() > 0 && traffic.data != "RED")
                {
                    cmd_vel.velocity = speed - 10;
                }
                else if(traffic.data == "RED" && node_distance() < 10 && node_distance() >= 5)
                {
                    cmd_vel.velocity = 10;
                }
                else if(traffic.data == "RED" && node_distance() < 5)
                {
                    cmd_vel.velocity = 0;
                }
            }
            else if(state.data == "school_zone")
            {
                cmd_vel.velocity = speed;
                if(traffic.data != "RED" && obstacle.points.size() > 0)
                {
                    cmd_vel.velocity = obstacle_distance();
                }
                else if(traffic.data == "RED" && node_distance() < 14 && node_distance() >= 8)
                {
                    cmd_vel.velocity = 10;
                }
                else if(traffic.data == "RED" && node_distance() < 7)
                {
                    cmd_vel.velocity = 0;
                }

            }
        }
        else
        {
            cmd_vel.velocity = 0;
        }
    }

    else if(narrow == true)
    {
        cmd_vel.longlCmdType = 2;
        if(way_pt.points.size() > 0)
        {
            cmd_vel.steering = lidar_tracking();
        }
        else
        {
            cmd_vel.steering = 0;
        }
        cmd_vel.velocity = 15;
    }

    else if(lane == true)
    {
        cmd_vel.longlCmdType = 2;
//        if(lane_point == true)
//        {
            cmd_vel.steering = -lane_twist.angular.z;
//        }
//        else
//        {
//            cmd_vel.steering = 0;
//        }
        cmd_vel.velocity = lane_twist.linear.x;
    }
    else if(flag==false && narrow == false && lane == false)
    {
        cmd_vel.steering = 0;
        cmd_vel.velocity = 0;
    }
    cmd_pub.publish(cmd_vel);
    object_distance.clear();
    object_velocity.clear();
    obstacle.points.clear();
    close_car_dis.clear();
    close_car_vel.clear();

    visualization_msgs::MarkerArray node_arr;
    visualization_msgs::Marker node1;
    if(node_list.points.size()>0)
    {
        node1.header.frame_id = "map"; // map frame 기준
        node1.header.stamp = ros::Time::now();
        node1.type = visualization_msgs::Marker::SPHERE;
        node1.id = 0;
        node1.action = visualization_msgs::Marker::ADD;
        node1.pose.orientation.w = 1.0;
        node1.pose.position.x = node_list.points.at(14).x; //노드의 x 좌표
        node1.pose.position.y = node_list.points.at(14).y; //노드의 y 좌표 // Points are green
        node1.color.g = 0.5;
        node1.color.a = 1.0;
        node1.scale.x = 5;
        node1.scale.y = 5;
        node_arr.markers.push_back(node1);
        marker_vel.publish(node_arr);
    }

}

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "local_path");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");

    local_path path;
    path.flag = false;

    ros::Rate loop_rate(path.rate);
    while(ros::ok())
    {
        path.process();
        loop_rate.sleep();
        ros::spinOnce();
    }

    ros::spin();
    return 0;
}
