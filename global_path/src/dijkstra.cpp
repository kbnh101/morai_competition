//selfcar
#include "dijkstra.h"

dijkstra::dijkstra()
{
    ros::NodeHandle nh;
    ros::NodeHandle pnh;
    pnh.param("UTM_OFFSET_X",UTM_OFFSET_X,361001.412425);
    pnh.param("UTM_OFFSET_Y",UTM_OFFSET_Y,4065821.07176);

    //subscribe
    map_sub = nh.subscribe("map",10,&dijkstra::MapCallback,this);
    startPoint_sub = nh.subscribe("initialpose", 10, &dijkstra::StartPointCallback,this);
    targetPoint_sub = nh.subscribe("move_base_simple/goal", 10, &dijkstra::TargetPointtCallback,this);
    pose_sub = nh.subscribe("/odom",10,&dijkstra::posecallback,this);
    object_sub = nh.subscribe("/contruction_area",10,&dijkstra::object_callback,this);
    gps_sub = nh.subscribe("gps1",10,&dijkstra::gps_callback,this);

    //publish
    path_pub = nh.advertise<nav_msgs::Path>("path",10);
    marker_pub = nh.advertise<visualization_msgs::MarkerArray>("marker_node", 1);
    speed_pub = nh.advertise<std_msgs::Int32>("speed",10);
    state_pub = nh.advertise<std_msgs::String>("state",10);
    object_marker = nh.advertise<visualization_msgs::MarkerArray>("object_marker",1);
    node_pub = nh.advertise<geometry_msgs::Polygon>("node_pose",10);
}

void dijkstra::posecallback(const nav_msgs::Odometry& msg)
{
    pose = msg.pose.pose;
    pose_point = cv::Point2d(msg.pose.pose.position.x, msg.pose.pose.position.y);
}

void dijkstra::gps_callback(const morai_msgs::GPSMessage &msg)
{
    gps = msg;
    if(gps.longitude == 0 && gps.latitude == 0)
    {
        fuckyou = true;
    }
    else
    {
        fuckyou = false;
    }
}

void dijkstra::MapCallback(const nav_msgs::OccupancyGrid& msg)
{
    map = msg;
    width = 1920;
    height = 1080;
}

void dijkstra::StartPointCallback(const geometry_msgs::PoseWithCovarianceStamped& msg)
{
    start_point = cv::Point2d(msg.pose.pose.position.x, msg.pose.pose.position.y);
    Done = false;
}

void dijkstra::TargetPointtCallback(const geometry_msgs::PoseStamped& msg)
{
    dst_point = cv::Point2d(msg.pose.position.x, msg.pose.position.y);
    flag = true;
}

void dijkstra::object_callback(const geometry_msgs::Polygon &msg)
{
    object = msg;
//    flag = true;
}

double dijkstra::nomalize(double num)
{
    num = num*1000;
    num = round(num);
    num = num/1000;

    return num;
}

void dijkstra::node_parse(istream& file)
{
    Node info;
    if (file.fail())
    {
      cout << "?????? ????????? ???????????? ????????? ???????????? ????????????." << endl;
    }
    while (file.good())
    {
       vector<string> row = csv_read_row(file, ',');
       info.id = atoi(row[0].c_str());
       double x = atoi(row[1].c_str()) - UTM_OFFSET_X;
       double y = atoi(row[2].c_str()) - UTM_OFFSET_Y;
       info.x = nomalize(x);
       info.y = nomalize(y);

       node.push_back(info);
    }
}

void dijkstra::edge_parse(istream& file)
{
    Edge info;
    if (file.fail())
    {
       std::cout << "?????? ????????? ???????????? ????????? ???????????? ????????????." << endl;
    }
    while (file.good())
    {
       vector<string> row = csv_read_row(file, ',');
       istringstream x(row[1]),y(row[2]);
       string wp_x,wp_y;
       vector<string> list_x, list_y;
       double temp_point_x, temp_point_y;

       info.link_id = atoi(row[0].c_str());
       while(getline(x,wp_x,' '))
       {
           list_x.push_back(wp_x);
       }
       while(getline(y,wp_y,' '))
       {
           list_y.push_back(wp_y);
       }
       for(int i = 0; i < list_x.size(); i++)
       {
           temp_point_x = atof(list_x.at(i).c_str()) - UTM_OFFSET_X;
           temp_point_y = atof(list_y.at(i).c_str()) - UTM_OFFSET_Y;

           double nomalize_x = nomalize(temp_point_x);
           double nomalize_y = nomalize(temp_point_y);
           info.way_point_x.push_back(nomalize_x);
           info.way_point_y.push_back(nomalize_y);
       }
       list_x.clear();
       list_y.clear();

       info.from_node = atoi(row[3].c_str());
       info.to_node = atoi(row[4].c_str());
       info.cost = 1;
       info.speed = atoi(row[5].c_str());
       info.state = row[6].c_str();

       edge.push_back(info);
       info.way_point_x.clear();
       info.way_point_y.clear();
    }
    for(int i = 0; i < node.size(); i++)
    {
        for(int j = 0; j < edge.size(); j++)
        {
            if(node.at(i).id == edge.at(j).to_node)
            {
                edge.at(j).way_point_x.push_back(node.at(i).x);
                edge.at(j).way_point_y.push_back(node.at(i).y);
            }
        }
    }
}

void dijkstra::find_path(cv::Point2d start_point, cv::Point2d dst_point)
{
    double short_dist = 10;
    double short_dist1 = 10;
    double Wend_num, Wstart_num, start_num, end_num = 0;
    double start_node, end_node = 0;
    vector<pair<int, int> > adj[node.size() + 1];

    for(int i = 0; i<edge.size(); i++)
    {
        for(int j = 0; j<edge.at(i).way_point_x.size(); j++)
        {
            double dis = (start_point.x - edge.at(i).way_point_x.at(j))*(start_point.x - edge.at(i).way_point_x.at(j)) + (start_point.y - edge.at(i).way_point_y.at(j))*(start_point.y - edge.at(i).way_point_y.at(j));
            double dist = std::sqrt(dis);
            if(dist<short_dist)
            {
                short_dist = dist;
                start_num = i;
                Wstart_num = j;
            }
        }
    }
    start_node = node.at(0).id;
    exstart = Wstart_num;

    for(int i = 0; i<edge.size(); i++)
    {
        for(int j = 0; j<edge.at(i).way_point_x.size(); j++)
        {
            double dis = (dst_point.x - edge.at(i).way_point_x.at(j))*(dst_point.x - edge.at(i).way_point_x.at(j)) + (dst_point.y - edge.at(i).way_point_y.at(j))*(dst_point.y - edge.at(i).way_point_y.at(j));
            double dist = std::sqrt(dis);
            if(dist<short_dist1)
            {
                short_dist1 = dist;
                end_num = i;
                Wend_num = j;
            }
        }
    }
    end_node = node.at(10).id;
    exend = Wend_num;

    vector<int> avoid_index;

    for(int i = 0; i<node.size(); i++)
    {
        for(int j = 0; j<object.points.size(); j++)
        {
            double x = object.points.at(j).x - node.at(i).x;
            double y = object.points.at(j).y - node.at(i).y;

            double dis = sqrt(x*x + y*y);
            if(dis < 5)
            {
                avoid_index.push_back(i);
            }
        }
    }

    for(int i = 0; i<avoid_index.size(); i++)
    {
        int node_num = node.at(avoid_index.at(i)).id;
        for(int j = 0; j<edge.size(); j++)
        {
            if(edge.at(j).from_node == node_num || edge.at(j).to_node == node_num)
            {
                edge.at(j).cost = 20;
            }
        }
    }

    for (int i = 0; i < edge.size(); i++)
    {
        adj[edge.at(i).from_node].push_back(make_pair(edge.at(i).to_node, edge.at(i).cost));
    }

    visualization_msgs::MarkerArray node_arr;
    for (size_t i = 0; i < object.points.size(); i++)
    {
        visualization_msgs::Marker node1;
        node1.header.frame_id = "/map"; // map frame ??????
        node1.header.stamp = ros::Time::now();
        node1.type = visualization_msgs::Marker::SPHERE;
        node1.id = i;
        node1.action = visualization_msgs::Marker::ADD;
        node1.pose.orientation.w = 1.0;
        node1.pose.position.x = object.points.at(i).x; //????????? x ??????
        node1.pose.position.y = object.points.at(i).y; //????????? y ?????? // Points are green
        node1.color.g = 1.0;
        node1.color.a = 1.0;
        node1.scale.x = 5.0;
        node1.scale.y = 5.0;
        node_arr.markers.push_back(node1);
    }

    object_marker.publish(node_arr);

    auto from = dijkstra1(start_node, node.size()+1, adj);
    print_path(start_node, end_node, from);
    shortest_path.push_back(end_node);
    object.points.clear();
    std::cout<<std::endl<<"Done"<<std::endl;
}

void dijkstra::print_path(int s, int e, vector<int>* from) {
    // ?????? trace_path??? ???????????? ?????? ????????? ????????????,
    trace_path(s, e, from);

    // ???????????? ?????? ????????? ????????????.
    std::cout << e;
}

vector<int>* dijkstra::dijkstra1(int start, int V, vector<pair<int,int> > adj[]) {
    int INF = INT32_MAX;

    vector<bool> found(V, false);//??????????????? ???????????? true
    vector<int> dist(V, INF);    // ?????? INF??? ?????????
    priority_queue<pair<int, int> > pq;
    vector<int>* from = new vector<int>(V);//??????????????? ???????????? ?????? ?????? ????????? ???????????? ?????? ??????

    dist[start] = 0;
    //adj[edge.at(i).to_node].push_back(make_pair(edge.at(i).from_node, edge.at(i).cost));
    pq.push(make_pair(0, start));    // ?????? ?????? ??????

    while (!pq.empty())
    {
        int cost = -pq.top().first;    // ????????? ????????? dist ???
        int cur = pq.top().second;    // ?????? ????????? ??????
        pq.pop();

        found[cur] = true;

        for (int i = 0; i < adj[cur].size(); i++) // ?????? ????????? ????????? ?????? ?????? ?????? ??????
        {
            int next = adj[cur][i].first;    // ????????? ?????? ??????
            int nCost = cost + adj[cur][i].second;    // ?????? ????????? ????????? ????????? ?????? ????????? ????????? ??????
            if (nCost < dist[next] )     // ?????? ???????????? ?????? ????????? ????????? ?????? ????????? ??? ?????????
            {
                dist[next] = nCost;    // ??????
                (*from)[next] = cur;

                pq.push(make_pair(-nCost, next));    // pq??? ??????
            }
        }
    }

    return from;
    delete[] from;
}

void dijkstra::trace_path(int s, int e, vector<int>* from) {
    // ?????? ?????? : ???????????? ???????????? ?????? ??????
    if ((*from)[e] == s) {
        cout << s << " -> ";
        shortest_path.push_back((*from)[e]);
        return;
    }
    // ??????????????? ?????? ?????? e?????? ????????? ?????? ????????? ????????????..
    trace_path(s, (*from)[e], from);

    // ?????????????????? ?????? e ?????? ????????? ????????? ????????? ????????????.
    shortest_path.push_back((*from)[e]);
    cout << (*from)[e] << " - > ";
}

void dijkstra::draw_path()
{
    if(flag == true)
    {
        resolution = 150;
        if(Done == false)
        {
            find_path(start_point, dst_point);
        }

        visualization_msgs::MarkerArray node_arr;
        for (size_t i = 0; i < node.size(); i++)
        {
            visualization_msgs::Marker node1;
            node1.header.frame_id = "/map"; // map frame ??????
            node1.header.stamp = ros::Time::now();
            node1.type = visualization_msgs::Marker::SPHERE;
            node1.id = i;
            node1.action = visualization_msgs::Marker::ADD;
            node1.pose.orientation.w = 1.0;
            node1.pose.position.x = node.at(i).x; //????????? x ??????
            node1.pose.position.y = node.at(i).y; //????????? y ?????? // Points are green
            node1.color.r = 1.0;
            node1.color.a = 1.0;
            node1.scale.x = 5.0;
            node1.scale.y = 5.0;
            node_arr.markers.push_back(node1);
        }
        marker_pub .publish(node_arr);

        nav_msgs::Path rpath;
        geometry_msgs::PoseStamped temp_pose;
        temp_pose.header.stamp = ros::Time::now();
        temp_pose.header.frame_id = "map";
        rpath.header.stamp = ros::Time::now();
        rpath.header.frame_id = "map";
        rpath.poses.clear();
        for(int j = 1; j<shortest_path.size(); j++)
        {
            for(int i = 0; i<edge.size(); i++)
            {
                int to = shortest_path.at(j);
                int from = shortest_path.at(j - 1);

                if(edge.at(i).to_node == to && edge.at(i).from_node == from)
                {
                    path_info cpPath;
                    for(int k = 0; k<edge.at(i).way_point_x.size();k++)
                    {
                        double x = edge.at(i).way_point_x.at(k);
                        double y = edge.at(i).way_point_y.at(k);
                        double vel = edge.at(i).speed;
                        temp_pose.pose.position.x = x;
                        temp_pose.pose.position.y = y;
                        temp_pose.pose.position.z = 0;
                        cpPath.x = x;
                        cpPath.y = y;
                        cpPath.vel = vel;
                        cpPath.state = edge.at(i).state;
                        rpath.poses.push_back(temp_pose);
                        pathinfo.push_back(cpPath);
                    }
                }
            }
        }
        if(Done == false)
        {
            double x = rpath.poses.at(1).pose.position.x - rpath.poses.at(0).pose.position.x;
            double y = rpath.poses.at(1).pose.position.y - rpath.poses.at(0).pose.position.y;

            double cyaw = atan2(y,x);

            std::cout<<cyaw<<std::endl;
            path_pub.publish(rpath);
            rpath.poses.clear();
            shortest_path.clear();

            for(int i = 0; i<node.size(); i++)
            {
                geometry_msgs::Point32 temp_point;
                temp_point.x = node.at(i).x;
                temp_point.y = node.at(i).y;

                node_pose.points.push_back(temp_point);
            }

            node_pub.publish(node_pose);
            node_pose.points.clear();
        }
        double index = 0;
        double min_dist = 20;

        for(int i = 0; i < pathinfo.size(); i++)
        {
            double x = pose.position.x - pathinfo.at(i).x;
            double y = pose.position.y - pathinfo.at(i).y;
            double dist = sqrt((x * x)+(y * y));
            if(min_dist > dist)
            {
                index = i;
                min_dist = dist;
            }
        }

        std::stringstream state_stream;
        std_msgs::String state;

        state_stream <<pathinfo.at(index).state;
        state.data = state_stream.str();

        speed.data = pathinfo.at(index).vel;

        if(speed != speed)
        {
            ROS_INFO_STREAM("speed = "<<speed);
        }
        speed_pub.publish(speed);
        if(fuckyou == false)
        {
            state_pub.publish(state);
        }
        else
        {
            state.data = "";
            state_pub.publish(state);
        }
        if(state.data == "end" || state.data == "rotate")
        {
            Done = false;
            flag = true;
        }
        else
        {
            Done = true;
        }
    }
    else
    {
        ROS_INFO("CAN'T FIND_PATH");
    }
}

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "dijkstra");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");

    dijkstra path;

    path.flag = false;
    path.Done = false;

    ifstream file_edge("/home/a/test_ws/src/morai/global_path/path_data/edge_final4.csv");
    ifstream file_node("/home/a/test_ws/src/morai/global_path/path_data/node_final4.csv");

    path.node_parse(file_node);
    path.edge_parse(file_edge);

    file_edge.close();
    file_node.close();

    ros::Rate  loop_rate(10);

    while(ros::ok())
    {
        path.draw_path();
        loop_rate.sleep();
        ros::spinOnce();
    }

    ros::spin();
    return 0;
}
