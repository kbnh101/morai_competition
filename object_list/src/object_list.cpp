#include "object_list.h"

object_list::object_list()
{
    ros::NodeHandle nh;
    //subscribe
    object_info = nh.subscribe("/Object_topic_adsc",10,&object_list::object_callback,this);
    //publish
    object_pub = nh.advertise<geometry_msgs::Polygon>("/contruction_area",10);
}

void object_list::object_callback(const morai_msgs::ObjectStatusList &msg)
{
    object = msg;
}

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "object_list");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");

    object_list info;

    ros::Rate  loop_rate(10);

    geometry_msgs::Point32 thing;

    while(ros::ok())
    {    for(int i = 0; i<info.object.npc_list.size(); i++)
        {
            thing.x = info.object.npc_list.at(i).position.x;
            thing.y = info.object.npc_list.at(i).position.y;

            info.info.points.push_back(thing);
        }

        info.object_pub.publish(info.info);
        info.info.points.clear();
        loop_rate.sleep();
        ros::spinOnce();
    }

    ros::spin();
    return 0;
}
