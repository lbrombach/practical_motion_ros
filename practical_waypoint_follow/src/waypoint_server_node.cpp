#include "ros/ros.h"
#include <iostream>
#include "practical_waypoint_follow/waypoint_server.h"

using namespace std;

const string DEFAULT_FILENAME = "waypoints.txt";

int main(int argc, char **argv)
{
    // standard ros node setup stuff
    ros::init(argc, argv, "waypoint_server");
    ros::NodeHandle node("~");

    // check the loop parameter. True means server will loop back to start when list complete
    bool loop = false;
    node.getParam("loop", loop);

    // check the reverse_headings parameter. True means that goal headings will be also reversed upon calling reverse_list
    bool reverseHeadings = false;
    node.getParam("reverse_headings", reverseHeadings);

    // check for command line filename argument
    string fileName = DEFAULT_FILENAME;
    if (argc > 2)
    {
        ROS_FATAL("Invalid number of arguments - program only accepts a file name");
        return 1;
    }
    if (argc == 2)
    {
        fileName = argv[1];
        fileName = fileName;
    }

    // create server
    WaypointServer waypointServer(fileName, loop, reverseHeadings);

    // create and advertise the services this node will provide
    ros::ServiceServer service0 = node.advertiseService("load_list", &WaypointServer::load_list_srv, &waypointServer);
    ros::ServiceServer service1 = node.advertiseService("get_next_waypoint", &WaypointServer::get_next_srv, &waypointServer);
    ros::ServiceServer service2 = node.advertiseService("get_current_waypoint", &WaypointServer::get_current_srv, &waypointServer);
    ros::ServiceServer service3 = node.advertiseService("get_random_waypoint", &WaypointServer::get_random_srv, &waypointServer);
    ros::ServiceServer service4 = node.advertiseService("reverse_list", &WaypointServer::reverse_list_srv, &waypointServer);
    ros::ServiceServer service5 = node.advertiseService("add_waypoint", &WaypointServer::add_waypoint_srv, &waypointServer);

    // spin to check for service calls at this rate in Hz
    ros::Rate loop_rate = 1;
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
