#ifndef WAYPOINT_SERVER_H
#define WAYPOINT_SERVER_H

#include <ros/ros.h>
#include "practical_waypoint_follow/waypoint.h"
#include "practical_motion_msgs/LoadList.h"
#include "practical_motion_msgs/GetNextWaypoint.h"
#include "practical_motion_msgs/GetCurrentWaypoint.h"
#include "practical_motion_msgs/GetRandomWaypoint.h"
#include "practical_motion_msgs/AddWaypoint.h"
#include "practical_motion_msgs/Waypoint.h"
#include "std_srvs/Empty.h"
#include <string>
#include <vector>

using namespace std;

class WaypointServer
{
    std::vector<Waypoint> waypoints;

    // The index of the next wapypoint that will be returned
    int nextWptToPublish;

    // The index of the last waypoint sent via one of the get services
    int currentWpt;

    // set from param.. if true keep looping when complete. Defaults false
    bool _loop;

    // set from param. if true, goal headings are also reversed when a call to reverse_list is made (so a heading of 0 becomes 3.14)
    bool _reverseHeadings;

    // utility to add 3.14 radians to a heading and keep in bounds of -3.14 to +3.14
    float reverseHeading(float heading);

public:
    // constructor
    WaypointServer(string fileName, bool loop = false, bool reverseHeadings = false);

    // builds list of waypoints by calling getList. Can use getList to create a superlist out of one or more waypoint list files
    bool buildList(string file, int emplaceIndex = 0);

    // reads the file male list of waypoints
    std::vector<Waypoint> getList(string fileName);

    // loads a new list. Replaces any list already loaded in memory.
    bool load_list_srv(practical_motion_msgs::LoadList::Request &req,
                       practical_motion_msgs::LoadList::Response &res);

    // publishes the next waypoint in the list
    bool get_next_srv(practical_motion_msgs::GetNextWaypoint::Request &req,
                      practical_motion_msgs::GetNextWaypoint::Response &res);

    // returns most recently sent waypoint without incrementing counter making any other changes
    bool get_current_srv(practical_motion_msgs::GetCurrentWaypoint::Request &req,
                         practical_motion_msgs::GetCurrentWaypoint::Response &res);

    // publishes the next waypoint in the list
    bool get_random_srv(practical_motion_msgs::GetRandomWaypoint::Request &req,
                        practical_motion_msgs::GetRandomWaypoint::Response &res);

    // Adds the waypoint provided in the Request to the waypoint list at the index also requested in the Request
    // A negative requested index will push to back of list
    bool add_waypoint_srv(practical_motion_msgs::AddWaypoint::Request &req,
                          practical_motion_msgs::AddWaypoint::Response &res);

    // Reverses the order of the waypoints in the list.
    bool reverse_list_srv(std_srvs::Empty::Request &req,
                          std_srvs::Empty::Response &res);
};

#endif