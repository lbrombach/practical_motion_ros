#include "ros/ros.h"
#include "practical_waypoint_follow/waypoint_server.h"
#include <ros/package.h>
#include <iostream>
#include <fstream>

using namespace std;

// constructor
WaypointServer::WaypointServer(string fileName, bool loop, bool reverseHeadings)
{
    nextWptToPublish = 0;
    currentWpt = -1;
    _loop = loop;
    _reverseHeadings = reverseHeadings;
    if (_loop)
        cout << "Starting server with loop              = true" << endl;
    else
        cout << "Starting server with loop              = false" << endl;
    if (_reverseHeadings)
        cout << "Starting server with reverse_headings  = true" << endl;
    else
        cout << "Starting server with reverse_headings  = false" << endl;

    if (!buildList(fileName))
    {
        ROS_FATAL("UNABLE TO BUILD WAYPOINT LIST FROM FILE - EXITING %s", fileName.c_str());
        ros::shutdown();
    }
}

// utility to add 3.14 radians to a heading and keep in bounds of -3.14 to +3.14
float WaypointServer::reverseHeading(float heading)
{
    return (heading > 0) ? heading - 3.14 : heading + 3.14;
}

// clears the existing waypoint list and loads a new one from file
bool WaypointServer::load_list_srv(practical_motion_msgs::LoadList::Request &req,
                                   practical_motion_msgs::LoadList::Response &res)
{
    // clear existing list
    waypoints.clear();

    // build list from file passed in the Request message
    bool result = buildList(req.listName);

    // start serving waypoints from index 0
    nextWptToPublish = 0;

    // add number of waypoints in new list to response message
    res.numWaypoints = waypoints.size();

    // return success/fail
    return result;
};

// sends the waypoint at the index of the nextWptToPublish counter variable
bool WaypointServer::get_next_srv(practical_motion_msgs::GetNextWaypoint::Request &req,
                                  practical_motion_msgs::GetNextWaypoint::Response &res)
{
    // keep track of last sent so we can identify when the last in list has already been sent
    static int previous = -1;

    // convert next waypoint from object to message and add to response
    res.wpt = waypoints[nextWptToPublish].toMsg();

    // mark the currentWpt so the index is retrievable;
    currentWpt = nextWptToPublish;

    // if true then we have alredy sent the final waypoint. Change the description so the client can identify end of list
    if (nextWptToPublish == previous)
        res.wpt.description = "END OF LIST REACHED";

    previous = nextWptToPublish;

    // if true we are at end of list
    if (nextWptToPublish == waypoints.size() - 1)
    {
        // wrap counter if _loop param is true else remain at index of last waypoint in list
        nextWptToPublish = (_loop) ? 0 : nextWptToPublish;
    }
    // not at end of list - increment counter
    else
    {
        nextWptToPublish++;
    }

    return true;
}

// returns a random waypoint from the list
bool WaypointServer::get_random_srv(practical_motion_msgs::GetRandomWaypoint::Request &req,
                                    practical_motion_msgs::GetRandomWaypoint::Response &res)
{
    int i = rand() % waypoints.size();
    res.wpt = waypoints[i].toMsg();

    // mark the currentWpt so the index is retrievable;
    currentWpt = i;

    return true;
}

// returns most recently sent waypoint without incrementing counter making any other changes
bool WaypointServer::get_current_srv(practical_motion_msgs::GetCurrentWaypoint::Request &req,
                                     practical_motion_msgs::GetCurrentWaypoint::Response &res)
{
    if (currentWpt == -1)
    {
        res.wpt.id = -1;
        res.wpt.description = "invalid";
    }
    else
        res.wpt = waypoints[currentWpt].toMsg();

    return true;
}

// adds a waypoint provided by client to the waypoint list
bool WaypointServer::add_waypoint_srv(practical_motion_msgs::AddWaypoint::Request &req,
                                      practical_motion_msgs::AddWaypoint::Response &res)
{
    // create waypoint object with data from the Request message
    Waypoint temp(req.wpt.id, req.wpt.frame_id, req.wpt.x, req.wpt.y, req.wpt.theta, req.wpt.description, req.wpt.lingerTime);
    cout << "adding" << temp.toString() << endl;

    // add to back of list if requested index is negative
    if (req.index < 0)
        waypoints.push_back(temp);
    // insert waypoint at requested index
    else if (req.index < waypoints.size())
    {
        std::vector<Waypoint>::iterator it = waypoints.begin() + req.index;
        waypoints.insert(it, temp);
    }
    // invalid index
    else
    {
        ROS_WARN("Invalid index, cannot add waypoint");
        return false;
    }

    // add new number of waypoints to response
    res.numWaypoints = waypoints.size();

    return true;
}

// Reverses the order of the waypoints in the list.
bool WaypointServer::reverse_list_srv(std_srvs::Empty::Request &req,
                                      std_srvs::Empty::Response &res)
{
    ROS_INFO("Reversing list");

    // reverse list
    reverse(waypoints.begin(), waypoints.end());

    // reverses headings if reverseHeadings is true and outputs list in new order
    for (int i = 0; i < waypoints.size(); i++)
    {
        if (_reverseHeadings)
            waypoints[i].setTheta(reverseHeading(waypoints[i].getTheta()));
        ROS_INFO("%i  %s", i, waypoints[i].toString().c_str());
    }

    return true;
}

// loads a new list from file
std::vector<Waypoint> WaypointServer::getList(string fileName)
{
    std::vector<Waypoint> list;
    std::string path = ros::package::getPath("practical_waypoint_follow");
    path += "/waypoints/";
    path += fileName;

    std::ifstream inFile;
    std::cout << "opening file: " << path << std::endl;
    inFile.open(path.c_str());
    if (inFile.fail())
    {
        ROS_INFO("UNABLE TO OPEN FILE %s", path.c_str());
    }

    // read file until break condition
    while (1)
    {
        int lingerTime;
        float x, y, theta, id;
        std::string frame, description;
        inFile.ignore(50, '#');
        inFile.ignore(50, '\n');
        if (!(inFile >> id))
            break;
        inFile >> frame;
        inFile.ignore();
        inFile >> x;
        inFile >> y;
        inFile >> theta;
        inFile >> description;
        inFile >> lingerTime;
        inFile.ignore(2, '\n');
        inFile.ignore(2, '\n');

        // if the description of an entry ends in .txt, call this function to load a sublist
        if (description.find(".txt") != std::string::npos)
        {
            ROS_INFO("Loading Sublist! %s", description.c_str());
            std::vector<Waypoint> sublist = getList(description);
            if (sublist.size() > 0)
                list.insert(list.end(), sublist.begin(), sublist.end());
        }
        // otherwise just add the waypoint to the list
        else
        {
            Waypoint temp(id, frame, x, y, theta, description, lingerTime);
            list.push_back(temp);
        }

        if (inFile.eof())
        {
            std::cout << "end of file" << std::endl;
            break;
        }
    }

    inFile.close();

    return list;
};

// file is the file to load.
// TODO: IMPLEMENT - emplaceIndex indicates is the index in the waypoints vector that the list will be inserted into
bool WaypointServer::buildList(std::string fileName, int emplaceIndex)
{
    // get the list
    waypoints = getList(fileName);

    // check that building a list was successful
    if (waypoints.empty())
        return false;

    // output the list to screen
    for (int i = 0; i < waypoints.size(); i++)
    {
        ROS_INFO("%i  %s", i, waypoints[i].toString().c_str());
    }
    return true;
}
