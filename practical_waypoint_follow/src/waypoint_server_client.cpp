/***********************************************************************************************************
 * waypoint_server_client.cpp is intended to serve as an example for how to use the services provided by the
 * waypoint_server_node and send the waypoints as goals to move_base.
 *
 * March 2022
 * Author: Lloyd Brombach
 * Contact: lbrombach2@gmail.com
 ************************************************************************************************************/

// always include for ros nodes
#include "ros/ros.h"

// must include to use an action. We will be using move_base action server
#include <actionlib/client/simple_action_client.h>

// required message for the move_base action server
#include <move_base_msgs/MoveBaseAction.h>

// we must also include the .h file that corresponds with each service that we intend to call
#include <practical_motion_msgs/AddWaypoint.h>
#include <practical_motion_msgs/GetNextWaypoint.h>
#include <practical_motion_msgs/GetCurrentWaypoint.h>
#include <practical_motion_msgs/GetRandomWaypoint.h>
#include <practical_motion_msgs/LoadList.h>
#include "practical_waypoint_follow/waypoint.h"
#include <std_srvs/Empty.h>

#include <iostream>

// this is required to create a move base action server client
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

// convenience
using namespace std;

// helper to convert the Waypoint message we will receive from service calls to the object of the Waypoint data type
// In other words: practical_motion_msgs::Waypoint.msg --> practical_waypoint_follow/waypoint.h
Waypoint waypoint_message_to_object(practical_motion_msgs::Waypoint msg)
{
    // constructor: Waypoint::Waypoint(int id, std::string frame, float x, float y, float theta, std::string description, int lingerTime)
    return Waypoint(msg.id, msg.frame_id, msg.x, msg.y, msg.theta, msg.description, msg.lingerTime);
}

// main function
int main(int argc, char **argv)
{

    // standard node setup - register with ros master and create a NodeHandle
    ros::init(argc, argv, "waypoint_server_client");
    ros::NodeHandle node("~");

    // create a client for each service we intend to use, using the NodeHandle we just created.
    // arguments are:  node.serviceClient<packageWhereServiceDefined::srvFileName>(nameOfService);
    ros::ServiceClient addWptClient = node.serviceClient<practical_motion_msgs::AddWaypoint>("/waypoint_server/add_waypoint");
    ros::ServiceClient getNextClient = node.serviceClient<practical_motion_msgs::GetNextWaypoint>("/waypoint_server/get_next_waypoint");
    ros::ServiceClient getCurrentClient = node.serviceClient<practical_motion_msgs::GetCurrentWaypoint>("/waypoint_server/get_current_waypoint");
    ros::ServiceClient getRandomClient = node.serviceClient<practical_motion_msgs::GetRandomWaypoint>("/waypoint_server/get_random_waypoint");
    ros::ServiceClient loadListClient = node.serviceClient<practical_motion_msgs::LoadList>("/waypoint_server/load_list");
    ros::ServiceClient reverseListClient = node.serviceClient<std_srvs::Empty>("/waypoint_server/reverse_list");

    // it is good practice to make sure service is available before trying to call it
    addWptClient.waitForExistence();
    getNextClient.waitForExistence();
    getCurrentClient.waitForExistence();
    getRandomClient.waitForExistence();
    loadListClient.waitForExistence();
    reverseListClient.waitForExistence();

    /*****************************************************************************************************************
     * This section of code shows how to load a new waypoint file. Any existing waypoints loaded are erased.
     ****************************************************************************************************************/
    // We must create the request and response objects of the service type we want to call. The data required
    // for each is listed in practical_motion_msgs/readme.md or the corresponding .srv file
    practical_motion_msgs::LoadList::Request req;
    practical_motion_msgs::LoadList::Response res;

    // a load_list service call requires a valid file name
    req.listName = "waypoints.txt";

    // the response will be filled by the service, so we don't need to do anything more here

    // call the service and pass the Request and Response
    loadListClient.call(req, res);

    // Output the Response data, which is the number of waypoints in the newly loaded file
    cout << "The loaded list contains " << res.numWaypoints << " waypoints" << endl;

    /*****************************************************************************************************************
     * This section of code shows how to reverse the order of a waypoint list loaded in the server. Since
     * reverse_list takes a std_srvs::Empty message, we don't have to do anything but create it and call the service
     *****************************************************************************************************************/
    std_srvs::Empty msg;
    reverseListClient.call(msg);

    /*****************************************************************************************************************
     * This section of code shows how to request the next waypoint in the list.
     *****************************************************************************************************************/
    // Create the request and response of the appropriate type, but for get_next_client the request and respose are empty
    practical_motion_msgs::GetNextWaypoint::Request req2;
    practical_motion_msgs::GetNextWaypoint::Response res2;

    // send the empty request and response, and the service will fill out the response
    getNextClient.call(req2, res2);

    // the response contains a waypoint message that must be converted to a waypoint object used throughout this package.
    // Convert it with the helper function defined earlier in this program
    Waypoint nextWaypoint(waypoint_message_to_object(res2.wpt));

    // The Waypoint class has a toString() function
    cout << "The next waypoint is: " << endl
         << nextWaypoint.toString() << endl;

    /*****************************************************************************************************************
     * This section of code shows how to request the most previously sent waypoint again without incrementing the counter
     * It is essentially identical to requesting the "next" waypoint, except the GetCurrentWaypoint service (srv)
     * file must be used instead. The user should be careful to handle cases where no waypoint has been fetched yet by
     * checking if the id is -1 or the description reads "invalid"
     * GetNextWaypoint.srv and GetRandomWaypoint.srv differ in name only - the contents are the same.
     *****************************************************************************************************************/
    // Create the request and response of the appropriate type, but for get_next_client the request and respose are empty
    practical_motion_msgs::GetCurrentWaypoint::Request req3;
    practical_motion_msgs::GetCurrentWaypoint::Response res3;

    // send the empty request and response, and the service will fill out the response
    getCurrentClient.call(req3, res3);

    // the response contains a waypoint message that must be converted to a waypoint object used throughout this package.
    // Convert it with the helper function defined earlier in this program
    Waypoint currentWaypoint(waypoint_message_to_object(res3.wpt));

    // The Waypoint class has a toString() function
    cout << "The current waypoint is: " << endl
         << currentWaypoint.toString() << endl;

    // If no waypoints have been served yet there will be no current to return so the server will return a waypoint
    // message with the id of -1 and the description "invalid"
    if (res3.wpt.id == -1 || res3.wpt.description == "invalid")
        cout << "This waypoint is not valid" << endl;
    else
        cout << "This waypoint is valid" << endl;

    /*****************************************************************************************************************
     * This section of code shows how to request a random waypoint from the list. It is essentially identical to
     * requesting the "next" waypoint, except the GetRandomWaypoint service (srv) file must be used instead.
     * GetNextWaypoint.srv and GetRandomWaypoint.srv differ in name only - the contents are the same.
     *****************************************************************************************************************/
    practical_motion_msgs::GetRandomWaypoint::Request req4;
    practical_motion_msgs::GetRandomWaypoint::Response res4;
    getRandomClient.call(req4, res4);
    Waypoint randomWaypoint(waypoint_message_to_object(res4.wpt));
    cout << "The random waypoint is: " << endl
         << randomWaypoint.toString() << endl;

    /*****************************************************************************************************************
     * This section of code shows how to add a waypoint to the list loaded in the waypoint_server_node
     *****************************************************************************************************************/

    // Create a waypoint object
    //  constructor: Waypoint::Waypoint(int id, std::string frame, float x, float y, float theta, std::string description, int lingerTime)
    Waypoint newWaypoint(1, "map", 2.5, 4.1, 1.57, "liquor_store", 30);

    // create request and response objects
    practical_motion_msgs::AddWaypoint::Request req5;
    practical_motion_msgs::AddWaypoint::Response res5;

    // the add_waypoint request requires and index you want to insert the waypoint to. A negative value means "push to the back"
    req5.index = -1;

    // the add_waypoint request requires a waypoint message be sent. The Waypoint class has a function to convert the object to a message
    req5.wpt = newWaypoint.toMsg();

    // call the service
    cout << "Adding waypoint ...  " << endl;
    addWptClient.call(req5, res5);

    // the response contains the new number of waypoints in the list
    cout << "The list now has " << res5.numWaypoints << " waypoints" << endl;

    /*****************************************************************************************************************
     * This section of code shows how to send a waypoint to move_base as a goal. This section is commented because
     * if move_base is not running it will cause the program to hang. If you want to test it, uncomment and recompile.
     *
     *****************************************************************************************************************/

    /*
    // create a move_base action server client
    MoveBaseClient mbc("move_base", true);

    // wait for the server to be available
    while (!mbc.waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    // Send the goal. However, move_base goals are different than waypoint objects. Use the toGoal() function to send the waypoint as a goal message
    mbc.sendGoal(nextWaypoint.toGoal());

    // Actions take time to complete and provide a result. You could do other things or just wait as we are here.
    mbc.waitForResult();

    // check the result message so we know if we made it to the goal or not.
    if (mbc.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("Goal Reached");
    else
        ROS_INFO("Goal NOT Reached");
        */

    return 0;
}