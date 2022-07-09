# Practical Motion MSGS
A package to build custom messages and service definitions for use with the practical_motion_ros collection of packages. For more information about the structure of message and service definitions, see the [Creating a ROS msg and srv tutorial](http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv).
 
 
<br>
<br>

# Messages Defined

## Waypoint:
Contains the elements of a waypoint object, as defined in [waypoint.h](../practical_waypoint_follow/include/practical_waypoint_follow/waypoint.h). The practical_motion_msgs::Waypoint differs from other waypoint messages in that the heading data is described with an Euler angle rather than a quaternion orientaion for the purpose of human readbility. The waypoint_server_node and waypoint_follow will convert these heading angles to quaternions, and required by the [move_base action server](http://wiki.ros.org/navigation/Tutorials/SendingSimpleGoals). 
<br>

- int16 id                     
    The (optional) id number of the waypoint
- string frame_id              
    The frame of reference the goal will be issued in. ie: map, odom, base_link etc
- float32 x                    
    The x coordinate of the goal
- float32 y                    
    the y coordinate of the goal
- float32 theta                
    The desired final heading of the goal as a simple euler angle using radians. -3.14 to +3.14
- string description           
    A descriptive name of the goal. 
- int16 lingerTime             
    how long to linger at a waypoint before moving on to the next. In seconds. (only works if handling node enable the waiting behavior)

<br><br>

# Services Defined
Services use a pair of messages deines as the request message and the response message. These request and response messages can be empty or hold one or more primitives or other messages.
<br><br>

## AddWaypoint:
A service provided by the waypoint_server to add a waypoint to the list currently loaded.

Request:
- int16 index
- Waypoint wpt

Response:
- int16 numWaypoints
<br><br>


## GetNextWaypoint:
A service provided by the waypoint_server that returns the next waypoint from the loaded list.

Request:
- none

Response:
- Waypoint wpt
<br><br>

## GetCurrentWaypoint:
A service provided by the waypoint_server that returns most recently sent waypoint without incrementing counter making any other changes. Returns a message with id = -1 and description "invalid"

Request:
- none

Response:
- Waypoint wpt
<br><br>

## GetRandomWaypoint:
A service provided by the waypoint_server that returns a random waypoint from the loaded list.

Request:
- none

Response:
- Waypoint wpt
<br><br>


## LoadList:
A service provided by the waypoint_server that loads the list with the name passed in. Returns the number of waypoints in the list that was loaded.

Request:
- string listName

Response:
- int16 numWaypoints


<br><hr>

## Contributing, bug reports, etc:
Please use the issues and bug reporting system for bugs and feature requests. With many projects, a job, a family, I can't promise
to get to feature requests very quickly, but am definitely listening for feedback to make improvement and will prioritize bugs. I am open to
pull requests if you'd like to contribute. I can be contacted by email at lbrombach2@gmail.com. 