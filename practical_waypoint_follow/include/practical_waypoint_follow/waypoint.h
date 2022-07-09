#ifndef WAYPOINTS_H
#define WAYPOINTS_H

#include <move_base_msgs/MoveBaseAction.h>
#include <string>
#include "practical_motion_msgs/Waypoint.h"

// This class is the waypoint object used through the practical_waypoint_follow package
class Waypoint
{
	// The ID of the waypoint. Not used by this package - for the users own purposes
	int id;

	// the frame that the waypoint is referenced in. would usually be "map" but might be "base_link" or "odom" or "gps"
	std::string frame_id;

	// the x coordinate
	float x;

	// the y coordinate
	float y;

	// the desired heading upon reaching a waypoint. Unit is radians, from -3.14 - +3.14
	float theta;

	// a description for use by the user
	std::string description;

	// how long to linger at a specific waypoint before moving on to the next
	int lingerTime;

public:
	Waypoint();

	// copy constructor
	Waypoint(const Waypoint &wpt);

	// standard constructor
	Waypoint(int id, std::string frame, float x, float y, float theta, std::string description, int lingerTime);

	// convert the waypoint to the type of message required by move_base
	move_base_msgs::MoveBaseGoal toGoal();

	// convert the data in this object to a message passable by the waypoint_server
	practical_motion_msgs::Waypoint toMsg();

	// getters
	int getId() { return id; }
	std::string getFrame() { return frame_id; }
	float getX() { return x; }
	float getY() { return y; }
	float getTheta() { return theta; }
	std::string getDescription() { return description; }
	int getLingerTime() { return lingerTime; }

	// setters
	float setTheta(float theta);

	// formats data as printable string
	std::string toString();
};

#endif
