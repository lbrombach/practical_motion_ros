#ifndef WAYPOINT_FOLLOW_H
#define WAYPOINT_FOLLOW_H

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "practical_waypoint_follow/waypoint.h"
#include <string>
#include <vector>

//necessary to use move_base action server
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class Follower
{

	Follower();

	ros::NodeHandle *node;
	MoveBaseClient *mbcPtr;
	std::vector<Waypoint> waypoints;

	// set from param.. if true keep looping when complete
	bool _loop;

	// if true return to start when done. Ignored if _continuous == true
	bool _return;

	std::string filePath;

	//reads the waypoint text file into a vector of waypoint objects
	bool buildPath();

	//sends a waypoint to move_base
	bool go();
	int nextGoal = 0;

public:
	Follower(ros::NodeHandle &nh, MoveBaseClient &mbc, std::string fileName);
	void init(std::string fileName);
	void run();
	void setLoop(bool val);
	void setReturn(bool val);
	void printSettings();
};

#endif
