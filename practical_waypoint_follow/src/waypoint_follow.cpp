#include "ros/ros.h"
#include "practical_waypoint_follow/waypoint_follow.h"
#include <ros/package.h>
#include <iostream>
#include <iomanip>
#include <fstream>

Follower::Follower()
{
}

Follower::Follower(ros::NodeHandle &nh, MoveBaseClient &mbc, std::string fileName)
{
	node = &nh;
	mbcPtr = &mbc;
	init(fileName);
}

void Follower::init(std::string fileName)
{
	// set from param.. if true keep looping when complete
	_loop = false;
	_return = false;

	// get the path to this package
	std::string path = ros::package::getPath("practical_waypoint_follow");
	path += "/waypoints/";
	path += fileName;
	ROS_INFO(path.c_str());

	// get file name and path from parameter. WIll override command line argument
	node->param("file_path", filePath, path);

	// build the list of waypoints
	if (!buildPath())
	{
		ROS_FATAL("UNABLE TO BUILD WAYPOINT LIST FROM FILE - EXITING");
		ros::shutdown();
	}
}

// reads the waypoint file into a vector of waypoint objects
bool Follower::buildPath()
{
	std::ifstream inFile;
	std::cout << "opening file: " << filePath << std::endl;
	inFile.open(filePath.c_str());
	if (inFile.fail())
	{
		ROS_INFO("UNABLE TO OPEN FILE");
		return false;
	}

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

		inFile >> std::setprecision(2) >> x;
		inFile >> std::setprecision(2) >> y;
		inFile >> theta;
		inFile >> description;
		inFile >> lingerTime;
		inFile.ignore(2, '\n');
		inFile.ignore(2, '\n');
		Waypoint temp(id, frame, x, y, theta, description, lingerTime);
		waypoints.push_back(temp);
		std::cout << std::setprecision(2) << temp.toString() << std::endl;

		if (inFile.eof())
		{
			std::cout << "end of file" << std::endl;
			break;
		}
	}

	inFile.close();

	for (auto i : waypoints)
		ROS_INFO(i.toString().c_str());

	return true;
}

// iterate through waypoints once if loop == false else forever
void Follower::run()
{

	while (ros::ok() && nextGoal < waypoints.size())
	{

		if (go())
		{
			ROS_INFO("Goal Reached. Next Goal: %d", nextGoal);
			ros::Duration(waypoints[nextGoal].getLingerTime()).sleep();
		}
		else
			ROS_INFO("Goal NOT Reached");

		// increment the index of the next goal and wrap back to beginning if at end of list and _loop param set to true
		if (++nextGoal >= waypoints.size() && _loop == true)
		{
			nextGoal = 0;
		}
	}

	// After completing above loop, issue a return to waypoint[0] if _return param set to true
	if (_return)
	{
		ROS_INFO("Patrol complete. Returning to start");
		// TODO get this from pose data instead of fixed first waypoint
		nextGoal = 0;
		go();
	}
}

// sends goals to move_base and waits for result
bool Follower::go()
{
	ROS_INFO("Sending goal: %s", waypoints[nextGoal].toString().c_str());
	mbcPtr->sendGoal(waypoints[nextGoal].toGoal());

	mbcPtr->waitForResult();
	if (mbcPtr->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
	{
		if (1)
		{
			return true;
		}
		return false;
	}
}

void Follower::setLoop(bool val)
{
	_loop = val;
}
void Follower::setReturn(bool val)
{
	_return = val;
}

void Follower::printSettings()
{
	if (_loop)
		ROS_INFO("Initializing waypoint follower with loop: true - ignoring return parameter");
	else
	{
		ROS_INFO("Initializing waypoint follower with loop  : false");
		if (_return)
			ROS_INFO("Initializing waypoint follower with return: true");
		else
			ROS_INFO("Initializing waypoint follower with return: false");
	}

	ROS_INFO("Initializing waypoint follower with file path: %s", filePath.c_str());
}
