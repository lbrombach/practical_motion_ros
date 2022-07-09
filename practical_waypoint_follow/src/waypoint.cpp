#include "ros/ros.h"
#include "practical_waypoint_follow/waypoint.h"
#include <tf2/LinearMath/Quaternion.h>

Waypoint::Waypoint()
{
	id = -1;
	x = 0;
	y = 0;
	theta = 0;
	frame_id = "map";
	description = "";
	lingerTime = 0;
};

Waypoint::Waypoint(const Waypoint &wpt)
{
	this->id = wpt.id;
	this->x = wpt.x;
	this->y = wpt.y;
	this->theta = wpt.theta;
	this->frame_id = wpt.frame_id;
	this->description = wpt.description;
	this->lingerTime = wpt.lingerTime;
};

Waypoint::Waypoint(int id, std::string frame, float x, float y, float theta, std::string description, int lingerTime)
{
	this->id = id;
	this->frame_id = frame;
	this->x = x;
	this->y = y;
	this->theta = theta;
	this->description = description;
	this->lingerTime = lingerTime;
};

// setter for theta
float Waypoint::setTheta(float theta)
{
	this->theta = theta;
}

move_base_msgs::MoveBaseGoal Waypoint::toGoal()
{
	// create the goal message
	move_base_msgs::MoveBaseGoal goal;

	// copy the directly copy-able elements
	goal.target_pose.header.frame_id = frame_id;
	goal.target_pose.header.stamp = ros::Time::now();
	goal.target_pose.pose.position.x = x;
	goal.target_pose.pose.position.y = y;

	// convert the euler heading angle to orientation in quaternions as required by move_base
	tf2::Quaternion q;
	q.setRPY(0, 0, theta);
	goal.target_pose.pose.orientation.x = q.x();
	goal.target_pose.pose.orientation.y = q.y();
	goal.target_pose.pose.orientation.z = q.z();
	goal.target_pose.pose.orientation.w = q.w();

	return goal;
}

practical_motion_msgs::Waypoint Waypoint::toMsg()
{
	// create a message and copy the elements
	practical_motion_msgs::Waypoint msg;
	msg.id = id;
	msg.frame_id = frame_id;
	msg.x = x;
	msg.y = y;
	msg.theta = theta;
	msg.description = description;
	msg.lingerTime = lingerTime;
	return msg;
}

std::string Waypoint::toString()
{
	return "ID: " + std::to_string(id) + "  point: " + std::to_string(x) + ", " + std::to_string(y) + "  theta: " + std::to_string(theta) + "  description: " + description + "  lingerTime: " + std::to_string(lingerTime);
};
