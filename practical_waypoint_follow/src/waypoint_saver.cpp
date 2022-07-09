#include "ros/ros.h"
#include <ros/package.h>
#include <tf/transform_listener.h>
#include "practical_waypoint_follow/waypoint.h"
#include <iostream>
#include <iomanip>
#include <fstream>

using namespace std;

// get user input string
string getInput(string msg)
{
	string input = "";
	bool valid = false;
	while (!valid)
	{
		if (cin.fail())
		{
			cin.clear();
			cin.ignore(50, '\n');
		}
		cout << msg;
		getline(cin, input);
		valid = input.compare("");
		if (!valid)
			cout << "invalid entry" << endl;
	}
	return input;
}

// Get float from user input
float getNum(string msg)
{
	bool gotIt = false;
	int x;
	while (gotIt == false)
	{
		cout << msg;
		cin >> x;
		if (cin.fail())
		{
			cout << "Invalid Data Type, try again" << endl;
			cin.clear();
			cin.ignore(50, '\n');
		}
		else
			gotIt = true;
	}
	system("clear");
	return x;
}

// Writes list to file
void save(vector<Waypoint> &waypoints, string fileName)
{
	string path = ros::package::getPath("practical_waypoint_follow");
	path += "/waypoints/";
	path += fileName;
	ofstream outFile;
	outFile.open(path.c_str());

	for (int i = 0; i < waypoints.size(); i++)
	{
		outFile << "#####" << endl
				<< setprecision(2)
				<< waypoints[i].getId() << endl
				<< waypoints[i].getFrame() << endl
				<< waypoints[i].getX() << endl
				<< waypoints[i].getY() << endl
				<< waypoints[i].getTheta() << endl
				<< waypoints[i].getDescription() << endl
				<< waypoints[i].getLingerTime() << endl;
	}
	outFile.close();
	cout << "saving and exiting" << endl;
}

int main(int argc, char **argv)
{
	// ros init stuff
	ros::init(argc, argv, "waypoint_saver");
	ros::NodeHandle node("~");

	// get user inputs
	string fileName = getInput("Enter filename to save to. Do not include path.  : ");
	string baseFrame = getInput("Enter the robots base frame id (ie: base_frame, base_link, etc) : ");
	string frame = getInput("Enter frame of waypoints (ie: map, utm, odom, base_link): ");

	// The container for hte waypoints
	vector<Waypoint> waypoints;

	// this node automatically assigns incrementing id number to waypoint. User can manually edit in waypoint text file if desired.
	int id = 0;

	// Node uses transform data to get current position
	static tf::TransformListener listener;
	tf::StampedTransform tf;

	// wait for transform data to become available
	ros::Rate loop_rate(1);
	while (!listener.canTransform(frame, baseFrame, ros::Time(0), NULL))
	{
		cout << "Waiting for " << frame << " to " << baseFrame << " tf to become available" << endl;
		loop_rate.sleep();
	}

	string ipt = "";
	while (ipt == "" && ros::ok())
	{
		ipt = getInput("Press enter a space to save current position or enter a frame_id to manually enter a waypoint. Q to save and quit: ");

		// quit and save
		if (ipt == "Q" || ipt == "q")
		{
			save(waypoints, fileName);
			for (int i = 0; i < waypoints.size(); i++)
			{
				cout << waypoints[i].toString() << endl;
			}

			return 0;
		}
		// save current position as waypoint
		if (ipt == " ")
		{

			string description = getInput("Enter description: ");
			int lingerTime = (int)getNum("How many seconds should the robot linger at this waypoint? (int required) ");

			// get current position data
			listener.lookupTransform(frame, "base_link", ros::Time(0), tf);

			float x, y;
			double roll, pitch, heading;

			// copy x and y
			x = tf.getOrigin().x();
			y = tf.getOrigin().y();

			// convert quaternion orientation data provided by tf to human readable euler angle heading in radians
			tf::Quaternion q(tf.getRotation().x(), tf.getRotation().y(), tf.getRotation().z(), tf.getRotation().w());
			tf::Matrix3x3 m(q);
			m.getRPY(roll, pitch, heading);

			// construct waypoint with Waypoint(int id, std::string frame, float x, float y, float theta, std::string description);
			Waypoint temp(id++, frame, x, y, heading, description, lingerTime);

			// add to waypoint vector
			waypoints.push_back(temp);
		}
		// manually enter a waypoint
		else
		{
			string frame = ipt;
			float x = getNum("Enter x: ");
			float y = getNum("Enter y: ");
			float heading = getNum("Enter desired heading at this waypoint: ");
			string description = getInput("Enter description: ");
			int lingerTime = (int)getNum("How many seconds should the robot linger at this waypoint? (int required) ");

			Waypoint temp(id++, frame, x, y, heading, description, lingerTime);
			waypoints.push_back(temp);
		}

		ipt = "";
	}

	return 0;
}
