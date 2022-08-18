#include "ros/ros.h"
#include <ros/package.h>
#include <tf/transform_listener.h>
#include "practical_waypoint_follow/waypoint.h"
#include <iostream>
#include <iomanip>
#include <fstream>

using namespace std;

geometry_msgs::PoseStamped lastClick;
bool gotNewClick = false;

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
	cin.ignore(1, '\n');
	system("clear");
	return x;
}

// get float euler from quaternion
float toEuler(const tf::Quaternion &q)
{
	double roll, pitch, heading;

	tf::Matrix3x3 m(q);
	m.getRPY(roll, pitch, heading);
	return heading;
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
				<< fixed
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
	cout << "saving and exiting to: " << path << endl;
}

// copy and store the message data
void updateClick(const geometry_msgs::PoseStamped &goal)
{
	gotNewClick = true;
	lastClick.header.frame_id = goal.header.frame_id;
	lastClick.pose.position.x = goal.pose.position.x;
	lastClick.pose.position.y = goal.pose.position.y;
	lastClick.pose.orientation.x = goal.pose.orientation.x;
	lastClick.pose.orientation.y = goal.pose.orientation.y;
	lastClick.pose.orientation.z = goal.pose.orientation.z;
	lastClick.pose.orientation.w = goal.pose.orientation.w;
}

int main(int argc, char **argv)
{
	// ros init stuff
	ros::init(argc, argv, "waypoint_saver_rviz");
	ros::NodeHandle node("~");

	// the uninitialized subscriber for rviz clicked goals
	ros::Subscriber subClicks = node.subscribe("/move_base_simple/goal", 1, updateClick);

	// The container for the waypoints
	vector<Waypoint> waypoints;

	// this node automatically assigns incrementing id number to waypoint. User can manually edit in waypoint text file if desired.
	int id = 0;

	system("clear");
	cout << "This node will record goals clicked in rviz as waypoints. \n"
		 << "Ensure the fixed frame selected in rviz global options is the one you want ro record (probably map) \n"
		 << "Don't forget to disable motors if you don't want robot to move.\n"
		 << "....................................................................................................\n"
		 << endl
		 << endl;

	// get user inputs
	string fileName = getInput("Enter filename to save to (must end with .txt). Do not include path.  : ");
	system("clear");

	string ipt = "";

	ros::Rate loop_rate(1);
	while (ipt == "" && ros::ok())
	{
		cout << "(Click a 2D Nav Goal in RVIZ before answering if you intend to save more)" << endl;
		ipt = getInput("Enter a space to save most recent clicked goal as a waypoint. Q to save and quit: ");

		// quit and save
		if (ipt == "Q" || ipt == "q")
		{
			if (waypoints.size() > 0)
			{
				save(waypoints, fileName);
				for (int i = 0; i < waypoints.size(); i++)
				{
					cout << waypoints[i].toString() << endl;
				}
			}
			else
				cout << "Nothing to save" << endl;

			return 0;
		}
		// save current position as waypoint
		if (ipt == " ")
		{
			ros::spinOnce();

			if (!gotNewClick)
			{
				cout << "No new click to save" << endl;
				ipt = "";
				continue;
			}

			tf::Quaternion q(lastClick.pose.orientation.x, lastClick.pose.orientation.y,
							 lastClick.pose.orientation.z, lastClick.pose.orientation.w);
			float heading = toEuler(q);

			system("clear");
			cout << "Confirm that you want to save "
				 << "\n x, y : " << lastClick.pose.position.x << ", " << lastClick.pose.position.y << "   heading : "
				 << heading << "   frame_id: " << lastClick.header.frame_id << endl;

			ipt = getInput("Y to save this waypoint or any other character to cancel: ");
			if (ipt == "Y" || ipt == "y")
			{
				string description = getInput("Enter description: ");
				int lingerTime = (int)getNum("How many seconds should the robot linger at this waypoint? (int required) ");

				// construct waypoint with Waypoint(int id, std::string frame, float x, float y, float theta, std::string description);
				Waypoint temp(id++, lastClick.header.frame_id, lastClick.pose.position.x, lastClick.pose.position.y,
							  heading, description, lingerTime);

				// add to waypoint vector
				waypoints.push_back(temp);
			}
			gotNewClick = false;
		}

		ipt = "";
		loop_rate.sleep();
	}

	return 0;
}
