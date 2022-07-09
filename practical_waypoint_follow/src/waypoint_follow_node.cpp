#include "ros/ros.h"
#include <iostream>
#include "practical_waypoint_follow/waypoint_follow.h"

using namespace std;

const string DEFAULT_FILENAME = "waypoints.txt";

int main(int argc, char **argv)
{
  // init with ros master and get node handle
  ros::init(argc, argv, "waypoint_follower");
  ros::NodeHandle node("~");

  // check the loop parameter. if true keep looping when complete
  bool loop = false;
  node.getParam("loop", loop);

  // if true return to start when done. Ignored if continuous or loop are true
  bool ret = false;
  node.getParam("return", ret);

  // check for command line filename argument.
  string fileName = DEFAULT_FILENAME;
  if (argc > 2)
  {
    ROS_FATAL("Invalid number of arguments - program only accepts a file name");
    return 1;
  }
  if (argc == 2)
  {
    fileName = argv[1];
    fileName = "/" + fileName;
  }

  // create move base client
  MoveBaseClient mbc("move_base", true);
  while (!mbc.waitForServer(ros::Duration(5.0)))
  {
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  // create the follower and start it running
  Follower follower(node, mbc, fileName);
  follower.setLoop(loop);
  follower.setReturn(ret);
  follower.printSettings();
  follower.run();

  return 0;
}
