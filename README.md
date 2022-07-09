# practical_motion_ros
A collection of ROS packages that handle motion matters such as following, wandering, etc.
<br><br>

## Installation
cd /catkin_ws/src
git clone https://github.com/lbrombach/practical_motion_ros
cd ..
catkin_make

## Usage
Please checkout the following individual package readme docs and check the [Practical Robotics YouTube channel](https://www.youtube.com/practicalrobotics) for tutorials. 

## Package: practical_motion_msgs
A package for defining custom messages and service files used by other nodes in this package. [practical_motions_msgs readme](practical_motion_msgs/readme.md)
<br><br>

## Package: practical_waypoint_follow
A package with several nodes to facilitate building a list of waypoints, serve waypoints to client nodes, and following waypoints directly with move_base action calls. See the [practical_waypoint_follow readme](practical_waypoint_follow/readme.md).
<br><br>

## Contributing, bug reports, etc:
If you find my tutorials and projects helpful, consider supporting the development of this and others at [BuyMeACoffee](https://www.buymeacoffee.com/practicalrobot).
Please use the issues and bug reporting system for bugs and feature requests. With many projects, a job, a family, I can't promise to get to feature requests very quickly, but am definitely listening for feedback to make improvement and will prioritize bugs. I am open to pull requests if you'd like to contribute. I can be contacted by email at lbrombach2@gmail.com. 
