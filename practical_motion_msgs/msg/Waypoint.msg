int16 id                     # the id number
string frame_id              # The frame of reference the goal will be issued in. ie: map, odom, base_link etc
float32 x                    # The x coordinate of the goal
float32 y                    # the y coordinate of the goal
float32 theta                # The desired final heading of the goal as a simple euler angle using radians. -3.14 to +3.14
string description           # A descriptive name of the goal. 
int16 lingerTime             # how long to linger at a waypoint before moving on to the next. In seconds. (only works if handling node enable the waiting behavior)