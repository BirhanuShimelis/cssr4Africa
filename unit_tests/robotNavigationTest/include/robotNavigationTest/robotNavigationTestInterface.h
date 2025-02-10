/* animateBehaviourTestInterface.h */

#ifndef ROBOT_NAVIGATION_TEST_INTERFACE_H
#define ROBOT_NAVIGATION_TEST_INTERFACE_H

#define ROS
 
#ifndef ROS
   #include <conio.h>
#else
   #include <ros/ros.h>
   #include <ros/package.h>
   #include <sys/select.h>
   #include <termios.h>
   //#include <stropts.h>
   #include <sys/ioctl.h>
#endif


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <cmath>  
#include <ctype.h>
#include <iostream>
#include <ros/ros.h>
#include <ros/package.h>
#include <turtlesim/Pose.h>
#include <turtlesim/TeleportAbsolute.h> // for turtle1/teleport_absolute service
#include <turtlesim/SetPen.h>           // for turtle1/set_pen service
#include <std_srvs/Empty.h>             // for reset and clear services
#include <geometry_msgs/Twist.h>        // For geometry_msgs::Twist
#include <nav_msgs/Odometry.h>          // For nav_msgs::Odometry
#include <iomanip>                      // for std::setprecision and std::fixed

#include "unit_tests/set_goal.h"  // Include for the set_goal service
#include "geometry_msgs/Pose2D.h"       // For geometry_msgs::Pose2D
#include <boost/algorithm/string.hpp>
#include <std_msgs/Float64.h>  // Include for publishing Float64 messages
#include <fstream>
#include <sstream>

#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <naoqi_bridge_msgs/JointAnglesWithSpeed.h>

#include <thread>
#include <atomic>
#include <queue>
#include <vector>
#include <climits>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <queue>
#include <vector>
#include <iostream>
#include <cmath>
#include <climits>
#include <string>
#include <opencv2/opencv.hpp>

#define ROS_PACKAGE_NAME    "unit_tests"              // Change to cssr_system for integration


#endif // EXAMPLE_COMPONENT_TEST_INTERFACE_H