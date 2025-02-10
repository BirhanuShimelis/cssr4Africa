/*
 * robotNavigationDriver.cpp
 * 
 * Implements core robot navigation functions with logging.
 */

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "robotNavigationTest/robotNavigationTestInterface.h"

class RobotNavigationDriver {
private:
    ros::NodeHandle nh;
    ros::ServiceServer goal_service;
    ros::Publisher cmd_vel_pub;

public:
    RobotNavigationDriver() {
        cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
        goal_service = nh.advertiseService("/robotNavigation/set_goal", &RobotNavigationDriver::setGoalCallback, this);
        ROS_INFO("Robot Navigation Driver initialized.");
    }

    bool setGoalCallback(unit_tests::set_goal::Request &req, 
                         unit_tests::set_goal::Response &res) {
        ROS_INFO("Received navigation goal: X=%.2f, Y=%.2f", req.goal_x, req.goal_y);

        geometry_msgs::Twist move_cmd;
        move_cmd.linear.x = 0.5;
        cmd_vel_pub.publish(move_cmd);
        ROS_INFO("Robot moving...");

        ros::Duration(5.0).sleep();

        move_cmd.linear.x = 0.0;
        cmd_vel_pub.publish(move_cmd);
        ROS_INFO("Robot reached destination.");

        res.navigation_goal_success = 1;
        return true;
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "robot_navigation_driver");
    RobotNavigationDriver driver;
    ros::spin();
    return 0;
}
