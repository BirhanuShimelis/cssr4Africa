#include "robotLocalization/robotLocalizationInterface.h"
#include <ros/ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "robotLocalization");
    ros::NodeHandle nh;

    RobotLocalization rl;

    ROS_INFO("Robot Localization Ready");
    double robot_initial_x = 0.0;
    double robot_initial_y = 0.0;
    double robot_initial_theta = 0.0;

    if (argc > 1)
    {
        robot_initial_x = std::stod(argv[1]);
        robot_initial_y = std::stod(argv[2]);
        robot_initial_theta = std::stod(argv[3]);
    }
    else
    {
        if (!nh.getParam("initial_robot_x", robot_initial_x))
        {
            ROS_WARN("Failed to get parameter 'initial_robot_x', using default value: %.2f", robot_initial_x);
        }
        if (!nh.getParam("initial_robot_y", robot_initial_y))
        {
            ROS_WARN("Failed to get parameter 'initial_robot_y', using default value: %.2f", robot_initial_y);
        }
        if (!nh.getParam("initial_robot_theta", robot_initial_theta))
        {
            ROS_WARN("Failed to get parameter 'initial_robot_theta', using default value: %.2f", robot_initial_theta);
        }
    }

    rl.setInitialValues(robot_initial_x, robot_initial_y, robot_initial_theta);

    while (ros::ok())
    {
        ROS_INFO_THROTTLE(10, "Robot Localization node running...");
        ros::spinOnce();
    }

    if (nh.hasParam("initial_robot_x"))
    {
        nh.deleteParam("initial_robot_x");
    }
    if (nh.hasParam("initial_robot_y"))
    {
        nh.deleteParam("initial_robot_y");
    }
    if (nh.hasParam("initial_robot_theta"))
    {
        nh.deleteParam("initial_robot_theta");
    }

    return 0;
}
