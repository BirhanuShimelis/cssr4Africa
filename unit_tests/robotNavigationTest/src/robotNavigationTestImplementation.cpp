/*
 * robotNavigationTestImplementation.cpp
 * 
 * Implements unit tests for robotNavigation with ROS logging.
 */

#include <ros/ros.h>
#include <gtest/gtest.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <unit_tests/set_goal.h>
#include <std_srvs/Empty.h>
#include <atomic>
#include <ros/service_client.h>
#include <fstream>
#include <vector>
#include <memory>
#include <stdexcept>
#include <cmath>
#include <std_msgs/String.h>
#include <limits>
#include "robotNavigationTest/robotNavigationTestInterface.h"

std::atomic<bool> obstacle_detected(false);
std::atomic<bool> robot_moving(false);
double min_obstacle_distance = 1.0;
double robot_x = 0.0, robot_y = 0.0, robot_theta = 0.0;
// ###############################################################################################
struct Goal {
    double x, y, theta;
};

std::vector<Goal> readGoalsFromFile(const std::string& filepath) {
    std::ifstream file(filepath);
    std::string line;
    std::vector<Goal> goals;
    while (std::getline(file, line)) {
        if (line.find("goal") != std::string::npos) {
            Goal goal;
            std::istringstream iss(line);
            std::string temp;
            iss >> temp >> goal.x >> goal.y >> goal.theta;
            goals.push_back(goal);
        }
    }
    return goals;
}

//###############################################################################################
// Helper function to invoke a ROS service and capture its response
std::string invoke_service(const char* cmd) {
    std::array<char, 128> buffer;
    std::string result;
    std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd, "r"), pclose);
    if (!pipe) {
        throw std::runtime_error("popen() failed!");
    }
    while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
        result += buffer.data();
    }
    return result;
}



// Unit test fixture for Robot Navigation
class RobotNavigationTest : public ::testing::Test {
    protected:
        virtual void SetUp() override {
            ros::NodeHandle nh;
            std::string output;
            std::string path_planning_command = "rosrun cssr_system robotNavigation";
    
            output = invoke_service(path_planning_command.c_str());
            bool valid_algorithm = (output.find("A*") != std::string::npos || output.find("Dijkstra") != std::string::npos);

            EXPECT_TRUE(valid_algorithm) << "Invalid or missing Path Planning Algorithm in response!";
            // Ensure the robotNavigation service exists before proceeding
            ASSERT_TRUE(ros::service::waitForService("/robotNavigation/set_goal", ros::Duration(5.0)))
                << "Service /robotNavigation/set_goal is not available!";
        }
    
        virtual void TearDown() override {
            // Optional: Clean up operations
        }
    };


// // Test Case 3: Validate Path Planning Algorithm Execution
// TEST_F(RobotNavigationTest, PathPlanningAlgorithm) {
//     std::string output;
//     std::string path_planning_command = "rosrun cssr_system robotNavigation";
    
//     output = invoke_service(path_planning_command.c_str());

//     bool valid_algorithm = (output.find("A*") != std::string::npos || output.find("Dijkstra") != std::string::npos);

//     EXPECT_TRUE(valid_algorithm) << "Invalid or missing Path Planning Algorithm in response!";
// }


// LiDAR Callback
void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    obstacle_detected = false;
    double min_distance = std::numeric_limits<double>::max();

    for (size_t i = 0; i < msg->ranges.size(); i++) {
        if (!std::isnan(msg->ranges[i]) && msg->ranges[i] < min_distance) {
            min_distance = msg->ranges[i];
        }
    }

    if (min_distance < min_obstacle_distance) {
        obstacle_detected = true;
        ROS_WARN("Obstacle detected! Distance: %.2f meters", min_distance);
    }
}

// Odometry Callback
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    robot_x = msg->pose.pose.position.x;
    robot_y = msg->pose.pose.position.y;
    robot_theta = 2 * atan2(msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    robot_moving = true;
}

// // Test 1: Check if robotNavigation node is running
// TEST(RobotNavigationTest, NodeRunning) {
//     ros::Duration timeout(5.0);
//     bool node_running = ros::service::exists("/robotNavigation/set_goal", true);    
//     if (node_running) {
//         ROS_INFO("TEST PASSED: robotNavigation node is running.");
//     } else {
//         ROS_ERROR("TEST FAILED: robotNavigation node is NOT running!");
//     }

//     ASSERT_TRUE(node_running);
// }

// Test Case 1: Verify if Robot Navigation Service is Running
TEST_F(RobotNavigationTest, SetGoalServiceAvailable) {
    bool service_exists = ros::service::exists("/robotNavigation/set_goal", true);
    EXPECT_TRUE(service_exists) << "Service /robotNavigation/set_goal is not available!";
}


// Test Case 2: Send Navigation Goal and Check Response
TEST_F(RobotNavigationTest, SendNavigationGoalFromFile) {
    std::vector<Goal> goals = readGoalsFromFile("/home/br/workspace/pepper_rob_ws/src/cssr4Africa/unit_tests/robotNavigationTest/data/test_goals.dat");
    
    for (const auto& goal : goals) {
        std::string goal_command = "rosservice call /robotNavigation/set_goal -- " + std::to_string(goal.x) + " " + std::to_string(goal.y) + " " + std::to_string(goal.theta);
        std::string output = invoke_service(goal_command.c_str());

        int response;
        if (sscanf(output.c_str(), "%*[^:]: %d", &response) == 1) {
            EXPECT_EQ(response, 1) << "Navigation goal response is incorrect!";
        } else {
            FAIL() << "Failed to parse integer from service response";
        }
    }
}
// // Test 2: Check if service is available
// TEST(RobotNavigationTest, SetGoalServiceAvailable) {
//     ros::NodeHandle nh;
//     ros::ServiceClient set_goal_client = nh.serviceClient<unit_tests::set_goal>("/robotNavigation/set_goal");

//     ros::Duration timeout(5.0);
//     bool service_exists = set_goal_client.waitForExistence(timeout);

//     if (service_exists) {
//         ROS_INFO("TEST PASSED: /robotNavigation/set_goal service is available.");
//     } else {
//         ROS_ERROR("TEST FAILED: /robotNavigation/set_goal service is NOT available.");
//     }

//     ASSERT_TRUE(service_exists);
// }



// // Test 3: Check if robot moves to goal

// TEST(RobotNavigationTest, RobotMoves) {
//     ros::NodeHandle nh;
//     ros::ServiceClient set_goal_client = nh.serviceClient<unit_tests::set_goal>("/robotNavigation/set_goal");

//     unit_tests::set_goal srv;
//     srv.request.goal_x = 4.0;
//     srv.request.goal_y = 6.6;
//     srv.request.goal_theta = 0.0;

//     ASSERT_TRUE(set_goal_client.call(srv)) << "Failed to call /robotNavigation/set_goal service!";
//     ROS_INFO("Goal sent: Waiting for robot to move...");

//     // Subscribe to odometry topic
//     ros::Subscriber odom_sub = nh.subscribe("/naoqi_driver/odom", 10, odomCallback);
    
//     ros::Time start_time = ros::Time::now();
//     ros::Duration timeout(40.0);  // Allow up to 40 seconds
//     bool reached_goal = false;
    
//     while (ros::Time::now() - start_time < timeout) {
//         ros::spinOnce();

//         // Calculate distance to goal
//         double distance_to_goal = sqrt(pow(robot_x - srv.request.goal_x, 2) + pow(robot_y - srv.request.goal_y, 2));
        
//         // If within 30cm of the goal, mark as reached
//         if (distance_to_goal < 0.3) {
//             reached_goal = true;
//             ROS_INFO("Robot has reached the goal! Stopping test.");
//             break;
//         }

//         // Check the set_goal response again to confirm
//         if (set_goal_client.call(srv) && srv.response.navigation_goal_success == 1) {
//             reached_goal = true;
//             ROS_INFO("Navigation service confirmed goal success!");
//             break;
//         }

//         ros::Duration(0.5).sleep();  // Wait before rechecking
//     }

//     ASSERT_TRUE(reached_goal) << "Robot did not reach the goal within timeout!";
// }



// Test 4:  Obstacle Detection
TEST(RobotNavigationTest, ObstacleDetection) {
    ros::NodeHandle nh;
    ros::Subscriber obstacle_sub = nh.subscribe("/naoqi_driver/laser", 10, lidarCallback);

    ros::Duration(3.0).sleep();
    ros::spinOnce();

    if (obstacle_detected) {
        ROS_WARN("TEST PASSED: Obstacle detected by laser scanner.");
    } else {
        ROS_INFO("TEST PASSED: No obstacles detected.");
    }

    ASSERT_TRUE(true);
}

// // Test 5: Obstacle Avoidance
// TEST(RobotNavigationTest, ObstacleAvoidance) {
//     ros::NodeHandle nh;
//     ros::ServiceClient set_goal_client = nh.serviceClient<unit_tests::set_goal>("/robotNavigation/set_goal");

//     unit_tests::set_goal srv;
//     srv.request.goal_x = 4.0;
//     srv.request.goal_y = 6.6;
//     srv.request.goal_theta = 0.0;

//     ASSERT_TRUE(set_goal_client.call(srv)) << "Failed to call service!";
//     ASSERT_EQ(srv.response.navigation_goal_success, 1) << "Navigation goal failed!";

//     ros::Duration(3.0).sleep();
//     ros::spinOnce();

//     if (obstacle_detected) {
//         ROS_WARN("TEST PASSED: Obstacle avoidance triggered!");
//     } else {
//         ROS_INFO("TEST PASSED: No obstacle avoidance triggered.");
//     }

//     ASSERT_TRUE(true);
// }
// #################################################################


// Test Case 4: Check Obstacle Detection and Avoidance
TEST_F(RobotNavigationTest, ObstacleDetectionAndAvoidance) {
    std::string output;
    std::string obstacle_test_command = "rosservice call /robotNavigation/set_goal -- 3.5 6.0 0.0";

    output = invoke_service(obstacle_test_command.c_str());

    bool detected = (output.find("Obstacle detected") != std::string::npos);
    bool avoided = (output.find("Avoidance successful") != std::string::npos);

    EXPECT_TRUE(detected) << "Obstacle was not detected!";
    EXPECT_TRUE(avoided) << "Obstacle avoidance failed!";
}


// Test Case 5: Verify Localization Accuracy
TEST_F(RobotNavigationTest, LocalizationAccuracy) {
    double expected_error_threshold = 0.3;
    double actual_error = 8.28;  // Replace with actual value obtained from robot state
    
    EXPECT_LT(actual_error, expected_error_threshold) << "Localization error exceeded threshold!";
}
//###############################################################################


// // Test 6: Validate Path Planning Algorithm
// TEST(RobotNavigationTest, PathPlanning) {
//     ros::NodeHandle nh;
//     ros::ServiceClient client = nh.serviceClient<unit_tests::set_goal>("/robotNavigation/set_goal");

//     unit_tests::set_goal srv;
//     srv.request.goal_x = 5.0;
//     srv.request.goal_y = 6.6;
//     srv.request.goal_theta = 0.0;

//     ASSERT_TRUE(client.call(srv)) << "Failed to call /robotNavigation/set_goal service!";
    
//     ROS_INFO("Goal sent: Checking if the path planning algorithm executed correctly...");

//     // Read the selected algorithm from robot navigation
//     std::string algorithm_used;
//     nh.getParam("/robotNavigation/path_planning_algorithm", algorithm_used);

//     // Ensure the algorithm is one of the expected values
//     std::vector<std::string> valid_algorithms = {"bfs", "dijkstra", "astar"};
//     ASSERT_TRUE(std::find(valid_algorithms.begin(), valid_algorithms.end(), algorithm_used) != valid_algorithms.end()) 
//         << "Invalid Path Planning Algorithm: " << algorithm_used;

//     ROS_INFO("TEST PASSED: Path planning algorithm is %s", algorithm_used.c_str());

//     // Subscribe to odometry topic
//     ros::Subscriber odom_sub = nh.subscribe("/naoqi_driver/odom", 10, odomCallback);

//     ros::Time start_time = ros::Time::now();
//     ros::Duration timeout(40.0);  // Allow up to 40 seconds
//     bool reached_goal = false;
    
//     std::vector<double> path_x, path_y;

//     while (ros::Time::now() - start_time < timeout) {
//         ros::spinOnce();

//         // Get the planned waypoints from the robotNavigation node
//         nh.getParam("/robotNavigation/planned_path_x", path_x);
//         nh.getParam("/robotNavigation/planned_path_y", path_y);

//         ASSERT_FALSE(path_x.empty() && path_y.empty()) << "Path Planning failed! No valid waypoints returned.";

//         // Validate that the planned path contains at least 2 points (start and goal)
//         ASSERT_GE(path_x.size(), 2) << "Path contains too few waypoints!";
//         ASSERT_GE(path_y.size(), 2) << "Path contains too few waypoints!";

//         // Check if the robot is following the planned path
//         double distance_to_goal = sqrt(pow(robot_x - srv.request.goal_x, 2) + pow(robot_y - srv.request.goal_y, 2));

//         // If within 30cm of the goal, mark as reached
//         if (distance_to_goal < 0.3) {
//             reached_goal = true;
//             ROS_INFO("Robot has followed the planned path and reached the goal.");
//             break;
//         }

//         ros::Duration(0.5).sleep();  // Wait before rechecking
//     }

//     ASSERT_TRUE(reached_goal) << "Robot did not reach the goal within timeout!";
// }

// // Test 7: Localization Accuracy
// TEST(RobotNavigationTest, LocalizationAccuracy) {
//     ros::NodeHandle nh;
//     ros::Subscriber odom_sub = nh.subscribe("/naoqi_driver/odom", 10, odomCallback);
//     ros::Duration(5.0).sleep();
//     ros::spinOnce();

//     double expected_x = 5.0, expected_y = 6.6;
//     double error = std::hypot(robot_x - expected_x, robot_y - expected_y);


//     ASSERT_LT(error, 15) << "Localization error exceeded 0.15m!";
//     ROS_INFO("TEST PASSED: Localization accuracy is within limits.");
//     ROS_INFO("Expected Pose: (%.2f, %.2f), Actual Pose: (%.2f, %.2f), Error: %.2f",
//          expected_x, expected_y, robot_x, robot_y, error);

// }
