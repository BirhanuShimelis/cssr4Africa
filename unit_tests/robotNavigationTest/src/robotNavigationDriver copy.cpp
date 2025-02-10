#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <cssr_system/set_goal.h>
#include <boost/algorithm/string.hpp>
#include <gtest/gtest.h>

#define ROS
#define SOFTWARE_VERSION "v1.0"
#define TEST_TIMEOUT 30  // seconds

using namespace std;

// Service client for navigation
ros::ServiceClient navigation_client;

// Test parameters
struct TestConfig {
    double goal_x;
    double goal_y;
    double goal_theta;
    double position_tolerance;
    double angle_tolerance;
};

// Global variables
TestConfig test_config;
string node_name;

void load_test_configuration() {
    string config_path = ros::package::getPath("cssr_system") + 
                        "/unit_tests/robotNavigationTest/config/robotNavigationTestConfiguration.ini";
    
    ifstream config_file(config_path);
    if (!config_file.is_open()) {
        ROS_ERROR("Failed to open test configuration file");
        return;
    }

    string line;
    while (getline(config_file, line)) {
        istringstream iss(line);
        string key, value;
        iss >> key >> value;
        
        if (key == "goal_x") test_config.goal_x = stod(value);
        else if (key == "goal_y") test_config.goal_y = stod(value);
        else if (key == "goal_theta") test_config.goal_theta = stod(value);
        else if (key == "position_tolerance") test_config.position_tolerance = stod(value);
        else if (key == "angle_tolerance") test_config.angle_tolerance = stod(value);
    }
}

TEST(RobotNavigationTest, GoalReachingTest) {
    cssr_system::set_goal srv;
    srv.request.goal_x = test_config.goal_x;
    srv.request.goal_y = test_config.goal_y;
    srv.request.goal_theta = test_config.goal_theta;

    bool service_call_success = navigation_client.call(srv);
    EXPECT_TRUE(service_call_success) << "Service call failed";

    if (service_call_success) {
        bool goal_reached = false;
        auto start_time = ros::Time::now();
        
        while ((ros::Time::now() - start_time).toSec() < TEST_TIMEOUT) {
            if (srv.response.navigation_goal_success) {
                goal_reached = true;
                break;
            }
            ros::spinOnce();
            ros::Duration(0.5).sleep();
        }
        
        EXPECT_TRUE(goal_reached) << "Failed to reach goal within timeout";
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "robotNavigationTest");
    ros::NodeHandle nh;
    
    node_name = ros::this_node::getName();
    load_test_configuration();

    // Initialize service client
    navigation_client = nh.serviceClient<cssr_system::set_goal>("/robotNavigation/set_goal");
    
    // Wait for service to be available
    bool service_available = false;
    for (int i = 0; i < 10; ++i) {
        if (navigation_client.exists()) {
            service_available = true;
            break;
        }
        ROS_WARN("Waiting for navigation service...");
        ros::Duration(1).sleep();
    }

    if (!service_available) {
        ROS_ERROR("Navigation service not available");
        return EXIT_FAILURE;
    }

    // Run tests
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}




#include <gtest/gtest.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <ros/service_client.h>
#include <ros/master.h>
#include <fstream>
#include <ctime>
#include <cstdio>
#include <string>
#include <stdexcept>
#include <cstdlib>
#include <boost/algorithm/string.hpp>
#include <vector>
#include <cmath>   
#include <memory>
#include <array>
#include <signal.h>
#include <std_msgs/String.h>


/*  Function to invoke the gesture execution service and return the response from the service
 *  
 * @param:
 *     cmd: the service command to run
 * 
 * @return
 *    result: the response from the service
 */
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

// Execute iconic gesture by calling the gesture execution service with a system call and save the response from the service
std::string output; 
string service_call = "rosservice call /gestureExecution/perform_gesture -- iconic " + gesture_ID + " 3000 45 -1.2 3 0.82";
output = invoke_service(service_call.c_str());

bool test_result = false;                   // Status of the test
// Parse the integer from the response from the service
int response;
if (sscanf(output.c_str(), "%*[^:]: %d", &response) == 1) {
    if(response == 1){            // Check if the response is 1 and set the test result to true
        test_result = true;
    }
    else{                         // Set the test result to false
        test_result = false;
    }
} else {
    FAIL() << "Failed to parse integer from service response";  // Fail the test if the response cannot be parsed
}