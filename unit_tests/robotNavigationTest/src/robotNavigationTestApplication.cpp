/*
 * robotNavigationTestApplication.cpp
 * 
 * Entry point for running unit tests with logging.
 */

#include <ros/ros.h>
#include <gtest/gtest.h>
#include <fstream>

class TestLogger : public ::testing::EmptyTestEventListener {
private:
    std::ofstream log_file;

public:
    TestLogger(const std::string& filename) {
        log_file.open(filename, std::ios::out | std::ios::app);
        if (!log_file.is_open()) {
            ROS_ERROR("Failed to open test log file: %s", filename.c_str());
        }
    }

    ~TestLogger() {
        if (log_file.is_open()) {
            log_file.close();
        }
    }

    void OnTestStart(const ::testing::TestInfo& test_info) override {
        log_file << "[ RUN      ] " << test_info.test_case_name() << "." << test_info.name() << std::endl;
    }

    void OnTestPartResult(const ::testing::TestPartResult& result) override {
        if (result.failed()) {
            log_file << "[  FAILED  ] " << result.file_name() << ":" << result.line_number() << " - " << result.summary() << std::endl;
        } else {
            log_file << "[       OK ] " << result.summary() << std::endl;
        }
    }

    void OnTestEnd(const ::testing::TestInfo& test_info) override {
        if (test_info.result()->Passed()) {
            log_file << "[       OK ] " << test_info.test_case_name() << "." << test_info.name() << std::endl;
        } else {
            log_file << "[  FAILED  ] " << test_info.test_case_name() << "." << test_info.name() << std::endl;
        }
        log_file.flush();
    }

    void OnTestProgramEnd(const ::testing::UnitTest& unit_test) override {
        log_file << "[==========] " << unit_test.total_test_count() << " tests from " << unit_test.test_suite_to_run_count()
                 << " test suite ran. (" << unit_test.elapsed_time() << " ms total)" << std::endl;

        log_file << "[  PASSED  ] " << unit_test.successful_test_count() << " tests." << std::endl;
        if (unit_test.failed_test_count() > 0) {
            log_file << "[  FAILED  ] " << unit_test.failed_test_count() << " tests." << std::endl;
        }
        log_file.flush();
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "robotNavigationTestApplication");
    ROS_INFO("Program: Robot Navigation unit Test Application");
    ROS_INFO("This project is funded by the African Engineering and Technology Network (Afretec)");
    ROS_INFO("Inclusive Digital Transformation Research Grant Programme.");
    ROS_INFO("Website: www.cssr4africa.org");
    ROS_INFO("This program comes with ABSOLUTELY NO WARRANTY.");
    ROS_INFO("robotNavigation: start-up.");
    ROS_INFO("robotNavigation: subscribed to /robotLocalization/pose");

    ::testing::InitGoogleTest(&argc, argv);
    // Enable verbose logging for Google Test
    ::testing::FLAGS_gtest_print_time = true; 
    ::testing::FLAGS_gtest_color = "yes";  
    ::testing::FLAGS_gtest_output = "xml:robotNavigationTestResults.xml"; // Optional: Export results


    // Define log file path
    std::string log_filename = "/home/br/workspace/pepper_rob_ws/src/cssr4Africa/unit_tests/robotNavigationTest/data/robotNavigationTestOutput.dat";

    // Register custom logger
    TestLogger logger(log_filename);
    ::testing::TestEventListeners& listeners = ::testing::UnitTest::GetInstance()->listeners();
    listeners.Append(&logger);

    // Run all tests
    int result = RUN_ALL_TESTS();

    if (result == 0) {
        ROS_INFO("All tests PASSED successfully!");
    } else {
        ROS_ERROR("Some tests FAILED! Check logs for details.");
    }

    while(ros::ok()){       
        ROS_INFO_THROTTLE(10, "Robot Navigation Node Running...");
        ros::spinOnce(); 

    }
    return result;
}
