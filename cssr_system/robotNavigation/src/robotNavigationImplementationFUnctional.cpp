

#include "robot_navigation/robotNavigationInterface.h"

// Directory where the package is located
std::string packagedir;

// Coordinates of the robot in the world (x, y, z, theta) - updated by subscribing to the /robotLocalization/pose topic
std::vector<double> robot_pose = {0.0, 0.0, 0.0};

// Waypoints for the robot to follow
waypointArrayType waypoints;

// Path for the robot to follow
pathType robot_path;

// Flag to indicate if a waypoint is being followed
bool waypointFlag;

// Graph for the path planning
graph g;
bool directed = false;              // directed graph

// Configuration parameters
std::string implementation_platform;
std::string environment_map_file;
std::string configuration_map_file;
int path_planning_algorithm;
bool social_distance_mode;
std::string simulator_topics;
std::string robot_topics;
string topics_filename;
bool verbose_mode;

// Publisher for the velocity commands
ros::Publisher navigation_velocity_publisher;

// Size of the map
int x_map_size;
int y_map_size;

// Window names to display the maps
string mapWindowName = "Environment Map"; 

locomotionParameterDataType locomotionParameterData;

geometry_msgs::Twist msg;

std::vector<double> leg_home_position = {0.0, 0.0, 0.0};  // Hip pitch, hip roll, knee pitch
std::vector<double> head_home_position = {0.0, 0.0};   // Head pitch and yaw
// Mat images to display the maps
// Mat mapImage;
// Mat mapImageColor;
// Mat mapImageLarge;
// Mat configurationSpaceImage;

/* 
 *   Function to read the the robot pose from an input file
 * @param:
 *   robot_pose_input: vector to store the robot pose
 *
 * @return:
 *    None
 */
void read_robot_pose_input(std::vector<double>& robot_pose_input){
    bool debug_mode = false;   // used to turn debug message on

    std::string data_file = "robotPose.dat";    // data filename
    std::string data_path;                                          // data path
    std::string data_path_and_file;                                 // data path and filename

    std::string x_key = "x";                                         // x key
    std::string y_key = "y";                                         // y key
    std::string theta_key = "theta";                                 // theta key

    std::string x_value;                                             // x value
    std::string y_value;                                             // y value
    std::string z_value;                                             // z value
    std::string theta_value;                                         // theta value

    // Construct the full path of the configuration file
    #ifdef ROS
        data_path = ros::package::getPath(ROS_PACKAGE_NAME).c_str();
    #else
        data_path = "..";
    #endif

    // set configuration path
    data_path += "/data/";
    data_path_and_file = data_path;
    data_path_and_file += data_file;

    if (debug_mode) printf("Data file is %s\n", data_path_and_file.c_str());

    // Open data file
    std::ifstream data_if(data_path_and_file.c_str());
    if (!data_if.is_open()){
        printf("Unable to open the data file %s\n", data_path_and_file.c_str());
        prompt_and_exit(1);
    }

    std::string data_line_read;  // variable to read the line in the file
    // Get key-value pairs from the data file
    while(std::getline(data_if, data_line_read)){
        std::istringstream iss(data_line_read);
        std::string param_key;
        std::string param_value;
        iss >> param_key;
        trim(param_key);
        std::getline(iss, param_value);
        iss >> param_value;
        trim(param_value);

        // Read the x value of the robot pose
        if (param_key == x_key){
            x_value = param_value;
            robot_pose_input[0] = std::stod(param_value);
        }

        // Read the y value of the robot pose
        else if (param_key == y_key){
            y_value = param_value;
            robot_pose_input[1] = std::stod(param_value);
        }

        // Read the theta value of the robot pose
        else if (param_key == theta_key){
            theta_value = param_value;
            robot_pose_input[2] = std::stod(param_value);
        }
    }
    // Close the data file
    data_if.close();

    if (debug_mode){
        printf("Robot location input:\n");
        printf("\tX: %f\n", robot_pose_input[0]);
        printf("\tY: %f\n", robot_pose_input[1]);
        printf("\tTheta: %f\n", robot_pose_input[2]);
    }
}

/*  
 *   Function to extract the topic from the topics file
 *   The function reads the topics file and extracts the topic for the specified key.
 *
 *   @param:
 *       key: the key to search for in the topics file
 *       topic_file_name: the topics filename
 *       topic_name: the topic name extracted
 *
 *   @return:
 *       0 if successful, 1 otherwise
 */
int extract_topic(string key, string topic_file_name, string *topic_name){
    bool debug = false;   // used to turn debug message on
    
    std::string topic_path;                                   // topic filename path
    std::string topic_path_and_file;                          // topic with path and file 

    std::string topic_value = "";                             // topic value

    // Construct the full path of the topic file
    #ifdef ROS
        topic_path = ros::package::getPath(ROS_PACKAGE_NAME).c_str();
    #else
        topic_path = "..";
    #endif

    // set topic path    
    topic_path += "/data/";
    topic_path_and_file = topic_path;
    topic_path_and_file += topic_file_name;

    if (debug) ROS_INFO("Topic file is %s\n", topic_path_and_file.c_str());

    // Open topic file
    std::ifstream topic_if(topic_path_and_file.c_str());
    if (!topic_if.is_open()){
        ROS_ERROR("Unable to open the topic file %s\n", topic_path_and_file.c_str());
        return 1;
    }

    std::string topic_line_read;   // variable to read the line in the file
    // Get key-value pairs from the topic file
    while(std::getline(topic_if, topic_line_read)){
        std::istringstream iss(topic_line_read);
        std::string param_key;
        std::string param_value;
        iss >> param_key;
        trim(param_key);
        std::getline(iss, param_value);
        iss >> param_value;
        trim(param_value);
        if (param_key == key) {                     // if the key is found
            topic_value = param_value;              // set the topic value
            break;
        }
    }
    topic_if.close();

    // verify the topic_value is not empty
    if (topic_value == ""){
        ROS_ERROR("Unable to find a valid topic for actuator: %s. Please check the topics file.\n", key.c_str());
        return 1;
    }

    *topic_name = topic_value;                      // set the topic name
    return 0;
}

/*  
 *   Function to move an actuator to a position when using linear interpolation
 *   The actuator is moved using the control client to the specified position
 *
 *   @param:
 *       client: the control client for the actuator
 *       joint_names: vector containing the joint names of the actuator
 *       duration: the duration of the movement
 *       open_hand: boolean to indicate if the hand should be open
 *       hand: the hand to be opened
 *       hand_topic: the topic for the hand
 *       position_name: the name of the position
 *       positions: vector containing the joint angles of the position to move the actuator to
 *
 *   @return:
 *       None
 */
void move_to_position(ControlClientPtr& client, const std::vector<std::string>& joint_names, double duration, 
                        bool open_hand, string hand, string hand_topic, 
                        const std::string& position_name, std::vector<double> positions){
    // Create a goal message
    control_msgs::FollowJointTrajectoryGoal goal;
    trajectory_msgs::JointTrajectory& trajectory = goal.trajectory;
    trajectory.joint_names = joint_names;                               // Set the joint names for the actuator to the specified joint names
    trajectory.points.resize(1);                                        // Set the number of points in the trajectory to 1

    trajectory.points[0].positions = positions;                         // Set the positions in the trajectory to the specified positions
    trajectory.points[0].time_from_start = ros::Duration(duration);     // Set the time from start of the trajectory to the specified duration

    // Joint angles of the hand positions
    std::vector<double> open_position = {1.0};
    std::vector<double> closed_position = {0.0};
    std::vector<double> home_position = {0.66608};

    // control client for the hand
    ControlClientPtr hand_client;

    // Goal messages for the hand
    control_msgs::FollowJointTrajectoryGoal hand_open_goal;
    control_msgs::FollowJointTrajectoryGoal hand_close_goal;

   //  // If the hand should be opened, create a client and send a goal to open the hand
   //  if(open_hand){
   //      hand_client = create_client(hand_topic);                    // Create a client for the hand
   //      if(hand_client == NULL){
   //          return;
   //      }
   //      std::vector<std::string> hand_joint_names = {hand};         // Set the joint names for the hand to the specified hand

   //      // Set the trajectory for the hand to open
   //      trajectory_msgs::JointTrajectory& hand_open_trajectory = hand_open_goal.trajectory;
   //      hand_open_trajectory.joint_names = hand_joint_names;
   //      hand_open_trajectory.points.resize(1);
   //      hand_open_trajectory.points[0].positions = open_position;
   //      hand_open_trajectory.points[0].time_from_start = ros::Duration(0.05);

   //      // Send the goal to open the hand
   //      hand_client->sendGoal(hand_open_goal);  // Open the hand
   //  }

    // Send the goal to move the actuator to the specified position
    client->sendGoal(goal);
    client->waitForResult(ros::Duration(duration)); // Wait for the actuator to reach the specified position

   //  // If the hand should not be opened, create a client and send a goal to take the hand to the home position
   //  if(!open_hand){
   //      hand_client = create_client(hand_topic);                    // Create a client for the hand
   //      if(hand_client == NULL){
   //          return;
   //      }
   //      std::vector<std::string> hand_joint_names = {hand};         // Set the joint names for the hand to the specified hand

   //      // Set the trajectory for the hand to move to home position
   //      trajectory_msgs::JointTrajectory& hand_close_trajectory = hand_close_goal.trajectory;
   //      hand_close_trajectory.joint_names = hand_joint_names;
   //      hand_close_trajectory.points.resize(1);
   //      hand_close_trajectory.points[0].positions = home_position;
   //      hand_close_trajectory.points[0].time_from_start = ros::Duration(0.05);

   //      // Send the goal to take the hand to the home position
   //      hand_client->sendGoal(hand_close_goal);  // Close the hand
   //  }

    return;
}

/* Read the overt attention configuration */
/* 
 *   Function to read the overt attention configuration.
 *   The configuration file contains the platform, camera, realignment threshold, x offset to head yaw, y offset to head pitch, simulator topics, robot topics, topics filename, and debug mode.
 *   The function reads the configuration file and sets the values for the specified parameters.
 * 
 * @param:
 *   platform: the platform value
 *   camera: the camera value
 *   realignment_threshold: the realignment threshold value
 *   x_offset_to_head_yaw: the x offset to head yaw value
 *   y_offset_to_head_pitch: the y offset to head pitch value
 *   simulator_topics: the simulator topics value
 *   robot_topics: the robot topics value
 *   topics_filename: the topics filename value
 *   debug_mode: the debug mode value
 * 
 * @return:
 *   0 if the configuration file is read successfully
 *   1 if the configuration file is not read successfully
 */
int read_configuration_file(string* platform, string* environment_map_file, string* configuration_map_file, int* path_planning_algorithm, bool* social_distance_mode, string* simulator_topics, string* robot_topics, string* topics_filename, bool* debug_mode){
    std::string config_file = "robotNavigationConfiguration.ini";       // data filename
    std::string config_path;                                            // data path
    std::string config_path_and_file;                                   // data path and filename
     
    std::string platform_key = "platform";                              // platform key 
    std::string environment_map_file_key = "environmentMap";            // camera key
    std::string configuration_map_file_key = "configurationMap";        // realignment threshold key
    std::string path_planning_algorithm_key = "pathPlanning";           // realignment threshold key
    std::string social_distance_mode_key = "socialDistance";          // x offset to head yaw key
    std::string simulator_topics_key = "simulatorTopics";               // simulator topics key
    std::string robot_topics_key = "robotTopics";                       // robot topics key
    std::string verbose_mode_key = "verboseMode";                       // verbose mode key

    std::string platform_value;                                         // platform value 
    std::string environment_map_file_value;                             // camera value
    std::string configuration_map_file_value;                           // realignment threshold value
    std::string path_planning_algorithm_value;                            // realignment threshold value
    std::string social_distance_mode_value;                             // x offset to head yaw value
    std::string simulator_topics_value;                                 // simulator topics value
    std::string robot_topics_value;                                     // robot topics value
    std::string verbose_mode_value;                                     // verbose mode value

    // Construct the full path of the configuration file
    #ifdef ROS
        config_path = ros::package::getPath(ROS_PACKAGE_NAME).c_str();
    #else
        data_path = "..";
    #endif

    // set configuration path
    config_path += "/config/";
    config_path_and_file = config_path;
    config_path_and_file += config_file;

    // Open configuration file
    std::ifstream data_if(config_path_and_file.c_str());
    if (!data_if.is_open()){
        printf("Unable to open the config file %s\n", config_path_and_file.c_str());
        return 1;
    }

    std::string data_line_read;  // variable to read the line in the file
    // Get key-value pairs from the configuration file
    while(std::getline(data_if, data_line_read)){
        std::istringstream iss(data_line_read);
        std::string param_key, param_value;
        iss >> param_key;
        trim(param_key);
        std::getline(iss, param_value);
        iss >> param_value;
        trim(param_value);
        // printf("paramKey: %s; paramValue: %s\n", paramKey.c_str(), paramValue.c_str());
        
        if (param_key == platform_key){ 
            boost::algorithm::to_lower(param_value); // modifies string to lower case
            platform_value = param_value;
            *platform = param_value;
            if(platform_value != "robot" && platform_value != "simulator"){
                printf("Platform value not supported. Supported values are: robot and simulator\n");
                return 1;
            }
        }
        
        else if (param_key == environment_map_file_key){ 
            environment_map_file_value = param_value;
            *environment_map_file = param_value;
        }

        else if (param_key == configuration_map_file_key){ 
            configuration_map_file_value = param_value;
            *configuration_map_file = param_value;
        }

        else if (param_key == path_planning_algorithm_key){ 
            path_planning_algorithm_value = param_value;
            boost::algorithm::to_lower(param_value); // modifies string to lower case
            if(param_value == "bfs"){
                *path_planning_algorithm = BFS_ALGORITHM;
            }
            else if((param_value == "dijkstra" || param_value == "dijsktra")){
                *path_planning_algorithm = DIJKSTRA_ALGORITHM;
            }
            else if((param_value == "a*") || (param_value == "a-star") || (param_value == "a_star") || (param_value == "astar")){
                *path_planning_algorithm = A_STAR_ALGORITHM;
            }
            else{
                printf("Path planning algorithm value not supported. Supported values are: bfs, dijkstra, and a*\n");
                return 1;
            }
        }

        else if (param_key == social_distance_mode_key){ 
            boost::algorithm::to_lower(param_value); // modifies string to lower case
            social_distance_mode_value = param_value;
            if(social_distance_mode_value == "true"){
                *social_distance_mode = true;
            }
            else if(social_distance_mode_value == "false"){
                *social_distance_mode = false;
            }
            else{
                printf("Social distance mode value not supported. Supported values are: true and false\n");
                return 1;
            }
        }

        else if (param_key == simulator_topics_key){ 
            simulator_topics_value = param_value;
            *simulator_topics = param_value;
        }

        else if (param_key == robot_topics_key){ 
            robot_topics_value = param_value;
            *robot_topics = param_value;
        }

        else if (param_key == verbose_mode_key){ 
            boost::algorithm::to_lower(param_value); // modifies string to lower case
            verbose_mode_value = param_value;
            if(verbose_mode_value == "true"){
                *debug_mode = true;
            }
            else if(verbose_mode_value == "false"){
                *debug_mode = false;
            }
            else{
                printf("Verbose mode value not supported. Supported values are: true and false\n");
                return 1;
            }
        }
    }
    data_if.close();

    if(*platform == "" || *environment_map_file == "" || *configuration_map_file == "" || *simulator_topics == "" || *robot_topics == ""){
        printf("Unable to find a valid configuration. Verify you have values in the configuration.\n");
        return 1;
    }

    if (platform_value == "robot"){
        *topics_filename = *robot_topics;
    }
    else if(platform_value == "simulator"){
        *topics_filename = *simulator_topics;
    }

    return 0;
}

/* Print the overt attention configuration */
void print_configuration(string platform, string environment_map_file, string configuration_map_file, int path_planning_algorithm, bool social_distance_mode, string simulator_topics, string robot_topics, string topics_filename, bool debug_mode){
    printf("Platform: %s\n", platform.c_str());
    printf("Environment Map File: %s\n", environment_map_file.c_str());
    printf("Configuration Map File: %s\n", configuration_map_file.c_str());
    printf("Path Planning Algorithm: %s\n", path_planning_algorithm == BFS_ALGORITHM ? "BFS" : path_planning_algorithm == DIJKSTRA_ALGORITHM ? "Dijsktra" : "A*");
    printf("Social Distance Mode: %s\n", social_distance_mode ? "true" : "false");
    printf("Simulator Topics: %s\n", simulator_topics.c_str());
    printf("Robot Topics: %s\n", robot_topics.c_str());
    printf("Topics Filename: %s\n", topics_filename.c_str());
    printf("Debug Mode: %s\n", debug_mode ? "true" : "false");
}

/******************************************************************************

global variables with the current robot pose

*******************************************************************************/

/* global variables with the initial and current robot pose, the odometry pose, and the difference between the initial pose and the initial odometry pose  */

float                  initial_x        = 0; 
float                  initial_y        = 0;
float                  initial_theta    = 0;
float                  current_x        = 0; 
float                  current_y        = 0; 
float                  current_theta    = 0;
float                  odom_x           = 0;
float                  odom_y           = 0;
float                  odom_theta       = 0;
float                  adjustment_x     = 0; 
float                  adjustment_y     = 0;
float                  adjustment_theta = 0;

/******************************************************************************

odomMessageReceived

Callback function, executed each time a new pose message arrives 

*******************************************************************************/

void odomMessageReceived(const nav_msgs::Odometry &msg)
{
   bool debug = false;

   float x, y;

   odom_x = msg.pose.pose.position.x;
   odom_y = msg.pose.pose.position.y;
   odom_theta = 2 * atan2(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w);

   /* change frame of reference from arbitrary odometry frame of reference to the world frame of reference */

   /* translation of origin */

   x = odom_x + adjustment_x - initial_x;
   y = odom_y + adjustment_y - initial_y;

   /* rotation about origin */

   current_x = x * cos(adjustment_theta) + y * -sin(adjustment_theta);
   current_y = x * sin(adjustment_theta) + y * cos(adjustment_theta);

   current_x += initial_x;
   current_y += initial_y;

   current_theta = odom_theta + adjustment_theta;

   /* check to ensure theta is still in the range -PI to +PI */

   if (current_theta < -PI)
      current_theta += 2 * PI;
   else if (current_theta > PI)
      current_theta -= 2 * PI;

   // printf("odom_x,y,theta %5.3f %5.3f %5.3f; adjustment_x,y,theta  %5.3f %5.3f %5.3f; x, y %5.3f %5.3f; current_x,y,theta %5.3f %5.3f %5.3f\n",  odom_x, odom_y, odom_theta, adjustment_x, adjustment_y, adjustment_theta, x, y, current_x, current_y, current_theta);

   if (debug)
   {
      printf("Odometry: position = (%5.3f, %5.3f) orientation = %5.3f\n", current_x, current_y, current_theta);
   }
   
}

/*******************************************************************************

readLocomotionParameterData

Read locomotion parameters from file     

*******************************************************************************/

void readLocomotionParameterData(char filename[], locomotionParameterDataType *locomotionParameterData)
{

   bool debug = true;
   int i;
   int j;
   int k;

   keyword keylist[NUMBER_OF_KEYS] = {
       "position_tolerance",
       "angle_tolerance_orienting",
       "angle_tolerance_going",
       "position_gain_dq",
       "angle_gain_dq",
       "position_gain_mimo",
       "angle_gain_mimo",
       "min_linear_velocity",
       "max_linear_velocity",
       "min_angular_velocity",
       "max_angular_velocity",
       "clearance",
       "shortest_path_algorithm",
       "robot_available"};

   keyword key;   // the key string when reading parameters
   keyword value; // the value string, used for the SHORTEST_PATH_ALGORITHM key
   keyword robot_state; // the robot_state string, used for the ROBOT_AVAILABLE key

   char input_string[STRING_LENGTH];
   FILE *fp_config;

   if ((fp_config = fopen(filename, "r")) == 0)
   {
     printf("Error can't open locomition parameter file %s\n", filename);
     prompt_and_exit(0);
   }

   /*** set default values ***/

   locomotionParameterData->position_tolerance = 0.025;
   locomotionParameterData->angle_tolerance_orienting = 0.075;
   locomotionParameterData->angle_tolerance_going = 0.075;
   locomotionParameterData->position_gain_dq = 0.3;
   locomotionParameterData->angle_gain_dq = 0.3;
   locomotionParameterData->position_gain_mimo = 0.2;
   locomotionParameterData->angle_gain_mimo = 0.5;
   locomotionParameterData->min_linear_velocity = 0.015;
   locomotionParameterData->max_linear_velocity = 0.5;
   locomotionParameterData->min_angular_velocity = 0.09;
   locomotionParameterData->max_angular_velocity = 1.0;
   locomotionParameterData->clearance = 0.05;
   locomotionParameterData->shortest_path_algorithm = ASTAR; // BFS, DIJKSTRA, ASTAR
   locomotionParameterData->robot_available = FALSE; // TRUE/FALSE

   /*** get the key-value pairs ***/

   for (i = 0; i < NUMBER_OF_KEYS; i++)
   {

     fgets(input_string, STRING_LENGTH, fp_config);
     // if (debug)  printf ("Input string: %s",input_string);

     /* extract the key */

     sscanf(input_string, " %s", key);

     for (j = 0; j < (int)strlen(key); j++)
        key[j] = tolower(key[j]);

     // if (debug)  printf ("key: %s\n",key);

     for (j = 0; j < NUMBER_OF_KEYS; j++)
     {
        if (strcmp(key, keylist[j]) == 0)
        {
           switch (j)
           {
           case 0:
              sscanf(input_string, " %s %f", key, &(locomotionParameterData->position_tolerance)); // position_tolerance
              break;
           case 1:
              sscanf(input_string, " %s %f", key, &(locomotionParameterData->angle_tolerance_orienting)); // angle_tolerance_orienting
              break;
           case 2:
              sscanf(input_string, " %s %f", key, &(locomotionParameterData->angle_tolerance_going)); // angle_tolerance_going
              break;
           case 3:
              sscanf(input_string, " %s %f", key, &(locomotionParameterData->position_gain_dq)); // position_gain_dq)
              break;
           case 4:
              sscanf(input_string, " %s %f", key, &(locomotionParameterData->angle_gain_dq)); // angle_gain_dq
              break;
           case 5:
              sscanf(input_string, " %s %f", key, &(locomotionParameterData->position_gain_mimo)); // position_gain_mimo
              break;
           case 6:
              sscanf(input_string, " %s %f", key, &(locomotionParameterData->angle_gain_mimo)); // angle_gain_mimo
              break;
           case 7:
              sscanf(input_string, " %s %f", key, &(locomotionParameterData->min_linear_velocity)); // min_linear_velocity
              break;
           case 8:
              sscanf(input_string, " %s %f", key, &(locomotionParameterData->max_linear_velocity)); // max_linear_velocity
              break;
           case 9:
              sscanf(input_string, " %s %f", key, &(locomotionParameterData->min_angular_velocity)); // min_angular_velocity
              break;
           case 10:
              sscanf(input_string, " %s %f", key, &(locomotionParameterData->max_angular_velocity)); // max_angular_velocity
              break;
           case 11:
              sscanf(input_string, " %s %f", key, &(locomotionParameterData->clearance)); // required clearance between robot and obstacle
              break;
           case 12:
              sscanf(input_string, " %s %s", key, value); // BFS,  Dijkstra,  ASTAR
              for (k = 0; k < (int)strlen(value); k++)
                 value[k] = tolower(value[k]);
              if (strcmp(value, "bfs") == 0)
                 locomotionParameterData->shortest_path_algorithm = BFS;
              else if (strcmp(value, "dijkstra") == 0)
                 locomotionParameterData->shortest_path_algorithm = DIJKSTRA;
              else if (strcmp(value, "astar") == 0)
                 locomotionParameterData->shortest_path_algorithm = ASTAR;
              else
                 locomotionParameterData->shortest_path_algorithm = ASTAR; // default value is A-star
              break;
           case 13:
              sscanf(input_string, " %s %s", key, robot_state); // true/false
              for (k = 0; k < (int)strlen(robot_state); k++)
                 robot_state[k] = tolower(robot_state[k]);
              if (strcmp(robot_state, "true") == 0)
                 locomotionParameterData->robot_available = TRUE;
              else if (strcmp(robot_state, "false") == 0)
                 locomotionParameterData->robot_available = FALSE;
           }
        }
     }
   }

   // printf("value %s\n", value);
   if (debug)
   {
     printf("POSITION_TOLERANCE:        %f\n", locomotionParameterData->position_tolerance);
     printf("ANGLE_TOLERANCE_ORIENTING: %f\n", locomotionParameterData->angle_tolerance_orienting);
     printf("ANGLE_TOLERANCE_GOING:     %f\n", locomotionParameterData->angle_tolerance_going);
     printf("POSITION_GAIN_DQ:          %f\n", locomotionParameterData->position_gain_dq);
     printf("ANGLE_GAIN_DQ:             %f\n", locomotionParameterData->angle_gain_dq);
     printf("POSITION_GAIN_MIMO:        %f\n", locomotionParameterData->position_gain_mimo);
     printf("ANGLE_GAIN_MIMO:           %f\n", locomotionParameterData->angle_gain_mimo);
     printf("MIN_LINEAR_VELOCITY:       %f\n", locomotionParameterData->min_linear_velocity);
     printf("MAX_LINEAR_VELOCITY:       %f\n", locomotionParameterData->max_linear_velocity);
     printf("MIN_ANGULAR_VELOCITY:      %f\n", locomotionParameterData->min_angular_velocity);
     printf("MAX_ANGULAR_VELOCITY:      %f\n", locomotionParameterData->max_angular_velocity);
     printf("CLEARANCE:                 %f\n", locomotionParameterData->clearance);
     printf("SHORTEST_PATH_ALGORITHM    %d\n", locomotionParameterData->shortest_path_algorithm);
     printf("ROBOT_AVAILABLE            %d\n", locomotionParameterData->robot_available);
   }
}

/***************************************************************************************************************************

   Function definitions for the graph data structure

****************************************************************************************************************************/


/* Breadth-First Search data structures */

bool processed[MAXV+1];   /* which vertices have been processed */
bool discovered[MAXV+1];  /* which vertices have been found */
int parent[MAXV+1];       /* discovery relation */

bool debug = false;

/* Initialize graph from data in a file                             */

void initialize_graph(graph *g, bool directed)
{

   int i; /* counter */

   g->nvertices = 0;
   g->nedges = 0;
   g->directed = directed;

   for (i = 1; i <= MAXV; i++)
     // g->degree[i] = 0;

     for (i = 1; i <= MAXV; i++)
         g->edges[i] = NULL;
}

/* map_to_graph                                                                      */
/* Initialize graph from map data                                                    */
/* Return false when the number of vertices is zero; true otherwise                  */
/* Generate a 4-connected graph with weights of 1 if the algorithm parameter == BFS  */
/* Generate an 8-connected graph with weights of 1 for horizontal and vertical edges */
/* and 1.414 for diagonal edges if the algorithm parameter == DIJKSTRA or ASTAR      */
/* BFS, DIJKSTRA, and ASTAR are defined in the include .h file                       */

bool map_to_graph(graph *g, bool directed, Mat map, int algorithm)
{

   long int i, j;    /* counter                */
   long int x, y;    /* vertices in edge (x,y) */
   float w1 = 1;     /* weight on horizontal and vertical */
   float w2 = 1.414; /* weight of diagonal edges          */
   int map_dimension_x;
   int map_dimension_y;

   map_dimension_x = map.rows;
   map_dimension_y = map.cols;

   initialize_graph(g, directed);

   g->nvertices = map_dimension_x * map_dimension_y;

   if (debug)
     printf("generate_graph %ld \n", g->nvertices);

   if (g->nvertices != 0)
   {

     /* insert horizontal edges on each line */

     for (i = 0; i < map_dimension_x; i++)
     {
         for (j = 0; j < map_dimension_y - 1; j++)
         {
            if (map.at<uchar>(i, j) == map.at<uchar>(i, j + 1))
            {
               x = vertex_number(i, j, map_dimension_y);     // was x
               y = vertex_number(i, j + 1, map_dimension_y); //
               insert_edge(g, x, y, directed, w1);
            }
         }
     }

     /* insert vertical edges on each column */

     for (i = 0; i < map_dimension_x - 1; i++)
     {  // NB -1
         for (j = 0; j < map_dimension_y; j++)
         {
            if (map.at<uchar>(i, j) == map.at<uchar>(i + 1, j))
            {
               x = vertex_number(i, j, map_dimension_y); // was x
               y = vertex_number(i + 1, j, map_dimension_y);
               insert_edge(g, x, y, directed, w1);
            }
         }
     }

     if (algorithm == DIJKSTRA || algorithm == ASTAR)
     {

         /* insert diagonal edges on each row ... but only for cells in the configuration space          */
         /* This is for the version where the freespace is 8 connected and the obstacles are 4 connected */

         for (i = 0; i < map_dimension_x - 1; i++)
         { // NB -1 ... forward
            for (j = 0; j < map_dimension_y - 1; j++)
            { // NB -1    diagonal

               if ((map.at<uchar>(i, j) == (map.at<uchar>(i + 1, j + 1)) && (map.at<uchar>(i, j) == 255)))
               {
                  x = vertex_number(i, j, map_dimension_y);         // was x
                  y = vertex_number(i + 1, j + 1, map_dimension_y); //
                  insert_edge(g, x, y, directed, w2);
               }
            }
         }

         for (i = 0; i < map_dimension_x - 1; i++)
         { // NB -1 ... backward
            for (j = 1; j < map_dimension_y; j++)
            { // NB  1 ... diagonal
               if ((map.at<uchar>(i, j) == map.at<uchar>(i + 1, j - 1) && (map.at<uchar>(i, j) == 255)))
               {
                  x = vertex_number(i, j, map_dimension_y);         // was x
                  y = vertex_number(i + 1, j - 1, map_dimension_y); //
                  insert_edge(g, x, y, directed, w2);
               }
            }
         }
     }

     return (true);
   }
   else
   {
     return (false);
   }
}

/* mark the part from start to end on the map */

bool find_path(graph *g, int start, int end, int parents[], Mat map, int algorithm, pathType &path)
{

   bool is_path;
   int x, y;
   int map_dimension_x;

   map_dimension_x = map.cols;

   if (end == -1)
   {
     is_path = false; // some vertex on the path back from the end has no parent (not counting start)
     if (debug)
         printf("\nis_path is false: ");
   }
   else if ((start == end))
   {
     if (debug)
         printf("%d", start);

     x = row_number(start, map_dimension_x);
     y = column_number(start, map_dimension_x);
     map.at<uchar>(x, y) = 128; // mark path on map

     path.point[path.numberOfPoints].x = x;
     path.point[path.numberOfPoints].y = y;
     path.numberOfPoints++;

     is_path = true; // we have reached the start vertex
   }
   else
   {
     is_path = find_path(g, start, parents[end], parents, map, algorithm, path);

     if (is_path)
     { // mark only if the path exists
         x = row_number(end, map_dimension_x);
         y = column_number(end, map_dimension_x);
         map.at<uchar>(x, y) = 128; // mark path on map

         path.point[path.numberOfPoints].x = x;
         path.point[path.numberOfPoints].y = y;
         path.numberOfPoints++;

         if (debug)
            printf(" %d", end);
     }
   }
   return (is_path);
}

/* mark a part from start to end on the map      */
/* DV abstract version that hides implementation */

bool find_path(graph *g, int start, int end, Mat map, int algorithm, pathType &path)
{

   bool is_path;

   if (debug)
     printf("Path from robot to goal: ");

   path.numberOfPoints = 0;

   initialize_search(g);
   if (algorithm == BFS)
   {
     bfs(g, start);
   }
   else if (algorithm == DIJKSTRA)
   {
     dijkstra(g, start);
   }
   else if (algorithm == ASTAR)
   {
     astar(g, start, end, map);
   }
   else
   {
     printf("Unknown traversal algorithm\n");
   }

   is_path = find_path(g, start, end, parent, map, algorithm, path);

   if (debug)
     printf("\n");

   return (is_path);
}

void compute_waypoints(pathType path, waypointArrayType &waypoints)
{

   /* this is just a stub and simply sets the number of waypoints to zero */

   bool debug = false;
   int i, j = 1;
   float orient;

   // waypoints.numberOfPoints = path.numberOfPoints - 1;
   waypoints.numberOfPoints = path.numberOfPoints / 25;
   waypoints.point[0].x = path.point[0].x;
   waypoints.point[0].y = path.point[0].y;

   // for (i=1; i<path.numberOfPoints; i++) {
   //    //if(fabs(path.point[i].x - path.point[i-1].x) >= 1 || fabs(path.point[i].y - path.point[i-1].y) >= 1)
   //    {
   //       orient = atan2((path.point[i].y - path.point[i-1].y),(path.point[i].x - path.point[i-1].x));
   //       if (orient >= 1.571 || orient <= -1.571)
   //       {
   //          waypoints.point[j].x = path.point[i].x;
   //          waypoints.point[j].y = path.point[i].y;
   //          j++;
   //       }
   //       printf("Orientation change = %.3f\n", orient);
   //    if (debug)
   //       printf("Waypoint selected  %d %d %5.3f\n", waypoints.point[i].x, waypoints.point[i].y, waypoints.point[i].theta);
   //    }
   // }

   // for (i=1; i<path.numberOfPoints; i++) {
   //    orient = atan2((path.point[i].y - path.point[i-1].y),(path.point[i].x - path.point[i-1].x));
   //    if(orient >= 0.7855)
   //    {
   //       waypoints.point[j].x = path.point[i].x;
   //       waypoints.point[j].y = path.point[i].y;
   //       j++;
   //    // waypoints.point[i].x = path.point[j].x;
   //    // waypoints.point[i].y = path.point[j].y;
   //    //j+=20;
   //    if (debug)
   //       printf("Waypoint selected  %d %d %5.3f\n", waypoints.point[i].x, waypoints.point[i].y, waypoints.point[i].theta);
   //    }
   // }
   // waypoints.numberOfPoints = j;

   for (i = 0; i < waypoints.numberOfPoints; i++)
   {
     waypoints.point[i].x = path.point[j].x;
     waypoints.point[i].y = path.point[j].y;
     j += 25;
     if (debug)
         printf("Waypoint selected  %d %d %5.3f\n", waypoints.point[i].x, waypoints.point[i].y, waypoints.point[i].theta);
   }
   waypoints.point[0].theta = 1.5708;
   waypoints.point[i].x = path.point[path.numberOfPoints - 1].x;
   waypoints.point[i].y = path.point[path.numberOfPoints - 1].y;
   if (debug)
     printf("Waypoint selected  %d %d %5.3f\n", waypoints.point[i].x, waypoints.point[i].y, waypoints.point[i].theta);
}

/* Initialize graph from data in a file                             */

void insert_edge(graph *g, int x, int y, bool directed, float w)
{

   edgenode *p; /* temporary pointer */

   p = (EDGENODE_PTR)malloc(sizeof(edgenode)); /* allocate edgenode storage */
                                               //^^^^^^^^^^^^^^ ADDED CAST. DV 7/11/2014

   p->weight = w;
   p->y = y;
   p->hidden = false; // DV all new edges are initially visible
   p->next = g->edges[x];

   g->edges[x] = p; /* insert at head of list        */

   // g->degree[x] ++;

   if (directed == false)           /* NB: if undirected add         */
     insert_edge(g, y, x, true, w); /* the reverse edge recursively  */
   else                             /* but directed TRUE so we do it */
     g->nedges++;                   /* only once                     */
}

/* Print a graph                                                    */

void print_graph(graph *g)
{

   int i;       /* counter           */
   edgenode *p; /* temporary pointer */

   for (i = 1; i <= g->nvertices; i++)
   {
     printf("%d: ", i);
     p = g->edges[i];
     while (p != NULL)
     {

         if (p->hidden == false)
         { // DV
            printf(" %ld-%5.3f", p->y, p->weight);
         }
         p = p->next;
     }
     printf("\n");
   }
}

/* Each vertex is initialized as undiscovered:                      */

void initialize_search(graph *g)
{

   int i; /* counter */

   for (i = 1; i <= g->nvertices; i++)
   {
     processed[i] = discovered[i] = FALSE;
     parent[i] = -1;
   }
}

/* Once a vertex is discovered, it is placed on a queue.           */
/* Since we process these vertices in first-in, first-out order,   */
/* the oldest vertices are expanded first, which are exactly those */
/* closest to the root                                             */

void bfs(graph *g, int start)
{
   dvqueue q;   /* queue of vertices to visit */
   int v;       /* current vertex             */
   int y;       /* successor vertex           */
   edgenode *p; /* temporary pointer          */

   init_queue(&q);
   enqueue(&q, start);
   discovered[start] = TRUE;
   while (empty_queue(&q) == FALSE)
   {
     v = dequeue(&q);
     process_vertex_early(v);
     processed[v] = TRUE;
     p = g->edges[v];

     while (p != NULL)
     {

         // DV ... new functionality ... if p->hidden != true carry on

         if (p->hidden != true)
         { // DV
            y = p->y;
            if ((processed[y] == FALSE) || g->directed)
               process_edge(v, y);
            if (discovered[y] == FALSE)
            {
               enqueue(&q, y);
               discovered[y] = TRUE;
               parent[y] = v;
            }
         }
         p = p->next;
     }
     process_vertex_late(v);
   }
}

/* The exact behaviour of bfs depends on the functions             */
/*    process vertex early()                                       */
/*    process vertex late()                                        */
/*    process edge()                                               */
/* These functions allow us to customize what the traversal does   */
/* as it makes its official visit to each edge and each vertex.    */
/* Here, e.g., we will do all of vertex processing on entry        */
/* (to print each vertex and edge exactly once)                    */
/* so process vertex late() returns without action                 */

void process_vertex_late(int v) {
}

void process_vertex_early(int v){
   //printf("processed vertex %d\n",v);
}

void process_edge(int x, int y) {
   //printf("processed edge (%d,%d)\n",x,y);
}

/* this version just counts the number of edges                     */
/*
void process_edge(int x, int y) {
   nedges = nedges + 1;
}
*/



/* Dijkstra's algorithm: implementation based of Prim's algorithm */

void dijkstra(graph *g, int start)
{
   int i;                    /* counter                          */
   edgenode *p;              /* temporary pointer                */
   bool intree[MAXV + 1];    /* is the vertex in the tree yet?   */
   float distance[MAXV + 1]; /* cost of adding to tree           */
   // int parent[MAXV+1];   /* parent vertex                    */
   int v;        /* current vertex to process        */
   int w;        /* candidate next vertex            */
   float weight; /* edge weight                      */
   float dist;   /* best current distance from start */

   bool debug = true;

   if (debug)
     printf("Dijkstra ... entering\n");

   for (i = 1; i <= g->nvertices; i++)
   {
     intree[i] = FALSE;
     distance[i] = INT_MAX;
     parent[i] = -1;
   }

   distance[start] = 0;
   v = start;

   while (intree[v] == FALSE)
   {
     intree[v] = TRUE;
     p = g->edges[v];
     while (p != NULL)
     {
         w = p->y;
         weight = p->weight;
         if ((distance[v] + weight < distance[w]))
         { // changes from Prim
            distance[w] = distance[v] + weight;
            parent[w] = v;
         }
         p = p->next;
     }
     v = 1;
     dist = INT_MAX;
     for (i = 1; i <= g->nvertices; i++)
         if ((intree[i] == FALSE) && (distance[i] < dist))
         {
            dist = distance[i];
            v = i;
         }
   }
   if (debug)
     printf("Dijkstra ... exiting\n");
}

/* Heuristic estimate for the A-star algorithm using diagonal distance*/
float calculate_heuristic(int start, int goal, Mat map)
{
   int start_x, start_y, goal_x, goal_y; // Variables to store x-y coordinates of the start and goal position
   int number_of_columns = map.cols;     // Extract number of columns from the dimension of the map

   float D = 1.00;     // Weight of horizontal or vertical movement
   float D2 = sqrt(2); // Weight of diagonal movement

   /* Obtain the x,y-coordinates of the start and end positions on the map */
   start_x = row_number(start, number_of_columns); //
   start_y = column_number(start, number_of_columns);

   goal_x = row_number(goal, number_of_columns);
   goal_y = column_number(goal, number_of_columns);

   float dx = abs(start_x - goal_x);
   float dy = abs(start_y - goal_y);
   // return D * (dx * dx + dy * dy);
   return D * (dx + dy) + (D2 - 2 * D) * min(dx, dy);
}


/* Functional A-star algorithm - using static variables: 
 * implementation based of DIjkstra algorithm to optimize for a single destination*/
// void astar(graph *g, int start, int goal, Mat map)
// {

//    int i;                                 /* counter                          */
//    edgenode *p;                           /* temporary pointer                */
//    static bool intree[MAXV + 1];          /* is the vertex in the tree yet?   */
//    static bool inpath[MAXV + 1];          /* is the vertex in the path yet?   */
//    static float distance[MAXV + 1];       /* cost of adding to tree       g     */
//    static float heuristic[MAXV + 1];      /* total cost of adding to tree    h       */
//    static float total_distance[MAXV + 1]; // f
//    // int parent[MAXV+1];   /* parent vertex                    */
//    int v;        /* current vertex to process        */
//    int w;        /* candidate next vertex            */
//    float weight; /* edge weight                      */
//    float dist;   /* best current distance from start */

//    bool debug = true;

//    if (debug)
//      printf("A-star ... entering\n");

//    for (i = 1; i <= g->nvertices; i++)
//    {
//      intree[i] = FALSE;
//      inpath[i] = FALSE;
//      distance[i] = INT_MAX;       // g
//      total_distance[i] = INT_MAX; // f
//      parent[i] = -1;
//      heuristic[i] = calculate_heuristic(i, goal, map);
//    }

//    distance[start] = 0;
//    v = start;

//    while (v != goal)
//    {
//      intree[v] = TRUE;
//      for (i = 1; i <= g->nvertices; i++)
//      {
//          inpath[i] = FALSE;
//      }
//      p = g->edges[v];
//      while (p != NULL)
//      {
//          w = p->y;
//          weight = p->weight;
//          if ((distance[v] + weight < distance[w]))
//          { // changes from Prim
//             distance[w] = distance[v] + weight;
//             parent[w] = v;
//             total_distance[w] = distance[w] + heuristic[w];
//          }
//          p = p->next;
//      }
//      // int v_parent = v;
//      v = 1;
//      dist = INT_MAX;
//      for (i = 1; i <= g->nvertices; i++)
//          if ((intree[i] == FALSE) && (inpath[i] == FALSE) && (total_distance[i] < dist))
//          {
//             dist = total_distance[i];
//             v = i;
//          }
//      inpath[v] = TRUE;
//      // if (v == goal){
//      //    parent[v] = v_parent;
//      // }
//    }
//    if (debug)
//      printf("A-star ... exiting\n");
// }

/* Functional A-star algorithm - using dynamic memory allocation: 
 * implementation based of Dijkstra algorithm to optimize for a single destination*/
void astar(graph *g, int start, int goal, Mat map)
{

   int i;                 /* counter                          */
   edgenode *p;           /* temporary pointer                */
   bool *intree;          /* is the vertex in the tree yet?   */
   bool *inpath;          /* is the vertex in the path yet?   */
   float *distance;       /* cost of adding to tree       g     */
   float *heuristic;      /* heuristic cost of getting to the destination from a node    h       */
   float *total_distance; /* total cost of adding to tree    f       */

   // Dynamically allocate memory using calloc()
   intree = (bool *)calloc(MAXV + 1, sizeof(bool));
   inpath = (bool *)calloc(MAXV + 1, sizeof(bool));
   distance = (float *)calloc(MAXV + 1, sizeof(float));
   heuristic = (float *)calloc(MAXV + 1, sizeof(float));
   total_distance = (float *)calloc(MAXV + 1, sizeof(float));

   // int parent[MAXV+1];   /* parent vertex                    */
   int v;        /* current vertex to process        */
   int w;        /* candidate next vertex            */
   float weight; /* edge weight                      */
   float dist;   /* best current distance from start */

   bool debug = true;

   if (debug)
     printf("A-star ... entering\n");

   for (i = 1; i <= g->nvertices; i++)
   {
     intree[i] = FALSE;
     inpath[i] = FALSE;
     distance[i] = INT_MAX;       // g
     total_distance[i] = INT_MAX; // f
     parent[i] = -1;
     heuristic[i] = calculate_heuristic(i, goal, map);
   }

   distance[start] = 0;
   v = start;

   while (v != goal)
   {
     intree[v] = TRUE;
     for (i = 1; i <= g->nvertices; i++)
     {
         inpath[i] = FALSE;
     }
     p = g->edges[v];
     while (p != NULL)
     {
         w = p->y;
         weight = p->weight;
         if ((distance[v] + weight < distance[w]))
         { // changes from Prim
            distance[w] = distance[v] + weight;
            parent[w] = v;
            total_distance[w] = distance[w] + heuristic[w];
         }
         p = p->next;
     }
     // int v_parent = v;
     v = 1;
     dist = INT_MAX;
     for (i = 1; i <= g->nvertices; i++)
         if ((intree[i] == FALSE) && (inpath[i] == FALSE) && (total_distance[i] < dist))
         {
            dist = total_distance[i];
            v = i;
         }
     inpath[v] = TRUE;
     // if (v == goal){
     //    parent[v] = v_parent;
     // }
   }

   // Free the memory
   free(intree);
   free(inpath);
   free(distance);
   free(heuristic);
   free(total_distance);

   if (debug)
     printf("A-star ... exiting\n");
}

/* adapted from original to return true only if it is possible to reach the end from the start */

bool find_path(int start, int end, int parents[])
{

   bool is_path;

   if (end == -1)
   {
     is_path = false; // some vertex on the path back from the end has no parent (not counting start)
     if (debug)
         printf("\nis_path is false: ");
   }
   else if ((start == end))
   {
     if (debug)
         printf("\n%d", start);
     is_path = true; // we have reached the start vertex
   }
   else
   {
     is_path = find_path(start, parents[end], parents);
     if (debug)
         printf(" %d", end);
   }
   return (is_path);
}

/* DV abstract version that hides implementation */

bool find_path(graph *g, int start, int end)
{
   bool is_path;

   initialize_search(g);
   bfs(g, start);
   is_path = find_path(start, end, parent);
   if (debug)
     printf("\n");
   return (is_path);
}

/* adapted from find_path */

bool print_path(FILE *fp_out, int start, int end, int parents[])
{

   bool is_path;

   if (end == -1)
   {
     is_path = false; // some vertex on the path back from the end has no parent (not counting start)
     fprintf(fp_out, "\nis_path is false: ");
   }
   else if ((start == end))
   {
     fprintf(fp_out, "%d", start);
     is_path = true; // we have reached the start vertex
   }
   else
   {
     is_path = print_path(fp_out, start, parents[end], parents);
     fprintf(fp_out, " %d", end);
   }
   return (is_path);
}

/* DV abstract version that hides implementation */

bool print_path(FILE *fp_out, graph *g, int start, int end)
{
   bool is_path;

   fprintf(fp_out, "Route = ");
   initialize_search(g);
   bfs(g, start);
   is_path = print_path(fp_out, start, end, parent);
   fprintf(fp_out, "\n\n");
   return (is_path);
}

/***************************************************************************************************************************

   Function definitions for the queue data structures

****************************************************************************************************************************/

/*	queue

	Implementation of a FIFO queue abstract data type.

	by: Steven Skiena
	begun: March 27, 2002
*/


/*
Copyright 2003 by Steven S. Skiena; all rights reserved. 

Permission is granted for use in non-commerical applications
provided this copyright notice remains intact and unchanged.

This program appears in my book:

"Programming Challenges: The Programming Contest Training Manual"
by Steven Skiena and Miguel Revilla, Springer-Verlag, New York 2003.

See our website www.programming-challenges.com for additional information.

This book can be ordered from Amazon.com at

http://www.amazon.com/exec/obidos/ASIN/0387001638/thealgorithmrepo/

*/

/* renamed queue to dvqueue to avoid name conflict. DV 16/11/2021 */

void init_queue(dvqueue *q)
{
   q->first = 0;
   q->last = QUEUESIZE - 1;
   q->count = 0;
}

void enqueue(dvqueue *q, item_type x)
{
   if (q->count >= QUEUESIZE)
     printf("Warning: queue overflow enqueue x=%d\n", x);
   else
   {
     q->last = (q->last + 1) % QUEUESIZE;
     q->q[q->last] = x;
     q->count = q->count + 1;
   }
}

item_type dequeue(dvqueue *q)
{
   item_type x;

   if (q->count <= 0)
     printf("Warning: empty queue dequeue.\n");
   else
   {
     x = q->q[q->first];
     q->first = (q->first + 1) % QUEUESIZE;
     q->count = q->count - 1;
   }

   return (x);
}

item_type headq(dvqueue *q)
{
   return (q->q[q->first]);
}

int empty_queue(dvqueue *q)
{
   if (q->count <= 0)
     return (TRUE);
   else
     return (FALSE);
}

void print_queue(dvqueue *q)
{
   int i;

   i = q->first;

   while (i != q->last)
   {
     printf("%d ", q->q[i]);
     i = (i + 1) % QUEUESIZE;
   }

   printf("%1d ", q->q[i]);
   printf("\n");
}

/***************************************************************************************************************************

   Function definitions for the mapping between graph and map data structures

****************************************************************************************************************************/

int row_number(int vertex_number, int number_of_columns)
{
   int n;
   n = (vertex_number - 1) / number_of_columns;
   return (n);
}

int column_number(int vertex_number, int number_of_columns)
{
   int n;
   n = (vertex_number - 1) % number_of_columns;
   return (n);
}

int vertex_number(int row, int column, int number_of_columns)
{
   int n;
   n = (row * number_of_columns) + column + 1;
   return (n);
}

/***************************************************************************************************************************

   General purpose function definitions 

****************************************************************************************************************************/

/* return the sign of a number as +/- 1 */

int signnum(float x)
{
    if (x >= 0.0){
        return 1;
    }
    return -1;
}

void display_error_and_exit(char error_message[])
{
   printf("%s\n", error_message);
   printf("Hit any key to continue >>");
   getchar();
   exit(1);
}

void print_message_to_file(FILE *fp, char message[])
{
   fprintf(fp, "The message is: %s\n", message);
}

#ifdef ROS
/**
 Linux (POSIX) implementation of _kbhit().
 Morgan McGuire, morgan@cs.brown.edu
 */
int _kbhit() {
    static const int STDIN = 0;
    static bool initialized = false;

    if (! initialized) {
        // Use termios to turn off line buffering
        termios term;
        tcgetattr(STDIN, &term);
        term.c_lflag &= ~ICANON;
        tcsetattr(STDIN, TCSANOW, &term);
        setbuf(stdin, NULL);
        initialized = true;
    }

    int bytesWaiting;
    ioctl(STDIN, FIONREAD, &bytesWaiting);
    return bytesWaiting;
}
#endif


int plan_robot_path(float start_x, float start_y, float start_theta, float goal_x, float goal_y, float goal_theta, int path_planning_algorithm, Mat mapImage, Mat configurationSpaceImage, bool debug){
   // Vertex number of the start and goal cells
    int robot;                              // vertex number of start cell
    int goal;                               // vertex number of goal cell

    // Map values of the start location
    int x_start_map;
    int y_start_map;

    // Map values of the goal location
    int x_goal_map;
    int y_goal_map;

    // Waypoint values
    float x_waypoint;
    float y_waypoint;
    float theta_waypoint;

    // parameters for image write
    vector<int> compressionParams;  

    // Map image in colour
    Mat mapImageColor;

    // Map image in large format
    Mat mapImageLarge;

    // scale the map and configuration images by this factor before displaying
    float image_display_scale_factor  = 4.0;

    // Convert the start location to map coordinates
    x_start_map = y_map_size - (int) (start_y * 100);  // NB: convert to cm and change frame of reference
    y_start_map = (int) (start_x * 100);               // ibid.

    // Convert the goal location to map coordinates
    x_goal_map = y_map_size - (int) (goal_y * 100);   // NB: convert to cm and change frame of reference
    y_goal_map = (int) (goal_x * 100);                //


    // Obtain the vertex number of the start and goal cells
    robot = vertex_number(x_start_map, y_start_map, x_map_size);
    goal  = vertex_number(x_goal_map,  y_goal_map,  x_map_size);

    /* find the shortest path from the start position to the goal position */
    /* ------------------------------------------------------------------- */

    if (!find_path(&g, robot, goal, configurationSpaceImage, path_planning_algorithm, robot_path)) {
        printf("No path from (%5.3f %5.3f) to (%5.3f %5.3f)\n", start_x, start_y, goal_x, goal_y);
        return 0;
    }  

    /* get the waypoints  */
    /* ------------------ */
    
    compute_waypoints(robot_path, waypoints); // This computes the waypoints for the robot to go through


    /* Draw path in colour with waypoint and display map and configuration space */
    /* ------------------------------------------------------------------------- */
    
    cvtColor(mapImage,mapImageColor,COLOR_GRAY2BGR);

    /* Draw a grid on the output map imitating the size of a tile in the laboratory. This can be commented out */
    int dist=60;

    for(int i=0;i<mapImageColor.rows;i+=dist)
        line(mapImageColor,Point(0,i),Point(mapImageColor.cols,i),Scalar(0,0,0));

    for(int i=0;i<mapImageColor.cols;i+=dist)
        line(mapImageColor,Point(i,0),Point(i,mapImageColor.rows),Scalar(0,0,0));
    
    for (int i=0; i<robot_path.numberOfPoints; i++) {
        mapImageColor.at<Vec3b>(robot_path.point[i].x,robot_path.point[i].y)[0] = 0;
        mapImageColor.at<Vec3b>(robot_path.point[i].x,robot_path.point[i].y)[1] = 0;
        mapImageColor.at<Vec3b>(robot_path.point[i].x,robot_path.point[i].y)[2] = 255;
    }

    /* Mark the waypoints on the map */

    for (int i=0; i<waypoints.numberOfPoints + 1; i++) {
        x_waypoint = (float) waypoints.point[i].y / 100;
        y_waypoint = (float) (y_map_size - waypoints.point[i].x)/100;
        float x_1_waypoint = (float) waypoints.point[i-1].y / 100;
        float y_1_waypoint = (float) (y_map_size - waypoints.point[i-1].x)/100;
        float x1_waypoint = (float) waypoints.point[i+1].y / 100;
        float y1_waypoint = (float) (y_map_size - waypoints.point[i+1].x)/100;
        // if(x_waypoint == x1_waypoint || y_waypoint == y1_waypoint)
        // {
        //    continue;
        // }
        line(mapImageColor,Point(waypoints.point[i].y-CROSS_HAIR_SIZE/2,waypoints.point[i].x),Point(waypoints.point[i].y+CROSS_HAIR_SIZE/2,waypoints.point[i].x),Scalar(0, 255, 0),1, LINE_AA); // Green
        line(mapImageColor,Point(waypoints.point[i].y,waypoints.point[i].x-CROSS_HAIR_SIZE/2),Point(waypoints.point[i].y,waypoints.point[i].x+CROSS_HAIR_SIZE/2),Scalar(0, 255, 0),1, LINE_AA);
    }

    resize(mapImageColor, mapImageLarge, Size(mapImage.cols * image_display_scale_factor, mapImage.rows * image_display_scale_factor), INTER_NEAREST);

    compressionParams.push_back(IMWRITE_PNG_COMPRESSION);
    compressionParams.push_back(9);                                  // 9 implies maximum compression   

    save_waypoint_map(compressionParams, mapImageLarge); // Save the map with the path and waypoints

    return 1;              // Return 1 if the path is found
}

// int navigate_to_goal(float start_x, float start_y, float start_theta, float goal_x, float goal_y, float goal_theta, int path_planning_algorithm, Mat mapImage, ros::Publisher velocity_publisher, bool debug){
int navigate_to_goal(float start_x, float start_y, float start_theta, float goal_x, float goal_y, float goal_theta, int path_planning_algorithm, Mat mapImage, Mat configurationSpaceImage, ros::Publisher velocity_publisher, bool debug){
   // Set the publish rate for the velocity commands
   ros::Rate rate(PUBLISH_RATE); // Publish  at this rate (in Hz)  until the node is shut down


   int path_found = 0;
   path_found = plan_robot_path(start_x, start_y, start_theta, goal_x, goal_y, goal_theta, path_planning_algorithm, mapImage, configurationSpaceImage, debug);
   if (path_found == 0){
      return 0;                        // Return 0 if the path is not found
   }
   printf("Path found from (%5.3f %5.3f %5.3f) to (%5.3f %5.3f %5.3f)\n", start_x, start_y, start_theta, goal_x, goal_y, goal_theta);

   // Move the leg to the home position
   int send_leg_to_home;
   send_leg_to_home = go_to_home("Leg", topics_filename, debug);

   // Move the head back to the home position
   // send_leg_to_home = go_to_home("Head", topics_filename, debug);
   move_robot(start_x, start_y, start_theta, goal_x, goal_y, goal_theta, velocity_publisher, rate, debug);
    return 1;
}

/*  
 *   Function to create a control client
 *   The function creates a control client for the specified topic.
 *
 *   @param:
 *       topic_name: the topic name
 *
 *   @return:
 *       the control client
 */
ControlClientPtr create_client(const std::string& topic_name) {
    // Create a new action client
    ControlClientPtr actionClient(new ControlClient(topic_name, true));
    int max_iterations = 10;        // maximum number of iterations to wait for the server to come up

    for (int iterations = 0; iterations < max_iterations; ++iterations) {
        if (actionClient->waitForServer(ros::Duration(5.0))) {
            return actionClient;   // return the action client if the server is available
        }
        ROS_DEBUG("Waiting for the %s controller to come up", topic_name.c_str());
    }
    // Throw an exception if the server is not available and client creation fails
    ROS_ERROR("Error creating action client for %s controller: Server not available", topic_name.c_str());
    return actionClient;  // return the action client
}


/* 
 *   Function to return all joints of an actuator to the home position.
 *   This function is called on only one actuator at a time.
 *   It used the global variable set above for the home positions of the actuators.
 *
 * @param:
 *   actuator: string indicating the actuator to move to the home position
 *   topics_filename: string indicating the topics filename
 *   interpolation: integer indicating the interpolation type
 *   debug: boolean indicating the debug mode
 *
 * @return:
 *   None
 */
int go_to_home(std::string actuator, std::string topics_filename, bool debug){
    // ros::Duration(0.5).sleep(); // Wait for one second to ensure that the joint states are updated
    std::vector<double> actuator_state;                             // stores the current state of the actuator joints
    std::vector<double> actuator_home_position;                     // stores the home position of the actuator joints
    ControlClientPtr actuator_client;                               // action client to control the actuator joints
    std::vector<std::string> actuator_joint_names;                  // stores the joint names of the actuator joints
    std::string actuator_topic;                                     // stores the topic of the actuator for control
    int number_of_joints;                                           // stores the number of joints of the actuator
    double home_duration;                                           // stores the duration to move to the home position

    // Set the open hand flag and the hand topic. ust for default, the hand is not to be open in home position
    bool open_hand = false;
    string hand = "RHand";
    string hand_topic = "/pepper_dcm/RightHand_controller/follow_joint_trajectory";

    // Extract the actuator topic
    if(extract_topic(actuator, topics_filename, &actuator_topic)){
        return 0;
    }

    // Set the home duration
    home_duration = 0.5;

    // Set the actuator state, home position, joint names, client and number of joints based on the actuator
   //  if(actuator == "RArm"){                             // Right arm
   //      actuator_state = right_arm_joint_states;
   //      actuator_home_position = right_arm_home_position;
   //      actuator_joint_names = {"RShoulderPitch", "RShoulderRoll", "RElbowRoll", "RElbowYaw", "RWristYaw"};
   //      actuator_client = create_client(actuator_topic);
   //      if(actuator_client == NULL){
   //          return 0;
   //      }
   //      number_of_joints = actuator_joint_names.size();
   //      hand = "RHand";
   //      if(extract_topic(hand, topics_filename, &hand_topic)){
   //          return 0;
   //      }
   //  }
   //  else if(actuator == "LArm"){                        // Left arm
   //      actuator_state = left_arm_joint_states;
   //      actuator_home_position = left_arm_home_position;
   //      actuator_joint_names = {"LShoulderPitch", "LShoulderRoll", "LElbowRoll", "LElbowYaw", "LWristYaw"};
   //      actuator_client = create_client(actuator_topic);
   //      if(actuator_client == NULL){
   //          return 0;
   //      }
   //      number_of_joints = actuator_joint_names.size();

   //      hand = "LHand";
   //      if(extract_topic(hand, topics_filename, &hand_topic)){
   //          return 0;
   //      }
   //  }
    if(actuator == "Leg"){                         // Leg
      //   actuator_state = leg_joint_states;
        actuator_home_position = leg_home_position;
        actuator_joint_names = {"HipPitch", "HipRoll", "KneePitch"};
        actuator_client = create_client(actuator_topic);
        if(actuator_client == NULL){
            return 0;
        }
        number_of_joints = actuator_joint_names.size();
    }
    else if(actuator == "Head"){                        // Head
      //   actuator_state = head_joint_states;
        actuator_home_position = head_home_position;
        actuator_joint_names = {"HeadPitch", "HeadYaw"};
        actuator_client = create_client(actuator_topic);
        if(actuator_client == NULL){
            return 0;
        }
        number_of_joints = actuator_joint_names.size();
    }

    // Vectors to store the trajectory information (positions, velocities, accelerations and durations)
    std::vector<std::vector<double>> positions_t;
    std::vector<std::vector<double>> velocities_t;
    std::vector<std::vector<double>> accelerations_t;
    std::vector<double> duration_t;

   //  // Compute the trajectory to get to the home position
   //  compute_trajectory(actuator_state, actuator_home_position, actuator_state.size(), home_duration, positions_t, velocities_t, accelerations_t, duration_t);

    // Move the joints of the actuator depending on the interpolation means selected                                                  // Linear interpolation
   move_to_position(actuator_client, actuator_joint_names, home_duration, open_hand, hand, hand_topic, "home", actuator_home_position);

    return 1;


}

/*********************************************************************************

setOdometryPose

Initialize the pose returned by the callback that services the subscription to the odom topic         
                                                                                                       
Odometry provides relative position orientation. Since we can't assume what odometry data is published
on the odom topic on start up, we use two extra sets of variables for the x, y, and theta values:     
adjustment variables and current variables.                                                           
                                                                                                      
We set the values of the adjustment variables to be the difference between                            

(a) the values associated with the start pose, and                                                    
(b) the values published on the odom topic on start up (or whenever we reinitialize the odometry),    
                                                                                                      
The callback then sets the values of the current variables as follows.

- the current x and y values are set to the sum of the adjustment x and y values and the odom x and y values 
  (this effectively translates the odom x and y values by the adjustment x and y values) 

- these translated values are then rotated about the Z axis by an angle equal to the difference 
  between the start theta value  and the odom theta value

- the current theta value is set to be the sum of the adjustment theta value and the odom theta value                                                            

**********************************************************************************/

void setOdometryPose(float x, float y, float theta)
{

   bool debug = true;

   sleep(1); // allow time for messages to be published on the odom topic
   ros::spinOnce();

   /* store the initial pose */
   initial_x = x;
   initial_y = y;
   initial_theta = theta;

   /* calculate the adjustment to the pose, i.e. the difference between the initial pose and odometry pose */
   adjustment_x = x - odom_x;
   adjustment_y = y - odom_y;
   adjustment_theta = theta - odom_theta;

   sleep(1); // allow time for adjusted  messages to be published on the odom topic
   ros::spinOnce();

   if (debug)
   {
      printf("setOdometryPose: odom_x,y,theta %f %f %f  adjustment_x, y, theta %f %f %f\n", odom_x, odom_y, odom_theta,
             adjustment_x, adjustment_y, adjustment_theta);
   }
}

int move_robot(float start_x, float start_y, float start_theta, float goal_x, float goal_y, float goal_theta, ros::Publisher velocity_publisher, ros::Rate rate, bool debug)
{

   float                x_waypoint;
   float                y_waypoint;
   float                theta_waypoint;
    /* Path planning is done, let's navigate */
   /* ====================================  */

   // If there is no physical robot, insert code to publish to simulator robot.
   // For now, it just exits the program

   if (locomotionParameterData.robot_available == false)
   {
      printf("No physical robot present\n");
      return 0;
   }

   // // Display the waypoints coordinates in image frame of reference
   // printf("Number of points is %d\n", waypoints.numberOfPoints);
   // for (int p = 0; p < waypoints.numberOfPoints + 1; p++) {
   //    printf("Waypoint selected  %d %d %5.3f\n", waypoints.point[p].x, waypoints.point[p].y, waypoints.point[i].theta);
   // }


   /* initialize the odometry */
   // ros::spinOnce();
   // sleep(1); // allow time for messages to be published on the odom topic
   
   setOdometryPose(start_x, start_y, start_theta);
   // ros::spin();
   
   /* 
      Compute the waypoints in robot frame of reference and navigate using MIMO
      Set waypointFlag to true so as to ensure that robot doesn't stop at waypoints and not worry about goal orientation yet
    */
   waypointFlag = true;
   x_waypoint = start_x;
   y_waypoint = start_y;
   theta_waypoint = start_theta;
   printf("Valid waypoint = %.3f, %.3f, %.3f\n", x_waypoint, y_waypoint, theta_waypoint);
   goToPoseDQ(x_waypoint, y_waypoint, theta_waypoint, locomotionParameterData, velocity_publisher, rate);
   // goToPoseMIMO(x_waypoint, y_waypoint, theta_waypoint, locomotionParameterData, velocity_publisher, rate, waypointFlag);

   x_waypoint = (float) waypoints.point[0].y / 100;
   y_waypoint = (float) (y_map_size - waypoints.point[0].x)/100;
   theta_waypoint = atan2((y_waypoint - start_y),(x_waypoint -  start_x));
   printf("Valid waypoint = %.3f, %.3f, %.3f\n", x_waypoint, y_waypoint, theta_waypoint);
   goToPoseDQ(x_waypoint, y_waypoint, theta_waypoint, locomotionParameterData, velocity_publisher, rate);
   // goToPoseMIMO(x_waypoint, y_waypoint, theta_waypoint, locomotionParameterData, velocity_publisher, rate, waypointFlag);
   for(int k = 1; k < waypoints.numberOfPoints + 1; k++)
   //for(int k = 1; k < 10; k+=2)
   {
      x_waypoint = (float) waypoints.point[k].y / 100;
      y_waypoint = (float) (y_map_size - waypoints.point[k].x)/100;
      float x_1_waypoint = (float) waypoints.point[k-1].y / 100;
      float y_1_waypoint = (float) (y_map_size - waypoints.point[k-1].x)/100;
      float x1_waypoint = (float) waypoints.point[k+1].y / 100;
      float y1_waypoint = (float) (y_map_size - waypoints.point[k+1].x)/100;
      if(x_waypoint == x1_waypoint || y_waypoint == y1_waypoint)
      {
         continue;
      }
      theta_waypoint = ((float) atan2((y_waypoint - y_1_waypoint),(x_waypoint -  x_1_waypoint)));
      //if(theta_waypoint < 0)
        // y_waypoint = y_waypoint - 0.1;
      //else
        // y_waypoint = y_waypoint + 0.15;
      if (debug)
         printf("Valid waypoint = %.3f, %.3f, %.3f\n", x_waypoint, y_waypoint, theta_waypoint);

      // goToPoseDQ(x_waypoint, y_waypoint, theta_waypoint, locomotionParameterData, velocity_publisher, rate);
      goToPoseMIMO(x_waypoint, y_waypoint, theta_waypoint, locomotionParameterData, velocity_publisher, rate, waypointFlag);
   }
   // Robot arrived at destination (or one waypoint before destination)
   // Then navigate to the final goal pose
   // Set waypointFlag to false to enable robot stop at final destination and correct final orientation to the goal orientation
   waypointFlag = false;

   // goToPoseMIMO(goal_x, goal_y, goal_theta, locomotionParameterData, velocity_publisher, rate, waypointFlag);
   goToPoseDQ(goal_x, goal_y, goal_theta, locomotionParameterData, velocity_publisher, rate);

}

void save_waypoint_map(vector<int> compressionParams, Mat mapImageLarge){
    char path_and_input_filename[MAX_FILENAME_LENGTH]        = "";
    std::string navigation_pathway_filename = "waypointMap.png";  // name of the output pathway image

    /* Combine the name for the output pathway image*/
    strcpy(path_and_input_filename, packagedir.c_str());  
    strcat(path_and_input_filename, "/data/"); 
    strcat(path_and_input_filename, navigation_pathway_filename.c_str());

    imwrite(path_and_input_filename, mapImageLarge, compressionParams);   // write the image to a file
    printf("Navigation pathway image is saved in %s\n", path_and_input_filename);
    //imshow(mapWindowName, mapImageLarge); // Display map image
}

/******************************************************************************

goToPoseDQ

Use the divide and conquer algorithm to drive the robot to a given pose

*******************************************************************************/

void goToPoseDQ(float x, float y, float theta, locomotionParameterDataType locomotionParameterData, ros::Publisher pub, ros::Rate rate)
{

   bool debug = false;

   geometry_msgs::Twist msg;

   float start_x;
   float start_y;
   float start_theta;

   float goal_x;
   float goal_y;
   float goal_theta;

   float goal_direction;

   float position_error;
   float angle_error;

   float angular_velocity;
   float linear_velocity;
   float current_linear_velocity = 0;

   int number_of_ramp_up_steps = 20;

   int mode; // GOING or ORIENTING

   goal_x = x;
   goal_y = y;
   goal_theta = theta;

   mode = ORIENTING; // divide and conquer always starts by adjusing the heading

   do
   {

      /* get the current pose */

      ros::spinOnce(); // Let ROS take over to handle the callback and publish the pose on the odom topic

      position_error = sqrt((goal_x - current_x) * (goal_x - current_x) +
                            (goal_y - current_y) * (goal_y - current_y));

      goal_direction = atan2((goal_y - current_y), (goal_x - current_x));
      // printf("Current theta %.3f\n", current_theta);
      angle_error = goal_direction - current_theta;

      /* The absolute error in direction can not be greater than Pi because the robot can rotate in two directions,             */
      /* positive and negative. Thus it can eliminate any angular error by rotating Pi radians or less in the correct direction */
      /* If the difference in directions is greater than +Pi, subtract  2 Pi; if it is less than -Pi, add 2 Pi                  */

      if (angle_error > PI)
      {
         angle_error = angle_error - 2 * PI;
      }
      else if (angle_error < -PI)
      {
         angle_error = angle_error + 2 * PI;
      }

      // if (fabs(angle_error) > locomotionParameterData.angle_tolerance_orienting) {
      if (((mode == ORIENTING) && (fabs(angle_error) > locomotionParameterData.angle_tolerance_orienting)) || // low angular tolerance when orienting to get the best initial heading
          ((mode == GOING) && (fabs(angle_error) > locomotionParameterData.angle_tolerance_going)))
      { // high angular tolerance when going so we don't have to correct the heading too often

         /* if the robot is not oriented correctly, adjust the heading */

         if (debug)
            printf("Orienting\n");

         mode = ORIENTING; // reset mode from GOING to ORIENTING to ensure we use the lower angular tolerance when reorienting

         /* set linear and angular velocities, taking care not to use values that exceed maximum values */
         /* or use values that are less than minimum values needed to produce a response in the robot   */

         msg.linear.x = 0;

         angular_velocity = locomotionParameterData.angle_gain_dq * angle_error;

         if (fabs(angular_velocity) < locomotionParameterData.min_angular_velocity)
            msg.angular.z = locomotionParameterData.min_angular_velocity * signnum(angular_velocity);
         else if (fabs(angular_velocity) > locomotionParameterData.max_angular_velocity)
            msg.angular.z = locomotionParameterData.max_angular_velocity * signnum(angular_velocity);
         else
            msg.angular.z = angular_velocity;
      }
      else if (position_error > locomotionParameterData.position_tolerance)
      {

         mode = GOING;

         /* if the robot has not reached the goal, adjust the distance */

         if (debug)
            printf("Going\n");

         /* set linear and angular velocities, taking care not to use values that exceed maximum values */
         /* or use values that are less than minimum values needed to produce a response in the robot   */

         linear_velocity = locomotionParameterData.position_gain_dq * position_error;

         if (linear_velocity < locomotionParameterData.min_linear_velocity)
            linear_velocity = locomotionParameterData.min_linear_velocity;
         else if (linear_velocity > locomotionParameterData.max_linear_velocity)
            linear_velocity = locomotionParameterData.max_linear_velocity;

         /* if stopped, ramp up to the required velocity ... don't attempt an infinite acceleration to the required velocity */

         if (current_linear_velocity == 0)
         {

            for (int i = 1; i < number_of_ramp_up_steps; i++)
            {
                msg.linear.x = (float)linear_velocity * ((float)i / (float)number_of_ramp_up_steps);
                msg.angular.z = 0;

                if (debug)
                {
         printf("Ramping up velocity\n");
         // printf("Current pose:         %5.3f %5.3f %5.3f\n", current_x, current_y, current_theta);
         // printf("Goal, heading, theta: %5.3f, %5.3f, %5.3f\n", goal_theta, goal_direction, current_theta);
         printf("Error:                %5.3f, %5.3f\n", position_error, angle_error);
         printf("velocity command:     %5.3f, %5.3f\n", msg.linear.x, msg.angular.z);
                }

                pub.publish(msg); // Publish the message

                rate.sleep(); // Wait until it's time for another iteration
            }
            current_linear_velocity = linear_velocity;
         }

         msg.linear.x = linear_velocity;
         msg.angular.z = 0;
      }

      if (debug)
      {
         // printf("Current pose:         %5.3f %5.3f %5.3f\n", current_x, current_y, current_theta);
         // printf("Goal, heading, theta: %5.3f, %5.3f, %5.3f\n", goal_theta, goal_direction, current_theta);
         printf("Error:                %5.3f, %5.3f\n", position_error, angle_error);
         printf("velocity command:     %5.3f, %5.3f\n\n", msg.linear.x, msg.angular.z);
      }

      pub.publish(msg); // Publish the message

      rate.sleep(); // Wait until it's time for another iteration

   } while ((position_error > locomotionParameterData.position_tolerance) && ros::ok());

   /* the robot has reached the destination so             */
   /* adjust the orientation to match the goal orientation */

   do
   {

      if (debug)
         printf("Orienting\n");

      /* get the current pose */

      ros::spinOnce(); // Let ROS take over to handle the callback

      angle_error = goal_theta - current_theta;

      /* The absolute error in direction can not be greater than Pi because the robot can rotate in two directions,            */
      /* positive and negative. Thus it can eliminate any angular error by rotating Pi radian or less in the correct direction */
      /* If the difference in directions is greater than +Pi, subtract  2 Pi; if it is less than -Pi, add 2 Pi                 */

      if (angle_error > PI)
      {
         angle_error = angle_error - 2 * PI;
      }
      else if (angle_error < -PI)
      {
         angle_error = angle_error + 2 * PI;
      }

      msg.linear.x = 0;

      /* set linear and angular velocities, taking care not to use values that exceed maximum values */
      /* or use values that are less than minimum values needed to produce a response in the robot   */

      angular_velocity = locomotionParameterData.angle_gain_dq * angle_error;

      if (fabs(angular_velocity) < locomotionParameterData.min_angular_velocity)
         msg.angular.z = locomotionParameterData.min_angular_velocity * signnum(angular_velocity);
      else if (fabs(angular_velocity) > locomotionParameterData.max_angular_velocity)
         msg.angular.z = locomotionParameterData.max_angular_velocity * signnum(angular_velocity);
      else
         msg.angular.z = angular_velocity;

      if (debug)
      {
         printf("Orienting\n");
         // printf("Current pose:         %5.3f %5.3f %5.3f\n", current_x, current_y, current_theta);
         // printf("Goal, heading, theta: %5.3f, %5.3f, %5.3f\n", goal_theta, goal_direction, current_theta);
         printf("Error:                %5.3f, %5.3f\n", position_error, angle_error);
         printf("velocity command:     %5.3f, %5.3f\n\n", msg.linear.x, msg.angular.z);
      }

      if((fabs(angle_error) > locomotionParameterData.angle_tolerance_orienting)){

         pub.publish(msg); // Publish the message

         rate.sleep(); // Wait until it's time for another iteration
      }

   } while ((fabs(angle_error) > locomotionParameterData.angle_tolerance_orienting) && ros::ok());

   msg.linear.x = 0;
   msg.angular.z = 0;

   pub.publish(msg); // Publish the message
   rate.sleep(); // Wait until it's time for another iteration


}

/**********************************************************************************************************************

goToPoseMIMO

Use the MIMO algorithm to drive the robot to a given position and then adjust orientation to achieve the required  pose

***********************************************************************************************************************/

 void goToPoseMIMO(float x, float y, float theta, locomotionParameterDataType locomotionParameterData, ros::Publisher pub, ros::Rate rate, bool waypoints)
 {

   bool debug = false;

   geometry_msgs::Twist msg;

   float start_x;
   float start_y;
   float start_theta;

   float goal_x;
   float goal_y;
   float goal_theta;

   float goal_direction;

   float position_error;
   float angle_error;

   float angular_velocity;
   float linear_velocity;

   static float current_linear_velocity = 0; // make this static so that it's valid on the next call

   int number_of_ramp_up_steps = 20;

   goal_x = x;
   goal_y = y;
   goal_theta = theta;

   do
   {

     /* get the current pose */

     ros::spinOnce(); // Let ROS take over to handle the callback

     position_error = sqrt((goal_x - current_x) * (goal_x - current_x) +
                           (goal_y - current_y) * (goal_y - current_y));

     goal_direction = atan2((goal_y - current_y), (goal_x - current_x));
     angle_error = goal_direction - current_theta;

     /* The absolute error in direction can not be greater than Pi because the robot can rotate in two directions,            */
     /* positive and negative. Thus it can eliminate any angular error by rotating Pi radian or less in the correct direction */
     /* If the difference in directions is greater than +Pi, subtract  2 Pi; if it is less than -Pi, add 2 Pi                 */

     if (angle_error > PI)
     {
         angle_error = angle_error - 2 * PI;
     }
     else if (angle_error < -PI)
     {
         angle_error = angle_error + 2 * PI;
     }

     /* set linear and angular velocities, taking care not to use values that exceed maximum values */
     /* or use values that are less than minimum values needed to produce a response in the robot   */

     angular_velocity = locomotionParameterData.angle_gain_mimo * angle_error;

     if (fabs(angular_velocity) < locomotionParameterData.min_angular_velocity)
         msg.angular.z = locomotionParameterData.min_angular_velocity * signnum(angular_velocity);
     else if (fabs(angular_velocity) > locomotionParameterData.max_angular_velocity)
         msg.angular.z = locomotionParameterData.max_angular_velocity * signnum(angular_velocity);
     else
         msg.angular.z = angular_velocity;

     /* don't slow down if driving to a waypoint */

     if (waypoints)
     {
         linear_velocity = locomotionParameterData.max_linear_velocity / 2;
     }
     else
     {
         linear_velocity = locomotionParameterData.position_gain_mimo * position_error;
     }

     if (linear_velocity < locomotionParameterData.min_linear_velocity)
         msg.linear.x = locomotionParameterData.min_linear_velocity;
     else if (linear_velocity > locomotionParameterData.max_linear_velocity)
         msg.linear.x = locomotionParameterData.max_linear_velocity;

     /* if currently stopped, ramp up to the required velocity ... don't attempt an infinite acceleration to the required velocity */

     if (current_linear_velocity == 0)
     {

         for (int i = 1; i < number_of_ramp_up_steps; i++)
         {
      msg.linear.x = (float)linear_velocity * ((float)i / (float)number_of_ramp_up_steps);
      msg.angular.z = 0;

      if (debug)
      {
              printf("Ramping up velocity\n");
              // printf("Current pose:         %5.3f %5.3f %5.3f\n", current_x, current_y, current_theta);
              // printf("Goal, heading, theta: %5.3f, %5.3f, %5.3f\n", goal_theta, goal_direction, current_theta);
              printf("Error:                %5.3f, %5.3f\n", position_error, angle_error);
              printf("velocity command:     %5.3f, %5.3f\n\n", msg.linear.x, msg.angular.z);
      }

      pub.publish(msg); // Publish the message

      rate.sleep(); // Wait until it's time for another iteration
         }
         current_linear_velocity = linear_velocity;
     }

     msg.linear.x = linear_velocity;

     if (debug)
     {
         printf("Going\n");
         // printf("Current pose:         %5.3f %5.3f %5.3f\n", current_x, current_y, current_theta);
         // printf("Goal, heading, theta: %5.3f, %5.3f, %5.3f\n", goal_theta, goal_direction, current_theta);
         printf("Error:                %5.3f, %5.3f\n", position_error, angle_error);
         printf("velocity command:     %5.3f, %5.3f\n\n", msg.linear.x, msg.angular.z);
     }

     pub.publish(msg); // Publish the message

     rate.sleep(); // Wait until it's time for another iteration

   } while ((fabs(position_error) > locomotionParameterData.position_tolerance) && ros::ok());

   /* for the final destination, adjust the orientation to match the goal pose */
   /* ------------------------------------------------------------------------ */

   if (!waypoints)
   {

     do
     {

         /* if the robot has reached the destination             */
         /* adjust the orientation to match the goal orientation */

         /* get the current pose */

         ros::spinOnce(); // Let ROS take over to handle the callback

         /* set linear and angular velocities, taking care not to use values that exceed maximum values */
         /* or use values that are less than minimum values needed to produce a response in the robot   */

         angle_error = goal_theta - current_theta;

         current_linear_velocity = 0;
         msg.linear.x = current_linear_velocity;

         angular_velocity = locomotionParameterData.angle_gain_mimo * angle_error;

         if (fabs(angular_velocity) < locomotionParameterData.min_angular_velocity)
      msg.angular.z = locomotionParameterData.min_angular_velocity * signnum(angular_velocity);
         else if (fabs(angular_velocity) > locomotionParameterData.max_angular_velocity)
      msg.angular.z = locomotionParameterData.max_angular_velocity * signnum(angular_velocity);
         else
      msg.angular.z = angular_velocity;

         if (debug)
         {
      printf("Orienting\n");
      // printf("Current pose:         %5.3f %5.3f %5.3f\n", current_x, current_y, current_theta);
      // printf("Goal, heading, theta: %5.3f, %5.3f, %5.3f\n", goal_theta, goal_direction, current_theta);
      printf("Error:                %5.3f, %5.3f\n", position_error, angle_error);
      printf("velocity command:     %5.3f, %5.3f\n\n", msg.linear.x, msg.angular.z);
         }

         pub.publish(msg); // Publish the message

         rate.sleep(); // Wait until it's time for another iteration

     } while ((fabs(angle_error) > locomotionParameterData.angle_tolerance_orienting) && ros::ok());

      current_linear_velocity = 0;
      msg.linear.x = current_linear_velocity;

      angular_velocity = 0;
      msg.angular.z = angular_velocity;

      pub.publish(msg); // Publish the message

      rate.sleep(); // Wait until it's time for another iteration

   }
 }

 /* 
 *   Function to convert radians to degrees
 *   This function converts the angle in radians to degrees
 *
 * @param:
 *   radians: the angle in radians
 *
 * @return:
 *   the angle in degrees
 */
float degrees(float radians)
{
    float degrees = radians * (float) 180.0 / (float) M_PI; // David Vernon ... cast to float
    return degrees;
}

/* 
 *   Function to convert degrees to radians
 *   This function converts the angle in degrees to radians
 *
 * @param:
 *   degrees: the angle in degrees
 *
 * @return:
 *   the angle in radians
 */
float radians(float degrees)
{
    float radians = degrees / ((float) 180.0 / (float) M_PI); // David Vernon ... cast to float
    return radians;
}


/*  
 *   Function to prompt the user to press any key to exit the program
 *
 *   @param:
 *       status: the status of the program
 *
 *   @return:
 *       None
 */
void prompt_and_exit(int status){
    printf("Press any key to exit ... \n");
    getchar();
    exit(status);
}

/*  
 *   Function to prompt the user to press any key to continue or press X to exit the program
 *
 *   @param:
 *       None
 *
 *   @return:
 *       None
 */
void prompt_and_continue(){
    printf("Press X to quit or Press any key to continue...\n");
    char got_char = getchar();
    if ((got_char == 'X') || (got_char == 'x')){
        printf("Exiting ...\n");
       exit(0);
    }
}