
<div align="center">
  <h1>Example Component Unit Test</h1>
</div>

<div align="center">
  <img src="CSSR4AfricaLogo.svg" alt="CSSR4Africa Logo" style="width:50%; height:auto;">
</div>

This module provides unit tests for the `exampleComponent` node within the CSSR4Africa project (`cssr_system` package). --Detailed Overview --. The results are logged in the file `~/workspace/pepper_rob_ws/src/unit_tests/exampleComponentTest/data/exampleComponentTestOutput.dat` for the physical robot and `~/workspace/pepper_sim_ws/src/unit_tests/exampleComponentTest/data/exampleComponentTestOutput.dat` for the simulator robot.

# Documentation
Accompanying this code is the deliverable report that provides a detailed explanation of the code and how to run the tests. The deliverable report can be found in [DX.Y Example COmponent](https://cssr4africa.github.io/deliverables/CSSR4Africa_Deliverable_D3.3.pdf).

# Run the Animate Behaviour Unit Test 
## Physical Robot 
### Steps
1. **Install the required software components:**

  Install the required software components to instantiate and set up the development environment for controlling the Pepper robot in both physical and simulated environments. Use the [CSSR4Africa Software Installation Manual](https://github.com/cssr4africa/cssr4africa/blob/main/docs/D3.3_Software_Installation_Manual.pdf). 

2. **Clone and build the project (if not already cloned)**:
   - Move to the source directory of the workspace
      ```bash 
         cd $HOME/workspace/pepper_rob_ws/src
       ```
   - Clone the `CSSR4Africa` software from the GitHub repository
      ```bash 
         git clone https://github.com/cssr4africa/cssr4africa.git
       ```
   - Build the source files
      ```bash 
         cd .. && source devel/setup.bash && catkin_make
       ```
       
3. **Update Configuration File:**
   
   Navigate to `~/workspace/pepper_rob_ws/src/unit_tests/exampleComponentTest/config/exampleComponentTestConfiguration.ini` and update the configuration according to the key-value pairs below:

   | Parameter | Description | Values |
   |-----------|-------------|---------|
   | `keyA` | Target platform | `valueA_1` or `valueA_2` |
   | `keyB` | Test iconic gestures | `valueB_1`, `valueB_2` |


   - To execute the gestures on the physical platform, change the first line of `exampleComponentTestConfiguration.ini` file in the config folder to “`platform robot`”. 
   - Any other important description
  

    <div style="background-color: #1e1e1e; padding: 15px; border-radius: 4px; border: 1px solid #404040; margin: 10px 0;">
      <span style="color: #ff3333; font-weight: bold;">NOTE: </span>
      <span style="color: #cccccc;">If you want to modify other configuration values, please refer to the <a href="https://github.com/cssr4africa/cssr4africa/blob/main/docs/DX.Y_Example_Component.pdf" style="color: #66b3ff;">DX.Y Example Component</a>. Otherwise, the preferred values are the ones already set in the `exampleComponentTestConfiguration.ini` file.</span>
  </div>

4. **Run the `exampleComponentTest` from the`unit_tests`  package**. 

    Follow below steps, run in different terminals.
    -  Source the workspace in first terminal:
        ```bash
          cd $HOME/workspace/pepper_rob_ws && source devel/setup.bash
        ```
    -  Launch the robot:
        ```bash
          roslaunch unit_tests exampleComponentLaunchRobot.launch robot_ip:=<robot_ip> roscore_ip:=<roscore_ip> network_interface:=<network_interface>
        ```
        <div style="background-color: #1e1e1e; padding: 15px; border-radius: 4px; border: 1px solid #404040; margin: 10px 0;">
         <span style="color: #ff3333; font-weight: bold;">NOTE: </span>
         <span style="color: #cccccc;">Ensure that the IP addresses <code>robot_ip</code> and <code>roscore_ip</code> and the network interface <code>network_interface</code> are correctly set based on your robot's configuration and your computer's network interface. </span>
        </div>
    - Open a new terminal to launch the exampleComponentTest (which launches the exampleComponent node and run tests on it). This creates a driver for the `/sample/topic` topic and a stub for the `/sample/service` service.
        ```bash
          cd $HOME/workspace/pepper_rob_ws && source devel/setup.bash && roslaunch unit_tests exampleComponentLaunchTestHarness.launch
        ```

## Simulator Robot
### Steps
1. **Install the required software components:**

  Install the required software components to instantiate and set up the development environment for controlling the Pepper robot in both physical and simulated environments. Use the [CSSR4Africa Software Installation Manual](https://github.com/cssr4africa/cssr4africa/blob/main/docs/D3.3_Software_Installation_Manual.pdf). 


2. **Clone and build the project (if not already cloned)**:
   - Move to the source directory of the workspace
      ```bash 
         cd $HOME/workspace/pepper_sim_ws/src
       ```
   - Clone the `CSSR4Africa` software from the GitHub repository
      ```bash 
         git clone https://github.com/cssr4africa/cssr4africa.git
       ```
   - Build the source files
      ```bash 
         cd .. && source devel/setup.bash && catkin_make
       ```
       
3. **Update Configuration File:**
   
   Navigate to `~/workspace/pepper_sim_ws/src/unit_tests/exampleComponent/config/exampleComponentConfiguration.ini` and update the configuration according to the key-value pairs below:

   | Parameter | Description | Values |
   |-----------|-------------|---------|
   | `keyA` | Target platform | `valueA_1` or `valueA_2` |
   | `keyB` | Test iconic gestures | `valueB_1`, `valueB_2` |

   - To execute the gestures on the physical platform, change the first line of `exampleComponentConfiguration.ini` file in the config folder to “`platform robot`”. 
   - Any other important description
  

    <div style="background-color: #1e1e1e; padding: 15px; border-radius: 4px; border: 1px solid #404040; margin: 10px 0;">
      <span style="color: #ff3333; font-weight: bold;">NOTE: </span>
      <span style="color: #cccccc;">If you want to modify other configuration values, please refer to the <a href="https://github.com/cssr4africa/cssr4africa/blob/main/docs/DX.Y_Example_Component.pdf" style="color: #66b3ff;">DX.Y Example Component</a>. Otherwise, the preferred values are the ones already set in the `exampleComponentTestConfiguration.ini` file.</span>
  </div>

4. **Run the `exampleComponent` from the `unit_tests`  package**. 

    Follow below steps, run in different terminals.
    -  Source the workspace in first terminal:
        ```bash
          cd $HOME/workspace/pepper_sim_ws && source devel/setup.bash
        ```
    -  Launch the robot:
        ```bash
          roslaunch unit_tests gestureExecutionLaunchSimulator.launch
        ```

    - Open a new terminal to launch the exampleComponent (which launches the gestureExecution node and run tests on it). This creates a driver for the `/sample/topic` topic and a stub for the `/sample/service` service.
        ```bash
          cd $HOME/workspace/pepper_sim_ws && source devel/setup.bash && roslaunch unit_tests animateBehaviourLaunchTestHarness.launch
        ```

## Tests Executed
### Test A
The test A contains the following:
  - `Test A_1`
  - `Test A_2`

The robot is expected to perform the following.

### Test B
The Test B contains the following:
  - `Test B_1`
  - `Test B_2`

The robot is expected to perform the following.

### Test C
The Test C contains the following:
  - `Test C_1`
  - `Test C_2`

The robot is expected to perform the following.

## Results
The results of the test is logged in the `~/workspace/pepper_rob_ws/src/unit_tests/exampleComponent/data/exampleComponentOutput.dat` file for the physical robot and `~/workspace/pepper_sim_ws/src/unit_tests/exampleComponent/data/exampleComponentOutput.dat` file for the simulator robot. It contains the test ran, the input commands and the satus of the test. Below is the output of the test when some smaple configuration is set
```
Example Component Test Report: robot
======================================
Date: 2024-12-06 14:03:22

Test A
	Input		  : A
	Result        : PASSED

Test B
	Input		  : B
	Result        : PASSED

Test C
	Input		  : C
	Result        : PASSED
```


## 
<div style="background-color: #1e1e1e; padding: 15px; border-radius: 4px; border: 1px solid #404040; margin: 10px 0;">
      <span style="color: #ff3333; font-weight: bold;">NOTE: </span>
      <span style="color: #cccccc;">To fully understand the configuration values, data requirements, debugging processes, and the overall functionality of the exampleComponentTest node, please refer to the <a href="https://github.com/cssr4africa/cssr4africa/blob/main/docs/DX.Y_Example_Component.pdf" style="color: #66b3ff;">DX.Y Example Component.pdf</a>. These manuals provide comprehensive explanations and step-by-step instructions essential for effective use and troubleshooting.</span>
  </div>
  
## Support

For issues or questions:
- Create an issue on GitHub
- Contact: <a href="mailto:dvernon@andrew.cmu.edu">dvernon@andrew.cmu.edu</a>, <a href="mailto:email@andrew.cmu.edu">email@andrew.cmu.edu</a><br>
- Visit: <a href="http://www.cssr4africa.org">www.cssr4africa.org</a>




## License  
Funded by African Engineering and Technology Network (Afretec)  
Inclusive Digital Transformation Research Grant Programme

Date:   2024-12-10