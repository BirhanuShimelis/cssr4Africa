<div align="center">
  <img src="CSSR4AfricaLogo.svg" alt="CSSR4Africa Logo" style="width:50%; height:auto;">
</div>

<div align="center">
  <h1>Robot Navigation Node Usage Guide</h1>
</div>




This guide provides concise instructions on how to:

- Install dependencies
- Build the package
- Run the node
- Echo the pose topic
- Use the reset pose service
- Send a goal

---

## Prerequisites

- ROS Installed: Ensure ROS is installed.
- Catkin Workspace: Place the `robotNavigation` package in the catkin workspace at `~/workspace/pepper_rob_ws/src/cssr4Africa/cssr_system/robotNavigation`.

---

## Package Location

The `robotNavigation` package should be located in:

```
~/workspace/pepper_rob_ws/src/cssr4Africa/cssr_system/robotNavigation
```

---

## Step 1: Install Dependencies

Navigate to the catkin workspace and install dependencies:

```bash
cd ~/workspace/pepper_rob_ws
rosdep install --from-paths src --ignore-src -r -y
```

---

## Step 2: Build the Package

Build the workspace:

```bash
br@br:~/workspace/pepper_rob_ws$ catkin_make
br@br:~/workspace/pepper_rob_ws$ source devel/setup.bash
```

---

## Step 3: Run the Node

Start the `robotNavigation` node:

```bash
br@br:~/workspace/pepper_rob_ws$ rosrun cssr_system robotNavigation
```

---

## Step 4: Echo the Pose Topic

View the pose data assuming robot localization is running and publishing the current pose:

```bash
br@br:~/workspace/pepper_rob_ws$ rostopic echo /robotNavigation/pose
```

---

## Step 5: Use the Reset Pose Service

Reset the robot's pose:

```bash
br@br:~/workspace/pepper_rob_ws$ rosservice call /robotLocalization/reset_pose 2.0 6.6 0.0
```

---

## Step 6: Send a Goal

Send a goal to the robot:

```bash
br@br:~/workspace/pepper_rob_ws$ rosservice call /robotNavigation/set_goal 2.0 6.6 0.0
```


