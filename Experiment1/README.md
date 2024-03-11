# RE510

# Title : Manipulator Teleoperation

Learn the basic concept of the manipulator teleoperation.
Implement a simple teleoperation system for the 7-DoF manipulator.

---
### Experiment details and scoring 

1. Implementing Teleoperation system with provided simulation setup (40%)
  1-1. Master-Slave coordinate mapping
    Since coordinate of slave robot is different from master’s, slave’s end-effector will move to the different direction with your master commands in default setting. 
    Human operator will control the master, with camera image information.
    Therefore, the correct mapping between master and slave is required. (Check sample video in p.16)
  1-2. Position-to-Position control
    Increasing the workspace of slave robot by using scaling and indexing methods.
  1-3. Position-to-Velocity control

2. Performing the teleoperation task with your implemented system (20%)
  2-1. Task: Drop cans from the table by pushing it with slave robot.
  2-2. Perform this task with each control methods(P-to-P, P-to-V) in each configuration (EIH, ETH) and record videos
    Use screen recorder program, such as ‘Kazam’,  ‘SimpleScreenRecorder’, and etc..

3. Report (40%)
  3-1. Submit source codes, videos and report.


---
### Source codes

You are only allowed to modify 
master_controller_node.cpp (EIH, ETH)
slave_controller_node.cpp (EIH, ETH)
franka_kinematics_solver.cpp

---
#### Running simulation systems (EIH Config.)
Launch Gazebo Simulator
	roslaunch gazebo_launch franka_gazebo_EIH.launch
	
Launch Teleoperation system (It includes Rviz)
	roslaunch teleoperation teleoperation_EIH.launch

Launch Virtual master controller
	roslaunch virtual_master_device virtual_master_device.launch

Launch Slave franka manipulator controller
	roslaunch franka_controller franka_controller.launch


#### Running simulation systems (ETH Config.)
Launch Gazebo Simulator
	roslaunch gazebo_launch franka_gazebo_ETH.launch
	
Launch Teleoperation system (It includes Rviz)
	roslaunch teleoperation teleoperation_ETH.launch

Launch Virtual master controller
	roslaunch virtual_master_device virtual_master_device.launch

Launch Slave franka manipulator controller
	roslaunch franka_controller franka_controller.launch


