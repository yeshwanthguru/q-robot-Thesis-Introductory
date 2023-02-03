 # Cognitive Architectures for Robots inspired by Quantum Computing (Pick and place classical system of Tiago robot Triggered by the quantum like robot perception)
 
 
 ![image](https://user-images.githubusercontent.com/72270080/216659306-8ca09f34-2624-4f28-9c65-368a33a74d15.png)  **![image](https://user-images.githubusercontent.com/72270080/216660262-a1bd8fca-5fcf-4a4d-bbf1-6fbe9f10fa28.png)**  ![image](https://user-images.githubusercontent.com/72270080/216659912-c5729aa5-2835-4f33-9b84-03f6d4fce050.png)


## ABSTRACT:
This project is to use and eventually extend the quantum-robot framework to Develop Cognitive Architecture for robots inspired by quantum Computing. And the project is to implement on Tiago++ robot with a simple Handover task(1.0) hardware and software.

## INTRODUCTION:
This whole project scene is developed in the Gazebo simulation environment, which was simulated based on the nodes developed using ROS MELODIC(ubuntu 18.04). The main aim of the project is to define a Software Architecture(ROS) to control the robot in gazebo simulation with a decision-making Quantum system and final testing with the real hardware. In particular, We have a "picking" classic system,which was (Tiago++ robot) . And we have a "decision making" quantum system, It encodes  some sensors (for this specific scenario three sensory inputs were encoded in tho the quantum state with the multiqubit encoded measuremnt ) a specific world configuration (encoded as a Hilbert state), and when that is measured it triggers the "picking" system or else not.Which can be stated as the **Quantum-classical interface(Quantum brain)** controller to speed up the classical system in the perception basis.

## THE ROBOT ARCHITECTURE AND ITS WORKING:
### Robot Working
In the simulation environment comprising a table and a box with an ArUco marker is defined. The robot then locates the object in the RGB of its camera and reconstructs its 3D pose.Then, MoveIt! is used in order to plan a pick trajectory to grasp the object, which is then lifted up
and finally a place trajectory is planned to restore the object in its former position. Gazebo will show up with TIAGo++ in front of a table and the object with the ArUco marker on its top.

https://user-images.githubusercontent.com/72270080/215219345-503e1c80-e754-4fe0-82af-3b52290d4e5e.mp4

And The Following setup has the nodes such as:  

• **/aruco_single**: ArUco marker detector node.  

• **/pick_and_place_server**: node in charge of defining the planning scene, request pick andplans with MoveIt! and execute them.  

•**/pick_client**: node that prepares the robot for the object detection and the pick and place operations: raises the arm to a safe pose and lowers the head to look at the table. Then it waits until the object marker is detected and its pose is retrieved in order to send a goal to the /pick_and_place_server.


•**/rviz**: in order to visualize all the steps involved in the demo.Rviz will also show up to help the user visualize the different steps involved in the
 
 On the ROS side we have Four Major nodes inside the ROS workspace and the Quantum Brain should be out side the ros environment with its working dependencies 
### QUANTUM-CLASSICAL-INTERFACE NODE WORKING(qrobot):
   This is the main context in the Quantum based Robot decision making system developed with the basis of [q-robot package](http://docs.quantum-robot.org/en/latest/),in which we should undergo the installation process following the above documentation.And this The is written in Python3.8 and implements a robot classical picking system with the help of multi-qubit quantum state decision making. The code use  the rospy library for Python ROS integration. The code defines a node called "Tiago_brain" and initializes it as an anonymous node.

The code sets up a number of classes and objects to represent the quantum system, including AngularModel, OneBurst, and QUnit. It then defines three callback functions: callback_function, pickup_callback, and place_callback, which are subscribed to ROS topics related to the sensory input of the robot (e.g. grasp poses, pickup actions, and place actions).

The main function tiago_classical_pickingsystem runs in a loop, checking if the ROS node is shutting down. If not, it subscribes to the relevant topics as uncertain data and measures the combined state of all three sensorial units. If the result of the measurement is greater than or equal to 0.5, the picking classical system is triggered. If the result is 0, the code waits for a decision. If there is no sensory connection, the code logs a warning.

## Installation process with ubuntu

Installing Ubuntu with ROS + TIAGO ++ supports only ROS melodic (ubuntu 18.04)

INSTALLATION PROCESS-http://wiki.ros.org/Robots/TIAGo%2B%2B/Tutorials/Installation/InstallUbuntuAndROS 

# CODE EXECUTION PROCESS :

### ROS SIDE 

Terminal1:
![Screenshot from 2023-01-29 02-58-56](https://user-images.githubusercontent.com/72270080/215300182-2cce5376-6335-4444-ad24-decbc4067eb8.png)
                              
                               roslaunch tiago_pick_demo pick_simulation.launch
once the above command executed wait till the command Arm tuck in the terminal where you executed the above code.Then execute the other command:
 
                                 roslaunch tiago_pick_demo pick_demo.launch 
                                 
![Screenshot from 2023-01-29 03-03-22](https://user-images.githubusercontent.com/72270080/215300331-68b19ad2-8913-439c-a223-14a4e872f380.png)
![Screenshot from 2023-01-29 03-03-39](https://user-images.githubusercontent.com/72270080/215300334-15b1934e-61fa-4609-b964-4703367ba654.png)
![Screenshot from 2023-01-29 03-03-43](https://user-images.githubusercontent.com/72270080/215300337-dcddbacd-7e33-4aea-aa38-b152cf7a7ce7.png)

### Non-ROS side:
Once everything ready run the code in the qrobot workspace.

Note that at each execution the selected plan will differ due to the random nature of the motionplanners in MoveIt!. If the selected plan is good enough the next execution steps will be as follows.The robot first start executing the planned pick trajectory controlling the torso lift joint and the 7degrees of freedom of the arm.  

**note**:once the simulation is proved want to test in the Hardware.

