# Cognitive Architecture for robots inspired by quantum Computing




Note: Tiago ++ Package needs to be installed in your environment before you proceed: http://wiki.ros.org/Robots/TIAGo%2B%2B/Tutorials/Installation/InstallUbuntuAndROS

And then you can clone the package from the below link 
                         https://github.com/yeshwanthguru/q-robot-Thesis-Introductory/tree/test_package_ros

After that, you need to launch the simulation environment 

          roslaunch tiago_dual_gazebo tiago_dual_gazebo.launch public_sim:=true end_effector_left:=pal-gripper end_effector_right:=pal-gripper world:=tutorial_office gzpose:="-x 1.40 -y -2.79 -z -0.003 -R 0.0 -P 0.0 -Y 0.0" use_moveit_camera:=true
 (there was some alteration in side the tiago packages to spawn the Aruco  cube instead you can drop the model from the sidebar of the gazebo inside the Tago package,it will be avail once you launch the launch file)
 
 And secondly need to add the following file to bend the tiago head as it is under mapping with real robot,i seems manually more commands

                         Python tuck_arm.py (in our package)
And once you get a notification that starts planning then need to execute the command

          roslaunch tiago_quantum_robot pick_demo.launch

Now at last we need to launch the pick-and-place script 

          python main.py (in our package)

Sensorial unit 1:https://github.com/yeshwanthguru/q-robot-Thesis-Introductory/blob/test_package_ros/scripts/tiago_pickobject_distancetracker.py 

Sensorial unit 2:https://github.com/yeshwanthguru/q-robot-Thesis-Introductory/blob/test_package_ros/scripts/tiago_arm_to_object_distance.py

Sensorila unit 3:https://github.com/yeshwanthguru/q-robot-Thesis-Introductory/blob/test_package_ros/scripts/humandistancecalculator.py

Apart from this attaching the qrobot node:

Single sensory 1:https://github.com/yeshwanthguru/q-robot-Thesis-Introductory/blob/test_main/tiago_testmain2.py
And right now i have sent a script in mail,if ros mg is not received set as false 

Multi-sensory: https://github.com/yeshwanthguru/q-robot-Thesis-Introductory/blob/test_main/test_main3.py
