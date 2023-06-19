# Cognitive Architecture for robots inspired by quantum Computing




Note: Tiago ++ Package needs to be installed in your environment before you proceed: http://wiki.ros.org/Robots/TIAGo%2B%2B/Tutorials/Installation/InstallUbuntuAndROS

And then you can clone the package from the below link 
                         https://github.com/yeshwanthguru/q-robot-Thesis-Introductory/tree/test_package_ros

After that, you need to launch the simulation environment 

          roslaunch tiago_dual_gazebo tiago_dual_gazebo.launch public_sim:=true end_effector_left:=pal-gripper end_effector_right:=pal-gripper world:=tutorial_office gzpose:="-x 1.40 -y -2.79 -z -0.003 -R 0.0 -P 0.0 -Y 0.0" use_moveit_camera:=true

And once you get a notification that start planning then need to execute the command

          roslaunch tiago_quantum_robot pick_demo.launch

Now at last we need to launch the pick-and-place script 

          python main.py
