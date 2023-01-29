# Tiago decision making pick and place scenario

https://user-images.githubusercontent.com/72270080/215219345-503e1c80-e754-4fe0-82af-3b52290d4e5e.mp4



## Doubts to be cleared in the code:
 1.default uncertain data scalar reading encoding in the qubits.
 
 2.during the decision process unable to see the circuit encode values, 
 
 3.if all(last_n_qubits) == 1: not sure about this whether match the scenario ,just need the clearance to this.
 as per the simulation it matches but need some error correction in the script.
 
## Installation process with ubuntu

Installing Ubuntu with ROS + TIAGO ++ supports only ROS melodic (ubuntu 18.04)

INSTALLATION PROCESS-http://wiki.ros.org/Robots/TIAGo%2B%2B/Tutorials/Installation/InstallUbuntuAndROS 

## QROBOT PACKAGE IN THE EXTERNAL NON ROS ENVIRONMENT WITH ALL ITS REQIRED PACKAGES

# CODE EXECUTION PROCESS :

#e ROS SIDE 

Terminal1:
![Screenshot from 2023-01-29 02-58-56](https://user-images.githubusercontent.com/72270080/215300182-2cce5376-6335-4444-ad24-decbc4067eb8.png)
                              
                               roslaunch tiago_pick_demo pick_simulation.launch
once the above command executed wait till the command Arm tuck in the terminal where you executed the above code.Then execute the other command:
 
                                 roslaunch tiago_pick_demo pick_demo.launch 
                                 
![Screenshot from 2023-01-29 03-03-22](https://user-images.githubusercontent.com/72270080/215300331-68b19ad2-8913-439c-a223-14a4e872f380.png)
![Screenshot from 2023-01-29 03-03-39](https://user-images.githubusercontent.com/72270080/215300334-15b1934e-61fa-4609-b964-4703367ba654.png)
![Screenshot from 2023-01-29 03-03-43](https://user-images.githubusercontent.com/72270080/215300337-dcddbacd-7e33-4aea-aa38-b152cf7a7ce7.png)

#Non-ROS side:
Once everything ready run the code in the qrobot workspace.
       
  
