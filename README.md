# Yeshwanth Guru's thesis project


https://user-images.githubusercontent.com/72270080/213934590-c9f7e64c-1719-40b4-810f-e99846cdbdb6.mp4

## pseudocode 

                              initialize ROS node
                              initialize publisher
                              define number of qubits, temporal window, and sampling time
                              create instance of SensorialUnit class
                              create instance of QUnit class
                              define move_turtle function
                              initialize last_n_qubits list with n elements set to 0
                              while not rospy shutdown:
                                 try:
                                   wait for turtle's Pose message
                                   update sensorial_unit's scalar reading with 1
                                   measure qubits
                                   update last_n_qubits list
                              check if all qubits in last_n_qubits are 1
                                   activate navigation system by publishing velocity command
                              if not all qubits in last_n_qubits are 1
                                   deactivate navigation system
                            except:
                                 update sensorial_unit's scalar reading with 0
                                 update last_n_qubits list
                                 reinitialize model after a certain time period


## Description
   The above code control a turtle in the turtlesim simulation. The turtle's position is read using the /turtle1/pose topic and the turtle's movement is controlled by publishing to the /turtle1/cmd_vel topic.The code  uses the qrobot library to implement a quantum decision-making system. The SensorialUnit class is used to capture the state of the turtle, and the QUnit class is used to make decisions based on the turtle's state. The AngularModel class is used to define the quantum model, and the OneBurst class is used to define the burst of quantum operations.The move_turtle() function is the main loop of the code. It first initializes an array last_n_qubits with n elements set to 0. It then enters a while loop that will keep running until the ROS node is shut down. Inside the while loop, the code waits for the turtle's Pose message and updates the sensorial_unit's scalar reading with 1. It then waits for the burst to finish and measures the qubits using the model's measure() method with a number of shots. The result is added to the last_n_qubits list and the oldest element is removed.If all elements in last_n_qubits list are 1, the turtle's movement is enabled by publishing velocity commands to the /turtle1/cmd_vel topic. If all elements in last_n_qubits list are 0, the turtle's movement is disabled.In case of an exception, the sensorial_unit's scalar reading is updated with 0, the qubits are measured again, and the result is added to the last_n_qubits list. The turtle is not detected and the model is cleared after a certain time period (5 seconds)

## Process to launch the simulation 

The Package has three nodes two inside ros environment such as Turtlesim and subscribernode which can be launched with the following commands,
                  
                    rosrun turtlesim turtlesim_node 
                    
turtle sim node should be initiated after the rosmaster initiated.Later launch the subscriber node with the following commnd 

                    rosrun turtle_control subscriber.py
                    
To initiate the node with quantum decision model in this perception model ,the node should be in the environment where all the qrobot dependencies were installed 

                     python3.8 quantum_publisher.py
                     
For the output I have attached the video for reference.
## scenario: 
We associate |0> of the single qubit with the "Turtle start" and |1> with the "Turtle stop", and we activate the "Navigation" classical system every time we measure |1> in the "decision-making" quantum system and we keep the navigation  process engaged as long as we read |1> in the quantum system. When we read |0>, we invert the process returning at the base location unless we do not read |1> again, in that case, we re-engage, etc.. etc...
## Main Thesis scenario :
 We associate |0> of the single qubit with the "Robot hand" and |1> with the "Human hand", and we activate the "picking" classical system every time we measure |1> in the "decision-making" quantum system and we keep the picking process engaged as long as we read |1> in the quantum system. When we read |0>, we invert the process returning at the base location unless we do not read |1> again, in that case, we re-engage, etc.. etc...
 
 There for above working scenario proved the main thesis scenario.Further development under process.......
 
 ## Can explore in the future working:
 Additionally, you could explore the use of quantum error correction codes or quantum entanglement to improve the robustness and accuracy of the decision-making process. For example,Encode multiple qubits to represent the turtle's state and use quantum error correction codes to protect the qubits against noise.


## note : The publisher node should be in the environment where the qrobot package and dependencies Present.
