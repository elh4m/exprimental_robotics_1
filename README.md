# exprimentalLab_project1
# Project 1 - Experimental Robotics (MSc Robotics Engineering, Unige)

The following repository contains a ROS package for a toy simulation of Clauedo game in which a robot explore the environment for hints to find the killer. The environment in this project is an appartment with three rooms in which robot enter one by one to find hints. Based on the discovered hints robot deduces a hypotheses regarding the killer. The deduced hypotheses by robot has to be consistent and correct which means it has to be based on three different types of hints and belongs to set of predefined hypotheses which are considered as corrrect.

1. who: Robot can find a name of a person as a hint which can be a killer e.g: Prof. Plum.
2. what: Robot can find a name of a weapon as a hint which killer might have used e.g: Dagger.
3. where: Robot can find a name of random place where the crime might have been committed e.g: Hall.

Statement of a consistent hypothesis will be something like this: “Prof. Plum with the Dagger in the Hall”. Incase the deduced hypotheses is wrong then the robot will visit the three rooms again for new hints until it forms a consistent hypotheses. 

To deduced an hypothesis robot use the ARMOR package service which is developed by researchers at University of Genova. ARMOR is versatile management system that can handle single or multiple-ontology architectures under ROS. Please find more details regarding ARMOR from here: https://github.com/EmaroLab/armor 

## Project Installation:

This project requires ROS with ARMOR package to be install in the system. Please make sure you have alrady install it before following the instructions. For installing ARMOR package please follow the instructions available in this respository: https://github.com/EmaroLab/armor 

1. Code available in **Main** branch is a ROS package which should be place in ROS workspace {ros_ws}/src after downloading.
2. To successfully deploy and build the package run the following command.
```
catkin_make
cd devel/
source setup.bash
```
3. In order to use the python modules contained in armor_py_api package run the following command to add the path of the armor python modules to your PYTHONPATH environmental variable.
``` 
export PYTHONPATH=$PYTHONPATH:/root/ros_ws/src/armor/armor_py_api/scripts/armor_api/
```
4. Download the 'cluedo_ontology.owl' file provided in this repository and place it in the '/root/Desktop/' directory. 

## Running the Project Simulation:

1. After successfully installing the python package and other dependecies open a command window and start ROS master by using the following command:
```
roscore&
```
2. After that start the ARMOR service by using the following command:
```
rosrun armor execute it.emarolab.armor.ARMORMainService
if this command does not work or it is the first time you are using ARMOR please first run :
roscd armor
./gradlew deployApp
it will not throw an error this time
```
3. Open the new tab in command terminal and run the ROS package launch file to start the simulation by using the following command: 
```
roslaunch exporobot_assignment1 exporobot_assignment1.launch
```
After running the command wait for the system to load all the files. Once all nodes are loaded, the user_interface node will ask to press 1. Upon pressing 1, the simulation will start.

## Software Architecture of the Project:

The project architecture is consist of four python nodes. 

1. user_interface.py
2. motion_controller.py 
3. hint_generator.py 
4. oracle.py

![experimental_robotics_assignment1](https://user-images.githubusercontent.com/61094879/142053996-5c6eaebb-67bb-4f27-918d-5f2c659b772f.jpg)

'user_interface.py' node communicates with the user and as per the provided commands, instruct the system to behave accordingly. If the user press 1 in the terminal, it request '/user interface' service which is hosted by 'motion_controller.py' node to start the robot simuation. Upon recieving the service request, motion_controller node starts the robot simulation in which robot visits Room1, Room2, and Room3 which have pre-defined coordinates(R1(2,0), R2(0,2), R3(-2,0)) in a X-Y axes grid. The robot starts the exploration from a predefined initial position P with coordinates (1,1).


<img width="347" alt="image_2023-01-12_10-49-21" src="https://user-images.githubusercontent.com/77781922/212034325-31b66d02-f34e-491a-afcc-a4b4450f9c93.png">


After reaching in any room the robot request for the hint from the hint_generator node by calling the '/request_hint' service. The hint_generator node respond to this request by generating a random hint from predefined lists of hints.

```
people = ['Rev. Green','Prof. Plum','Col. Mustard','Mrs. Peacock', 'Miss. Scarlett', 'Mrs. White']
weapons = ['Candlestick','Dagger','Lead Pipe','Revolver', Rope', 'Spanner']
places = ['Kitchen','Lounge','Library','Hall','Study', 'Ballroom']
```
After recieving the hint, the robot request the '/oracle_service' service which is hosted by oracle node to load the recently discovered hint in the AMOR reasonser. In response to this request, the robot recieve the confirmation feedback if the hint is successfully loaded in the reasoner. After that robot resume its exploration by visiting other rooms  and repeating the same setups of requesting services for getting hints and then later loading hints in the reasoner. Once the robot has visited all the rooms and collected all the hints then it once again request '/oracle_service' to starts the reasoner to deduced the hypotheses and then to check if the hypotheses based on recenlty loaded hints is conistent or not. If the hints are consistent then the robot go to the origin position O (0,0) and check hypotheses correctness. Hypotheses is correct if it is consistence and belong to a predefined list of hypotheses in oracle node. If hypotheses is also correct then print the hypotheses statement (e.g: “Prof. Plum with the Dagger in the Hall”) on the terminal screen, otherwise it repeats the exploration again starting from Room1. 

The oracle node itself used the '/armor_interface_srv' sevice which is hosted by ARMOR package. When it recieve the request from motion_controller node then it forward this request to the following service making itself working as a bridge between motion_controller and ARMOR package. The idea of having the oracle node in the first place is debatable but in this project the decision for having this node was taken for the purpose of clarity in software architecture and for assigning dedicated task to each node. 

## Project Simulation Demo:

https://user-images.githubusercontent.com/77781922/212033233-c2099e08-8070-413c-9d07-f818dd7bd8f6.mp4

## Code Documentation:

The code documentation is done using tools Doxygen  In the *main* branch the doxygen documentation can be found under the name of dconfig.
## Contant Info: 
1. Author: ELham Mohammadi
2. Email: Elham.mohammadi20154@gmail.com
