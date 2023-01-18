#! /usr/bin/env python

## @package exporobot_assignment1
# \file user_interface.py
# \brief This file contains code for 'user_interface' node.
# \author Elham Mohammadi
# \version 1.0
# \date 10/01/2023
#
# \details
#
# Service : <BR>
# 	° /user_interface
#
# This node is user interface of the project that communicates with the user and as per the provided commands, instruct the system to behave accordingly. If the user press 1 in the terminal, it request '/user interface' service which is hosted by 'motion_controller.py' node to start the robot simuation.
#

import rospy
import time
from exporobot_assignment1.srv import Command
from std_msgs.msg import String


##
# \brief This is a 'main' function of user_interface node. 
# 
# \return [none].
#
# This function is a 'main' function of  'user_interface' node. It initializes client for '/user_interface'
# service hosted by 'motion_controller' node. Upon user's request it send the signal to the 'motion_controller' 
# node to start the simulation. 
#
def main():
	rospy.init_node('user_interface')
	user_client = rospy.ServiceProxy('/user_interface', Command)
	rate = rospy.Rate(10)
	x = int(input("\nPress 1 to start the exploration "))
	while not rospy.is_shutdown():
		if (x == 1):
			print("Exploration started!")
			uc_response_ = user_client("start")		
						
			if(uc_response_.res == "done"):
				print("Exploration Completed!")
				x = int(input("\nWant to the start exploration again? Press 1 "))

if __name__ == '__main__':
	main()
