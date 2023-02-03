#! /usr/bin/env python

## @package exporobot_assignment1
# \file hint_generator.py
# \brief This file contains code for hint generator node.
# \author Shozab Abidi
# \version 1.0
# \date 15/11/2021
#
# \details
#
# Service : <BR>
# 	° /request_hint
#
# This node controls the random hint generation task. It waits for the '/request_hint' service's request from the motion controller node. Upon recieving request it randomly generates the hint using a predefined lists of hints and response back to the client with the generated hint.  
  
import rospy
import time
from exporobot_assignment1.srv import Hint, HintResponse
from std_msgs.msg import String
from random import randint


##  Initializing global variable 'people' list which one of the type of hints.
people = ['Rev. Green','Prof. Plum','Col. Mustard','Mrs. Peacock',
						'Miss. Scarlett', 'Mrs. White']

##  Initializing global variable 'people' list which one of the type of hints.
weapons = ['Candlestick','Dagger','Lead Pipe','Revolver',
						'Rope', 'Spanner']

##  Initializing global variable 'people' list which one of the type of hints.
places = ['Kitchen','Lounge','Library','Hall','Study', 'Ballroom']
  	
##
# \brief This is a 'clbk_hint_generator' function of hint_generator node. 
# 
# \return Bool.
#
# This is a 'clbk_hint_generator' function of 'hint_generator' node. This function waits for the '/request_hint' service's request from the motion controller node. Upon recieving request it randomly generates the hint using a predefined lists of hints and response back to the client with the generated hint.
#
def clbk_hint_generator(msg):
	if (msg.req == "need_hint"):
		time.sleep(2)
		x = randint(0,2)
		if x == 0:
			hint = ['who',msg.hypo,people[randint(0,len(people)-1)]]

		elif x == 1:
			hint = ['what',msg.hypo,weapons[randint(0,len(weapons)-1)]]

		elif x == 2:
			hint = ['where',msg.hypo,places[randint(0,len(places)-1)]]
			
		print('Generated Hint :', hint)
		return HintResponse(hint)
		  
	else:
		print("hint request msg is not right.", msg.req)
		return HintResponse([])
		
	
##
# \brief This is a 'main' function of hint_generator node. 
# 
# \return [none].
#
# This function is a 'main' function of  'hint_generator' node. It initializes server for '/request_hint' service. 
#	

def main():
    rospy.init_node('hint_generator')
    user_server = rospy.Service('/request_hint', Hint, clbk_hint_generator)
    rospy.spin()

if __name__ == '__main__':
    main()
