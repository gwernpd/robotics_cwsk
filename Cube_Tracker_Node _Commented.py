#!/usr/bin/env python
# Importing necessary libraries
# rospy for ROS Python API
# sys for parsing command line arguments

from __future__ import print_function
from re import X

import roslib
roslib.load_manifest('cube_spotter')
import sys
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import JointState



from cube_spotter.msg import cubeData
from cube_spotter.msg import cubeArray
import numpy as np;
import math

from open_manipulator_msgs.msg import OpenManipulatorState
from open_manipulator_msgs.msg import JointPosition
from open_manipulator_msgs.srv import SetJointPosition
from open_manipulator_msgs.srv import GetJointPosition

#--------CONDITIONAL IDENTITIES------------
switch = True #used to centralise joint 0 when the area of a block is less 50000
ready_pick_up = False #activate pick up proceedure function if true
ready_place = False #activate place proceedure function if true

#Used to identify the order of blocks being picked up in the search function
block_1 = False
block_2 = False
block_3 = False

cube_colour = '' #List of cube colours: RED, BLUE, YELLOW
search = True #activate the search function
start_detect = False #when true, enter a 'for' loop to identify the colour of the block

#When true, stop seaching for each specific colour
no_red = False
no_blue = False
no_yellow = False

check_range = False #When true, enter 'if' statements to identify whether blocks in the camera lens are within the workspace 

#------------------------------------------------------------------------------------
# Define cubeTracker class
class cubeTracker:

  def __init__(self): # Initialize the class
    # Create ServiceProxies for robot control, 'goal_joint_space_path' and 'goal_tool_control' are custom ROS services
    self.setPose = rospy.ServiceProxy('goal_joint_space_path', SetJointPosition)
    self.setGripper = rospy.ServiceProxy('goal_tool_control', SetJointPosition)
    
    # Where the block is in the image (start at the centre)
    self.targetX=0.5
    self.targetY=0.5
    self.targetZ=0.1
    self.targetArea=0
    self.previous_area=0
    self.colour = ''

    # Whether the robot is ready to move (assume it isn't)
    self.readyToMove=False

    # Home pose for the robot to move to
    self.jointPose=[0.136,0,0.236]

    # Create the subscribers for ROS topics 'states', 'joint_states' and 'cubes'
    self.image_sub = rospy.Subscriber('states',OpenManipulatorState,self.getStates)
    self.joint_state_sub = rospy.Subscriber('joint_states',JointState,self.getJoints)
    self.moving_sub = rospy.Subscriber('cubes',cubeArray,self.getTarget)

    # Initialise gripper and send the robot "home"
    self.gripperRequest=JointPosition()
    self.gripperRequest.joint_name=["gripper"]  
    self.gripperRequest.position=[-0.01]# -0.01 represents closed
    self.setGripper(str(),self.gripperRequest,1.0) 
    rospy.sleep(1) # Wait for the arm to stand up

    # open the gripper
    self.gripperRequest=JointPosition()
    self.gripperRequest.joint_name=["gripper"]  
    self.gripperRequest.position=[0.01]# 0.01 represents open
    self.setGripper(str(),self.gripperRequest,1.0)

    self.setPose = rospy.ServiceProxy('goal_joint_space_path', SetJointPosition) #set a new xyz pose   
    self.jointRequest=JointPosition() #request current joint angles
    self.jointRequest.joint_name=["joint1","joint2","joint3","joint4"] #Name each joint
    self.jointRequest.position=[0.0,-0.5,0,2] #request joint angle positions
    self.setPose(str(),self.jointRequest,2.0) #set joint angle request in a given time (s)

    rospy.sleep(2) # Wait for the arm to stand up

  # Get the robot's joint positions
  def getJoints(self,data):
    self.jointPose=data.position

  # Get data on if the robot is currently moving
  def getStates(self,data):    # Defining getStates() function
    if (data.open_manipulator_moving_state=='"STOPPED"'):    # checking if open_manipulator_moving_state is equal to "STOPPED"
      self.readyToMove=True    # Set readyToMove to True
    else: 
      self.readyToMove=False    # Set readyToMove to False

  def search(self, joint_1):    # Define search function
    global counter, block_1, block_2, block_3, search, start_detect, check_range    # Define global variables
    print('Searching...')    # Print message on console
    
    if joint_1 < 0.25:    # check if joint_1 is less than 0 rad
      self.jointRequest.position=[joint_1,-1.352,0.379,1.906]    # request joint joint positions
      self.setPose(str(), self.jointRequest,2.0)    # call function setPose() to set joint request
      rospy.sleep(2) #    # wait for 2 seconds
      start_detect = True    # set value of start_detect to True


    if self.targetArea > 500:    # check if targetArea is greater than 500
      search = False    # set value of search to False
      check_range = True
      if counter == 0:    # check if counter is 0
        block_1 = True    # set value of block_1 to True
      if counter == 1:    # check if counter is 1
        block_1 = False    # set value of block_1 to False
        block_2 = True    # set value of block_2 to True
      if counter == 2:    # check if counter is 2
        block_2 = False    # set value of block_2 to False
        block_3 = True    # set value of block_3 to True

  # Using the data from all the subscribers, call the robot's services to move the end effector
  def centralise(self):    # Defining function "centralise"
      if self.jointPose[3] > 1.90:    # Check if value of joint 4's jointPose is greater than 1.9
            self.jointRequest.position[3]=self.jointPose[3]-(0.15)    # Subtract 0.15 from the value of joint 4's jointPose and store in 3rd index of jointRequest
            self.jointRequest.position[1]=(self.jointPose[1]+(0.15))    # Add 0.15 from the value of joint 2's jointPose and store in 1st index of jointRequest
            self.jointRequest.position[2]=(self.jointPose[2]+(0.1))    # Add 0.1 from the value of joint 3's jointPose and store in 2nd index of jointRequest

      # Aim towards the target using joints [0] and [3]
      if (abs(self.targetY-0.5)>0.02):    # Check if the difference of targetY and 0.5 is greater than 0.02
        self.jointRequest.position[3]=self.jointPose[3]+(self.targetY-0.5)   # Add the difference of targetY and 0.5 to the value of joint 4's jointPose and store in 3rd index of jointRequest
      global switch, check_range    # Declare switch and check_range as global variables
      if self.targetArea > 50000:    # Check if value of targetArea is greater than 50000
        switch = False    # Assign false to switch
      if switch == True:    # Check if switch is true
        if (abs(self.targetX-0.5)>0.1):    # Check if difference of targetX and 0.5 is greater than 0.1
          self.jointRequest.position[0]=self.jointPose[0]-(self.targetX-0.5)    # Subtract the difference of targetX and 0.5 to the value of joint 1's jointPose and store in 0th index of jointRequest
          
      # This command sends the message to the robot    # Comment
      self.setPose(str(),self.jointRequest,1.0)    # set requested joint pose in 1 second
      rospy.sleep(1) # Wait 1 second
      #check_range = True    # Assign true to check_range
      # This command sends the message to the robot
      self.setPose(str(),self.jointRequest,1.0)
      rospy.sleep(1) # Sleep after sending the service request as you can crash the robot firmware if you poll too fast
      #check_range = True    # Assign true to check_range
  
  def move_towards(self):    # Defining function "move_towards"
    global counter, check_range, ready_pick_up, search, no_red, no_blue, no_yellow,start_detect    # initializing counters
    if self.readyToMove==True: # If readyToMove is true
      #print(self.targetArea)    # print targetArea 
      move = True    # set move to true
      if check_range == True:    # check if check_range is true 
        if abs(self.targetArea)<10750:    # check if targetArea is less than 10750
          if len(self.colour) ==0:    # check if colour index is empty
            pass    # continue searching if there is no colour detected
          if self.colour == 'red':    # if colour red is detected in the index
            print("red is out of range")    # print message on console
            no_red = True    # initializing no_red as true
          elif self.colour == 'blue':    # if colour blue is detected in the index
            print("blue is out of range")    # print message on console
            no_blue = True    # initializing no_blue as true
          elif self.colour == 'yellow':    # if colour yellow is detected in the index
            print("yellow is out of range")    # print message on console
            no_yellow = True    # initializing no_yellow as true
          move = False    # set as false
          search = True    # set search as true
          start_detect = True    # set start_detect as true
          check_range = False    # set check_range as false
          
      if move == True:    # check move is true 

        if abs(self.targetArea)<135000 and abs(self.targetArea)>10750:    # check if targetArea is betweem 135000 and 10750 
          if self.jointPose[3] > 1.90:    # check if jointPose is greater than 1.90 
            self.jointRequest.position[3]=self.jointPose[3]+(0.1)    # increase the value of joint 4 by 0.1
            self.jointRequest.position[1]=(self.jointPose[1]-(0.1))    # decrease the value of joint 2 by 0.1
            self.jointRequest.position[2]=(self.jointPose[2]+(0.1))    # increase the value of joint 3 by 0.1
          try:    # try block
            self.jointRequest.position[1]=(self.jointPose[1]+(0.2))    # increase the value of joint 2 by 0.2
            self.jointRequest.position[3]=self.jointPose[3]-(0.05)    # decrease the value of joint 4 by 0.05
            self.jointRequest.position[2]=(self.jointPose[2]-(0.1))    # decrease the value of joint 3 by 0.1
          except:    # except block
            self.jointRequest.position[1]=(self.jointPose[1]+(0.15))    # increase the value of joint 2 by 0.15
      
      if abs(self.targetArea)>125000:    # check if targetArea is greater than 125000 
        
        ready_pick_up = True    # set ready_pick_up as true


  def pick_procedure(self):    # Define function "pick_procedure"
    global ready_place, start_detect  # Initialize global variables
    start_detect = False #set start_detect to false
    #print('picking_up') #print picking up
    self.jointRequest.position[3]=self.jointPose[3]-(0.44)    # decrease joint 3 by 0.44 from the current position 
    self.setPose(str(),self.jointRequest,2.0)    # Set the pose of the robot in 2 seconds
    rospy.sleep(2)    # wait for 2 seconds
    self.gripperRequest.position=[-0.01]# -0.01 represents closed
    self.setGripper(str(),self.gripperRequest,1.0)    # Set gripper position
    rospy.sleep(2)    # wait for 2 seconds
    ready_place = True    # Change value of ready_place to True

  def place_procedure(self):    # Define function "place_procedure"
    global search, joint_1, no_red, no_blue, no_yellow, counter, start_detect 
    if block_1 == True:    # if block_1 is True
      self.jointRequest.position=[self.jointPose[0],-0.014,-0.640,1.7]    # request the joints to move positions
      self.setPose(str(),self.jointRequest,2.0)    # Set the pose of joints
      rospy.sleep(2)    # wait for 2 seconds
      self.jointRequest.position=[-1.457,-0.014,-0.640,1.7]    # request the joints to move positions
      self.setPose(str(),self.jointRequest,2.0)    # Set the pose of joints
      rospy.sleep(2)    # wait for 2 seconds

      self.jointRequest.position=[-1.457,0.600,-0.494,1.55]    # request the joints to move positions
      self.setPose(str(),self.jointRequest,2.0)    # Set the pose of joints
      rospy.sleep(2)    # wait for 2 seconds

      if self.colour == 'red':    # if colour of block being picked up is red
        no_red = True    # change value of no_red to True
        search = True    # change value of search to True to re-enter search_procedure
      elif self.colour == 'blue':    # else if colour of the block being picked up is blue
        no_blue = True    # change value of no_blue to True
        search = True    # change value of search to True
      elif self.colour == 'yellow':    # else if colour of the  block being picked up is yellow
        no_yellow = True    # change value of no_yellow to True
        search = True    # change value of search to True
    if block_2 == True:    # if block_2 is True
      self.jointRequest.position=[self.jointPose[0],-0.014,-0.640,1.7]    # request joint 1, 2, 3, 4 to move to given positions
      self.setPose(str(),self.jointRequest,2.0)    # Set the joint position
      rospy.sleep(2)    # wait for 2 seconds
      self.jointRequest.position=[-1.457,-0.046,-0.463,1.703]    # request joint 1, 2, 3, 4 to move to given positions
      self.setPose(str(),self.jointRequest,2.0)    # Set the joint position
      rospy.sleep(2)    # wait for 2 seconds
      self.jointRequest.position=[-1.457,0.374,-0.437,1.608]    # request joint 1, 2, 3, 4 to move to given positions
      self.setPose(str(),self.jointRequest,2.0)    # Set the joint position
      rospy.sleep(2)    # wait for 2 seconds

      if self.colour == 'red':    # if colour is red
        no_red = True    # change value of no_red to True
        search = True    # change value of search to True
      elif self.colour == 'blue':    # else if colour is blue
        no_blue = True    # change value of no_blue to True
        search = True    # change value of search to True
      elif self.colour == 'yellow':    # else if colour is yellow
        no_yellow = True    # change value of no_yellow to True
        search = True    # change value of search to True
      
    if block_3 == True:    # if block_3 is True
      self.jointRequest.position=[self.jointPose[0],-0.014,-0.640,1.7]    # Request new joint position
      self.setPose(str(),self.jointRequest,2.0)    # Set the pose of robot 
      rospy.sleep(2)    # wait for 2 seconds
      self.jointRequest.position=[-1.457,-0.219,-0.426,1.611]    #  request new joint position
      self.setPose(str(),self.jointRequest,2.0)    # Set the pose of robot 
      rospy.sleep(2)    # wait for 2 seconds
      
      self.jointRequest.position=[-1.457,0.249,-0.417,1.7]    # request new joint position
      self.setPose(str(),self.jointRequest,2.0)    # Set the pose of robot 
      rospy.sleep(2)    # wait for 2 seconds

      no_red = True    # change value of no_red to True
      no_blue = True    # change value of no_blue to True
      no_yellow = True    # change value of no_yellow to True
      counter = 3    # change value of counter to 3
    
    
    self.gripperRequest.position=[0.01]# 0.01 represents open
    self.setGripper(str(),self.gripperRequest,1.0)    # Set gripper in open position
    rospy.sleep(2)    # Delay of 2 seconds
    self.jointRequest.position=[-1.457,-0.5,0,2]    # request joint positions
    self.setPose(str(),self.jointRequest,2.0)    # Set the robot pose 
    rospy.sleep(5)    # Delay of 5 seconds
    start_detect = True    # Change start_detect to True
    self.jointRequest.position=[joint_1,-1.352,0.379,1.906]    # request joint positions
    self.setPose(str(),self.jointRequest,2.0)    # Set the robot pose
    rospy.sleep(2)    # Delay of 2 seconds

    # Find the normalised XY co-ordinate of a cube
  def getTarget(self,data):    # Define the function getTarget()
    area=[]    # Initialize an empty list for area
    colour = []    # Initialize an empty list for colour
    coX=[]    # Initialize an empty list for x co-ordinate
    coY=[]    # Initialize an empty list for y co-ordinate
    global block_1, block_2, block_3, search, start_detect, no_red, no_blue, no_yellow, counter    # Initialize global variables
      # Get the cubes
    if start_detect == True:    # If start_detect is True
      for c in range(len(data.cubes)):    # Loop through the cubes
        if no_red == True and data.cubes[c].cube_colour == 'red':    # Check if no_red is True and cube colour is red
          pass    # Do nothing
        elif no_blue == True and data.cubes[c].cube_colour == 'blue':    # Check if no_blue is True and cube colour is blue
          pass    # Do nothing
        elif no_yellow == True and data.cubes[c].cube_colour == 'yellow':    # Check if no_yellow is True and cube colour is yellow
          pass    # Do nothing
        else:    # If no colours are true
          area.append(data.cubes[c].area)    # Append area to area list
          colour.append(data.cubes[c].cube_colour)    # Append colour to colour list
          coX.append(data.cubes[c].normalisedCoordinateX)    # Append x co-ordinate to coX list
          coY.append(data.cubes[c].normalisedCoordinateY)    # Append y co-ordinate to coY list
          if no_red == True and no_blue == True and no_yellow == True:    # Check if no other colours are true
            counter = 3    # Set counter to 3

    if (len(area))>0:    # Check if area list is empty 
      global detected    # Initialize detected as global variable
      index_max = max(range(len(area)), key=area.__getitem__)    # Find the biggest value in the area index
      self.targetX=coX[index_max]    # set targetX as maximum x co-ordinate
      self.targetY=coY[index_max]    # set targetY as maximum y co-ordinate
      self.targetArea=area[index_max]    # set targetArea as maximum area
      self.colour = colour[index_max]    # set colour as maximum colour
    else: # If you dont find a target, report the centre of the image to keep the camera still
      self.targetX=0.5    # set targetX as 0.5
      self.targetY=0.5    # set targetY as 0.5

counter = 0    # set counter as 0
joint_1 = -1.0    # set joint_1 as -1.0
search = True    # set search as True

# Main 
def main(args):    # Main function
  ic = cubeTracker()    # Initialize cubeTracker class
  rospy.init_node('cube_tracker', anonymous=True)    # Initialize ros node
  try:
    while not rospy.is_shutdown():    # While ros is not shutdown
      global ready_pick_up, ready_place, block_1, block_2, block_3, counter, switch, joint_1    # Initialize global variables

      #Display wether a block has been placed or out of reach
      print('Not bothering with Red: {}'.format(no_red))    # Print red colour
      print('Not bothering with Blue: {}'.format(no_blue))    # Print blue colour
      print('Not bothering with Yellow: {}'.format(no_yellow))    # Print yellow colour
      print("                               ")

      if no_red == True and no_blue == True and no_yellow == True:    # Check if no other colours are true
            counter = 3    # Set counter to 3
      if counter ==3 :    # Check if counter is 3
        break    # Break the loop to end task
      if search == True:    # Check if search is True
        ic.search(joint_1)    # Call search function
        joint_1 = joint_1 + 0.25    # increase joint_1 by 0.25
        if joint_1 == 0.25: # Check if joint_1 has reached our limit
          joint_1 = -1 # resets the search function
      
      if ready_pick_up == False:    # Check if ready_pick_up is False
        ic.centralise() # Call centralise function
        rospy.sleep(1)    # Delay of 1 second
        ic.move_towards()    # Call move_towards function
      if ready_pick_up == True and ready_place ==False:    # Check if ready_pick_up is True and ready_place is False
        ic.pick_procedure()    # Call pick_procedure function
      if ready_place ==True:    # Check if ready_place is True
        ic.place_procedure()    # Call place_procedure function
        block_2 = True    # Set block_2 as True
        block_3 = False    # Set block_3 as False
        block_1 = False    # Set block_1 as False
        ready_place = False    # Set ready_place as False
        ready_pick_up = False    # Set ready_pick_up as False
        switch = True    # Set switch as True
        counter+=1    # increase counter by 1
        if counter == 2:    # Check if counter is 2
          block_3 = True    # Set block_3 as True
          block_2 = False    # Set block_2 as False
          block_1 = False    # Set block_1 as False
          ready_place = False    # Set ready_place as False
          ready_pick_up = False    # Set ready_pick_up as False
          switch = True    # Set switch as True
        if counter ==3:    # Check if counter is 3
          break    # Break the loop to end task
  except KeyboardInterrupt: #when pressing ctrl+c
    print("Shutting down")    # Print message on console

if __name__ == '__main__':
    main(sys.argv)    # Call main function
