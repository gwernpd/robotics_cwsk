#!/usr/bin/env python
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
switch = True
ready_pick_up = False
ready_place = False
block_1 = False
block_2 = False
block_3 = False
cube_colour = ''
search = True
class cubeTracker:

  def __init__(self):
    self.setPose = rospy.ServiceProxy('goal_joint_space_path', SetJointPosition)
    self.setGripper = rospy.ServiceProxy('goal_tool_control', SetJointPosition)
    
    # Where the block is in the image (start at the centre)
    self.targetX=0.5
    self.targetY=0.5
    self.targetZ=0.1
    self.targetArea=0
    self.previous_area=0

    # Whether the robot is ready to move (assume it isn't)
    self.readyToMove=False

    # Home postion for the robot to move to
    self.jointPose=[0.136,0,0.236]

    # Create the subscribers
    self.image_sub = rospy.Subscriber('states',OpenManipulatorState,self.getStates)
    self.joint_state_sub = rospy.Subscriber('joint_states',JointState,self.getJoints)
    self.moving_sub = rospy.Subscriber('cubes',cubeArray,self.getTarget)

    # Create the service caller to move the robot

    ## EXAMPLE POSES - Start by moving the robot to some example positions, and use the gripper



    # Send the robot "home"
    self.gripperRequest=JointPosition()
    self.gripperRequest.joint_name=["gripper"]  
    self.gripperRequest.position=[-0.01]# -0.01 represents closed
    self.setGripper(str(),self.gripperRequest,1.0)
    rospy.sleep(1) # Wait for the arm to stand up


    # Close the gripper
    self.gripperRequest=JointPosition()
    self.gripperRequest.joint_name=["gripper"]  
    self.gripperRequest.position=[0.01]# -0.01 represents closed
    self.setGripper(str(),self.gripperRequest,1.0)

    self.setPose = rospy.ServiceProxy('goal_joint_space_path', SetJointPosition)    
    self.jointRequest=JointPosition()
    self.jointRequest.joint_name=["joint1","joint2","joint3","joint4"]  
    self.jointRequest.position=[0.0,-0.5,0,2]
    self.setPose(str(),self.jointRequest,2.0)

    rospy.sleep(2) # Wait for the arm to stand up




  # Get the robot's joint positions
  def getJoints(self,data):
    self.jointPose=data.position

  
    



  # Get data on if the robot is currently moving
  def getStates(self,data):
    if (data.open_manipulator_moving_state=='"STOPPED"'):
      self.readyToMove=True
    else:
      self.readyToMove=False

  def search(self, joint_1):
    global counter, block_1, block_2, block_3, cube_colour, search
    print('Searching...')
    if joint_1 < 0.8:
      self.jointRequest.position=[joint_1,-1.352,0.379,1.906]
      self.setPose(str(), self.jointRequest,2.0)
      rospy.sleep(2) #
    if self.targetArea > 500:
      search = False
      if counter == 0:
        block_1 = True
        print(cube_colour)
      if counter == 1:
        block_1 = False
        block_2 = True
        print(cube_colour)
      if counter == 2:
        block_2 = False
        block_3 = True
        print(cube_colour)


  # Using the data from all the subscribers, call the robot's services to move the end effector
  def centralise(self):

      # Extremely simple - aim towards the target using joints [0] and [3]
      if (abs(self.targetY-0.5)>0.02):
        self.jointRequest.position[3]=self.jointPose[3]+(self.targetY-0.5)
      global switch
      if switch == True:
        if (abs(self.targetX-0.5)>0.1):
          self.jointRequest.position[0]=self.jointPose[0]-(self.targetX-0.5)
          switch = False

      # This command sends the message to the robot
      self.setPose(str(),self.jointRequest,1.0)
      rospy.sleep(1) # Sleep after sending the service request as you can crash the robot firmware if you poll too fast
  
  
  def move_towards(self):
    if self.readyToMove==True: # If the robot state is not moving
      print(self.targetArea)

      if abs(self.targetArea)<125000:
        try:
          self.jointRequest.position[1]=(self.jointPose[1]+(0.2))
          self.jointRequest.position[3]=self.jointPose[3]-(0.05)
          self.jointRequest.position[2]=(self.jointPose[2]-(0.05))
        except:
          self.jointRequest.position[1]=(self.jointPose[1]+(0.15))
      
      if abs(self.targetArea)>125000:
        global ready_pick_up
        ready_pick_up = True


  def pick_procedure(self):
    print('picking_up')
    self.jointRequest.position[3]=self.jointPose[3]-(0.44)
    self.setPose(str(),self.jointRequest,2.0)
    rospy.sleep(2)
    self.gripperRequest.position=[-0.01]# -0.01 represents closed
    self.setGripper(str(),self.gripperRequest,1.0)
    rospy.sleep(2)
    global ready_place
    ready_place = True

  def place_procedure(self):
    global search, joint_1
    self.jointRequest.position=[joint_1,-0.189,-0.178,1.852]
    self.setPose(str(),self.jointRequest,2.0)
    rospy.sleep(2)
    if block_1 == True:
      self.jointRequest.position=[-1.457,0.584,-0.502,1.607]
      self.setPose(str(),self.jointRequest,2.0)
      rospy.sleep(2)
      search = True
    if block_2 == True:
      self.jointRequest.position=[-1.457,-0.5,-0.4,2]
      self.setPose(str(),self.jointRequest,2.0)
      rospy.sleep(2)
      self.jointRequest.position=[-1.457,0.348,-0.328,1.6548]
      self.setPose(str(),self.jointRequest,2.0)
      rospy.sleep(2)
      search = True
    if block_3 == True:
      self.jointRequest.position=[-1.457,-0.014,-0.640,1.7]
      self.setPose(str(),self.jointRequest,1.0)
      rospy.sleep(2)
      self.jointRequest.position=[-1.457,0.281,-0.419,1.669]
      self.setPose(str(),self.jointRequest,1.0)
      rospy.sleep(2)
    
    self.gripperRequest.position=[0.01]# -0.01 represents closed
    self.setGripper(str(),self.gripperRequest,1.0)
    rospy.sleep(2)
    self.jointRequest.position=[-1.457,-0.5,0,2]
    self.setPose(str(),self.jointRequest,1.0)
    rospy.sleep(2)
    self.jointRequest.position=[joint_1,-1.352,0.379,1.906]
    self.setPose(str(),self.jointRequest,2.0)
    rospy.sleep(2)


  # Find the normalised XY co-ordinate of a cube
  def getTarget(self,data):

    # Example = track the biggest red object
    area=[]
    coX=[]
    coY=[]
    global block_1, block_2, block_3, cube_colour
      # Get the cubes
    for c in range(len(data.cubes)):
      cube_colour = data.cubes[c].cube_colour
      if (data.cubes[c].cube_colour==cube_colour):
        area.append(data.cubes[c].area)
        coX.append(data.cubes[c].normalisedCoordinateX)
        coY.append(data.cubes[c].normalisedCoordinateY)


    # Find the biggest red cube
    if (len(area))>0:
      global detected
      index_max = max(range(len(area)), key=area.__getitem__)
      self.targetX=coX[index_max]
      self.targetY=coY[index_max]
      self.targetArea=area[index_max]
    else: # If you dont find a target, report the centre of the image to keep the camera still
      #self.previous_area = area[index_max-1]
      self.targetX=0.5
      self.targetY=0.5

counter = 0
joint_1 = -1.0
search = True
# Main 
def main(args):
  ic = cubeTracker()
  rospy.init_node('cube_tracker', anonymous=True)
  try:
    while not rospy.is_shutdown():
      global ready_pick_up, ready_place, block_1, block_2, block_3, counter, switch, joint_1
      if search == True:
        ic.search(joint_1)
        joint_1 = joint_1 + 0.3
      if ready_pick_up == False:
        ic.centralise() # This is the actual code which controls the robot
        ic.move_towards()
      if ready_pick_up == True and ready_place ==False:
        ic.pick_procedure()
      if ready_place ==True:
        ic.place_procedure()
        block_2 = True
        block_3 = False
        block_1 = False
        ready_place = False
        ready_pick_up = False
        switch = True
        counter+=1
        if counter == 2:
          block_3 = True
          block_2 = False
          block_1 = False
          ready_place = False
          ready_pick_up = False
          switch = True
        if counter ==3:
          break

  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)
