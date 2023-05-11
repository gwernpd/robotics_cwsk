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
block1_done = False
block_3 = False
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
    self.setPose(str(),self.jointRequest,1.0)

    rospy.sleep(1) # Wait for the arm to stand up




  # Get the robot's joint positions
  def getJoints(self,data):
    self.jointPose=data.position

  
    



  # Get data on if the robot is currently moving
  def getStates(self,data):
    if (data.open_manipulator_moving_state=='"STOPPED"'):
      self.readyToMove=True
    else:
      self.readyToMove=False

  def search(self, joint1):
    print('Searching...')
    if joint1 < 0.8:
      self.jointRequest.position=[joint1,-1.352,0.379,1.906]
      self.setPose(str(), self.jointRequest,1.0)
      rospy.sleep(1) #


  # Using the data from all the subscribers, call the robot's services to move the end effector
  def centralise(self):
    if self.readyToMove==True: # If the robot state is not moving

      # Extremely simple - aim towards the target using joints [0] and [3]
      if (abs(self.targetY-0.5)>0.09):
        self.jointRequest.position[3]=self.jointPose[3]+(self.targetY-0.5)
      global switch
      if switch == True:
        if (abs(self.targetX-0.5)>0.09):
          self.jointRequest.position[0]=self.jointPose[0]-(self.targetX-0.5)
          switch = False

      # This command sends the message to the robot
      self.setPose(str(),self.jointRequest,1.0)
      rospy.sleep(1) # Sleep after sending the service request as you can crash the robot firmware if you poll too fast
  
  
  def move_towards(self):
    if self.readyToMove==True: # If the robot state is not moving
      print(self.targetArea)

      if abs(self.targetArea)<125000:
        self.jointRequest.position[1]=self.jointPose[1]+(0.1)
        self.jointRequest.position[3]=self.jointPose[3]-(0.25)
      
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
    self.jointRequest.position=[0.0,-0.5,0,2]
    self.setPose(str(),self.jointRequest,1.0)
    rospy.sleep(2)
    if block1_done == False and block_3 == False:
      self.jointRequest.position=[-1.457,0.370,-0.267,1.671]
      self.setPose(str(),self.jointRequest,1.0)
      rospy.sleep(2)
    if block1_done == True:
      self.jointRequest.position=[-1.457,-0.5,-0.4,2]
      self.setPose(str(),self.jointRequest,1.0)
      rospy.sleep(2)
      self.jointRequest.position=[-1.457,0.017,-0.054,1.634]
      self.setPose(str(),self.jointRequest,1.0)
      rospy.sleep(2)
    if block_3 == True:
      self.jointRequest.position=[-1.457,-0.014,-0.497,1.980]
      self.setPose(str(),self.jointRequest,1.0)
      rospy.sleep(2)
      self.jointRequest.position=[-1.457,0.021,-0.206,1.717]
      self.setPose(str(),self.jointRequest,1.0)
      rospy.sleep(2)
    
    self.gripperRequest.position=[0.01]# -0.01 represents closed
    self.setGripper(str(),self.gripperRequest,1.0)
    rospy.sleep(2)
    self.jointRequest.position=[-1.457,-0.5,0,2]
    self.setPose(str(),self.jointRequest,1.0)
    rospy.sleep(2)
    self.jointRequest.position=[0.0,-0.5,0,2]
    self.setPose(str(),self.jointRequest,1.0)
    rospy.sleep(2)


  # Find the normalised XY co-ordinate of a cube
  def getTarget(self,data):

    # Example = track the biggest red object
    area=[]
    coX=[]
    coY=[]
    global block1_done, block3
    if block1_done == False:
      # Get the red cubes
      for c in range(len(data.cubes)):
        if (data.cubes[c].cube_colour=='red'):
          area.append(data.cubes[c].area)
          coX.append(data.cubes[c].normalisedCoordinateX)
          coY.append(data.cubes[c].normalisedCoordinateY)
    if block1_done == True:
      for c in range(len(data.cubes)):
          if (data.cubes[c].cube_colour=='blue'):
            area.append(data.cubes[c].area)
            coX.append(data.cubes[c].normalisedCoordinateX)
            coY.append(data.cubes[c].normalisedCoordinateY)
    if block_3 == True:
      for c in range(len(data.cubes)):
          if (data.cubes[c].cube_colour=='yellow'):
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
# Main 
def main(args):
  ic = cubeTracker()
  rospy.init_node('cube_tracker', anonymous=True)
  try:
    while not rospy.is_shutdown():
      global ready_pick_up, ready_place, block1_done, block_3, counter
      if ready_pick_up == False:
        ic.centralise() # This is the actual code which controls the robot
        ic.move_towards()
      if ready_pick_up == True and ready_place ==False:
        ic.pick_procedure()
      if ready_place ==True:
        ic.place_procedure()
        block1_done = True
        ready_place = False
        ready_pick_up = False
        counter+=1
        if counter == 2:
          block_3 = True
          block1_done = False
          ready_place = False
          ready_pick_up = False
        if counter ==3:
          break

  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)
