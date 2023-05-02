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
central = True
test = True
pick_up = False
back_home = False


class cubeTracker:

  def __init__(self):

    # Where the block is in the image (start at the centre)
    self.targetX=0.5
    self.targetY=0.5
    self.targetZ=0.1
    self.targetArea=0
    self.previous_area=0

    # Whether the robot is ready to move (assume it isn't)
    self.readyToMove=False

    # Home postion for the robot to move to
    self.jointPose=[0.0,-1.05,0.357,0.703]

    # Create the subscribers
    self.image_sub = rospy.Subscriber('states',OpenManipulatorState,self.getStates)
    self.joint_state_sub = rospy.Subscriber('joint_states',JointState,self.getJoints)
    self.moving_sub = rospy.Subscriber('cubes',cubeArray,self.getTarget)

    # Create the service caller to move the robot
    self.setPose = rospy.ServiceProxy('goal_joint_space_path', SetJointPosition)
    self.setGripper = rospy.ServiceProxy('goal_tool_control', SetJointPosition)


    ## EXAMPLE POSES - Start by moving the robot to some example positions, and use the gripper


    # Send the robot to "zero"
    self.jointRequest=JointPosition()
    self.jointRequest.joint_name=["joint1","joint2","joint3","joint4"]  
    self.jointRequest.position=[0.0,0.0,0.0,0.0]
    self.setPose(str(),self.jointRequest,1.0)

    rospy.sleep(1) # Wait for the arm to stand up

    # Open the gripper
    self.gripperRequest=JointPosition()
    self.gripperRequest.joint_name=["gripper"]  
    self.gripperRequest.position=[0.01]# 0.01 represents open
    self.setGripper(str(),self.gripperRequest,1.0)

    rospy.sleep(1) # Wait for the gripper to open

    # Close the gripper
    self.gripperRequest=JointPosition()
    self.gripperRequest.joint_name=["gripper"]  
    self.gripperRequest.position=[0.01]# -0.01 represents closed
    self.setGripper(str(),self.gripperRequest,1.0)

    rospy.sleep(1) # Wait for the gripper to close

    # Send the robot "home"
    self.jointRequest=JointPosition()
    self.jointRequest.joint_name=["joint1","joint2","joint3","joint4"]  
    self.jointRequest.position=[0.0,-1.05,0.35,1.8]
    self.setPose(str(),self.jointRequest,1.0)

    rospy.sleep(1) # Wait for the arm to stand up

    # As the last movement called was the arm, we dont update the request again below, but it would be necessary if switching between the arm and the gripper.


  # Get the robot's joint positions
  def getJoints(self,data):
    self.jointPose=data.position





  # Get data on if the robot is currently moving
  def getStates(self,data):
    if (data.open_manipulator_moving_state=='"STOPPED"'):
      self.readyToMove=True
    else:
      self.readyToMove=False



  # Using the data from all the subscribers, call the robot's services to move the end effector
  def aimCamera(self):
    global central
    if self.readyToMove==True: # If the robot state is not moving

      # Extremely simple - aim towards the target using joints [0] and [3]
      if (abs(self.targetY-0.58)>0.1):
        self.jointRequest.position[3]=self.jointPose[3]+(self.targetY-0.85)
        central = True

      if (abs(self.targetX-0.55)>0.1):
        self.jointRequest.position[0]=self.jointPose[0]-(self.targetX-0.55)
        central = True

      

      # This command sends the message to the robot
      self.setPose(str(),self.jointRequest,1.0)
      rospy.sleep(1) # Sleep after sending the service request as you can crash the robot firmware if you poll too fast
  
  
  def move_towards(self):
      global central, test, pick_up
      if self.readyToMove==True: # If the robot state is not moving
        print(self.targetArea)
        if self.targetArea < 60000 and test==True:
          self.jointRequest.position[1]=self.jointPose[1]+0.1
        if self.targetArea > 60000:
        
          central=False
          test = False
          pick_up = True


        # This command sends the message to the robot
        self.setPose(str(),self.jointRequest,1.0)
        rospy.sleep(1) # Sleep after sending the service request as you can crash the robot firmware if you poll too fast

  def calc_Z(self):
    distance = 0.233*math.tan(self.jointPose[3])
    #print(distance)

  def picking_up(self):
    global pick_up, back_home
    if self.readyToMove==True:
      self.jointRequest.position[2]=self.jointPose[2]-0.6
      self.jointRequest.position[1]=self.jointPose[1]+0.5
      self.gripperRequest.position=[-0.01]# -0.01 represents closed
      self.setGripper(str(),self.gripperRequest,1.0)
      pick_up = False
      #back_home = True

  def go_home(self):
    self.jointRequest.position[0] = 0
    self.jointRequest.position[1] = 0
    self.jointRequest.position[2] = 0
    self.jointRequest.position[3] = 0

  # Find the normalised XY co-ordinate of a cube
  def getTarget(self,data):

    # Example = track the biggest red object
    area=[]
    coX=[]
    coY=[]


    # Get the red cubes
    for c in range(len(data.cubes)):
      if (data.cubes[c].cube_colour=='red'):
        area.append(data.cubes[c].area)
        coX.append(data.cubes[c].normalisedCoordinateX)
        coY.append(data.cubes[c].normalisedCoordinateY)



    # Find the biggest red cube
    if (len(area))>0:
      index_max = max(range(len(area)), key=area.__getitem__)
      self.targetX=coX[index_max]
      self.targetY=coY[index_max]
      self.targetArea=area[index_max]
      self.previous_area = area[index_max-1]
    else: # If you dont find a target, report the centre of the image to keep the camera still 
      # ADD PIVOT CODE
      self.targetX=0.5
      self.targetY=0.5


# Main 
def main(args):
  ic = cubeTracker()
  rospy.init_node('cube_tracker', anonymous=True)

  try:
    while not rospy.is_shutdown():
      if central == True:
        ic.aimCamera()
        ic.calc_Z()
      ic.move_towards()
      if pick_up == True:
        ic.picking_up()
      if back_home == True:
        ic.go_home()
      
  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)
