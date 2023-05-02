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
test = True

class cubeTracker:

  def __init__(self):

    # Where the block is in the image (start at the centre)
    self.targetX=0.5
    self.targetY=0.5
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
    self.gripperRequest.position=[-0.01]# -0.01 represents closed
    self.setGripper(str(),self.gripperRequest,1.0)

    rospy.sleep(1) # Wait for the gripper to close

    # Send the robot "home"
    self.jointRequest=JointPosition()
    self.jointRequest.joint_name=["joint1","joint2","joint3","joint4"]  
    self.jointRequest.position=[0.0,-1.05,0.357,1.8]
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
    if self.readyToMove==True: # If the robot state is not moving

      # Extremely simple - aim towards the target using joints [0] and [3]
      if (abs(self.targetY-0.5)>0.1 and self.jointPose[3] < 1.9):
        self.jointRequest.position[3]=self.jointPose[3]+(self.targetY-0.5)

      if (abs(self.targetX-0.5)>0.1 and self.jointPose[0] > -3.14):
        self.jointRequest.position[0]=self.jointPose[0]-(self.targetX-0.5)

    

      # This command sends the message to the robot
      self.setPose(str(),self.jointRequest,1.0)
      rospy.sleep(1) # Sleep after sending the service request as you can crash the robot firmware if you poll too fast

def fk():
  # Define the link lengths of the robot arm
  L1 = 1
  L2 = 2
  L3 = 3
  L4 = 4
  L5 = 5

  # Define the joint angles of the robot arm
  theta1 = math.radians(30)
  theta2 = math.radians(60)
  theta3 = math.radians(-90)
  theta4 = math.radians(45)
  theta5 = math.radians(-30)

  # Define the transformation matrices for each joint
  T1 = np.array([[math.cos(theta1), -math.sin(theta1), 0, 0],
                 [math.sin(theta1), math.cos(theta1), 0, 0],
                 [0, 0, 1, 0],
                 [0, 0, 0, 1]])

  T2 = np.array([[math.cos(theta2), -math.sin(theta2), 0, L2],
                 [math.sin(theta2), math.cos(theta2), 0, 0],
                 [0, 0, 1, 0],
                 [0, 0, 0, 1]])

  T3 = np.array([[math.cos(theta3), -math.sin(theta3), 0, L3],
                 [math.sin(theta3), math.cos(theta3), 0, 0],
                 [0, 0, 1, 0],
                 [0, 0, 0, 1]])

  T4 = np.array([[math.cos(theta4), -math.sin(theta4), 0, L4],
                 [math.sin(theta4), math.cos(theta4), 0, 0],
                 [0, 0, 1, 0],
                 [0, 0, 0, 1]])

  T5 = np.array([[math.cos(theta5), -math.sin(theta5), 0, L5],
                 [math.sin(theta5), math.cos(theta5), 0, 0],
                 [0, 0, 1, 0],
                 [0, 0, 0, 1]])

  # Calculate the transformation matrix from the base to the end effector
  T = T1.dot(T2).dot(T3).dot(T4).dot(T5)

  # Extract the position and orientation of the end effector
  position = T[:3, 3]
  orientation = T[:3, :3]

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
      ic.aimCamera()
      ic.move_towards()
  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)
