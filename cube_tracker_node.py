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

from open_manipulator_msgs.msg import *
from open_manipulator_msgs.srv import *
detected = False


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
    self.jointPose=[0.136,0,0.236]

    # Create the subscribers
    self.image_sub = rospy.Subscriber('states',OpenManipulatorState,self.getStates)
    self.joint_state_sub = rospy.Subscriber('joint_states',JointState,self.getJoints)
    self.moving_sub = rospy.Subscriber('cubes',cubeArray,self.getTarget)

    # Create the service caller to move the robot
    self.setPose = rospy.ServiceProxy('goal_joint_space_path_to_kinematics_pose', SetKinematicsPose)
    self.setGripper = rospy.ServiceProxy('goal_tool_control', SetJointPosition)


    ## EXAMPLE POSES - Start by moving the robot to some example positions, and use the gripper



    # Send the robot "home"
    self.gripperRequest=JointPosition()
    self.gripperRequest.joint_name=["gripper"]  
    self.gripperRequest.position=[0.01]# -0.01 represents closed
    self.setGripper(str(),self.gripperRequest,1.0)
    rospy.sleep(1) # Wait for the arm to stand up


    self.poseRequest=KinematicsPose()
    self.poseRequest.pose.position.x = 0.134
    self.poseRequest.pose.position.y = 0
    self.poseRequest.pose.position.z = 0.239
    self.setPose(str(),'gripper',self.poseRequest,1.0)
    rospy.sleep(5)

    self.setPose = rospy.ServiceProxy('goal_joint_space_path', SetJointPosition)    
    self.jointRequest=JointPosition()
    self.jointRequest.joint_name=["joint1","joint2","joint3","joint4"]  
    self.jointRequest.position=[-1.9,-1.352,0.379,1.906]
    self.setPose(str(), self.jointRequest,5.0)
    rospy.sleep(5) #
    global joint1
    joint1 = -1.9



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
  def pick_up(self):
    print("Picking Up")
    self.setPose = rospy.ServiceProxy('goal_joint_space_path_to_kinematics_pose', SetKinematicsPose)
    self.poseRequest=KinematicsPose()
    self.poseRequest.pose.position.x = 0.135
    self.poseRequest.pose.position.y = 0
    self.poseRequest.pose.position.z = 0.185
    self.setPose(str(),'gripper',self.poseRequest,3.0)
    rospy.sleep(1)


    self.poseRequest=KinematicsPose()
    self.poseRequest.pose.position.x = 0.276
    self.poseRequest.pose.position.y = 0
    self.poseRequest.pose.position.z = 0.165
    self.setPose(str(),'gripper',self.poseRequest,3.0)
    rospy.sleep(5)

    self.poseRequest=KinematicsPose()
    self.poseRequest.pose.position.x = 0.276
    self.poseRequest.pose.position.y = 0
    self.poseRequest.pose.position.z = 0.1
    self.setPose(str(),'gripper',self.poseRequest,3.0)
    rospy.sleep(5)

    self.poseRequest=KinematicsPose()
    self.poseRequest.pose.position.x = 0.267
    self.poseRequest.pose.position.y = 0.002
    self.poseRequest.pose.position.z = 0.03
    self.setPose(str(),'gripper',self.poseRequest,3.0)
    rospy.sleep(5)

    self.gripperRequest=JointPosition()
    self.gripperRequest.joint_name=["gripper"]  
    self.gripperRequest.position=[-0.01]# -0.01 represents closed
    self.setGripper(str(),self.gripperRequest,1.0)
    rospy.sleep(1)

    self.poseRequest=KinematicsPose()
    self.poseRequest.pose.position.x = 0.292
    self.poseRequest.pose.position.y = 0
    self.poseRequest.pose.position.z = 0.128
    self.setPose(str(),'gripper',self.poseRequest,3.0)
    rospy.sleep(5)

    self.poseRequest=KinematicsPose()
    self.poseRequest.pose.position.x = 0.252
    self.poseRequest.pose.position.y = -0.147
    self.poseRequest.pose.position.z = 0.114
    self.setPose(str(),'gripper',self.poseRequest,3.0)
    rospy.sleep(5)

    self.poseRequest=KinematicsPose()
    self.poseRequest.pose.position.x = 0.249
    self.poseRequest.pose.position.y = -0.134
    self.poseRequest.pose.position.z = 0.035
    self.setPose(str(),'gripper',self.poseRequest,3.0)
    rospy.sleep(5)


    self.gripperRequest=JointPosition()
    self.gripperRequest.joint_name=["gripper"]  
    self.gripperRequest.position=[0.01]# -0.01 represents closed
    self.setGripper(str(),self.gripperRequest,1.0)
    rospy.sleep(5)


    self.poseRequest=KinematicsPose()
    self.poseRequest.pose.position.x = 0.239
    self.poseRequest.pose.position.y = -0.111
    self.poseRequest.pose.position.z = 0.118
    self.setPose(str(),'gripper',self.poseRequest,2.0)
    rospy.sleep(2)


    self.poseRequest=KinematicsPose()
    self.poseRequest.pose.position.x = 0.118
    self.poseRequest.pose.position.y = -0.05
    self.poseRequest.pose.position.z = 0.18
    self.setPose(str(),'gripper',self.poseRequest,2.0)
    rospy.sleep(2)

    self.poseRequest=KinematicsPose()
    self.poseRequest.pose.position.x = 0.134
    self.poseRequest.pose.position.y = 0
    self.poseRequest.pose.position.z = 0.239
    self.setPose(str(),'gripper',self.poseRequest,2.0)
    rospy.sleep(5)
    #if self.readyToMove==True: # If the robot state is not moving
    #  print(self.targetY)
    #  # Extremely simple - aim towards the target using joints [0] and [3]
    #  if (abs(self.targetY-0.5)>0.1):
    #    self.jointRequest.position[3]=self.jointPose[3]+(self.targetY-0.5)
    #    central = True#

     # if (abs(self.targetX-0.5)>0.1):
    #    self.jointRequest.position[0]=self.jointPose[0]-(self.targetX-0.5)
      #  central = True

      

      # This command sends the message to the robot
      #self.setPose(str(),self.jointRequest,1.0)
      #rospy.sleep(1) # Sleep after sending the service request as you can crash the robot firmware if you poll too fast
  
  
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
      global detected
      index_max = max(range(len(area)), key=area.__getitem__)
      self.targetX=coX[index_max]
      self.targetY=coY[index_max]
      self.targetArea=area[index_max]
      self.previous_area = area[index_max-1]
      if self.targetArea > 0:
        detected = True
    else: # If you dont find a target, report the centre of the image to keep the camera still 
      # ADD PIVOT CODE
      self.targetX=0.5
      self.targetY=0.5


# Main 
def main(args):
  ic = cubeTracker()
  rospy.init_node('cube_tracker', anonymous=True)
  joint1 = -1.9
  try:
    while not rospy.is_shutdown():
      if detected == False:
        ic.search(joint1=joint1)
        joint1 += 0.1
      if detected == True:
        print('Detected')
        ic.pick_up()
      #ic.move_towards()
      #if pick_up == True:
      #  ic.picking_up()
      #if back_home == True:
      #  ic.go_home()
      
  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)