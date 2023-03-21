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

from open_manipulator_msgs.msg import OpenManipulatorState
from open_manipulator_msgs.msg import JointPosition
from open_manipulator_msgs.srv import SetJointPosition
from open_manipulator_msgs.srv import GetJointPosition


class cubeTracker:
  
  def __init__(self):

    # Where the block is in the image (start at the centre)
    self.targetX=0.5
    self.targetY=0.5
    self.targetZ=0.5

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
    self.jointRequest.position=[0.0,-1.05,0.357,0.703]
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

      # Extremely simple - aim towards the target using joints [0], [2] and [3]
      if (abs(self.targetY-0.5)>0.1):
        self.jointRequest.position[3]=self.jointPose[3]+(self.targetY-0.5)

      if (abs(self.targetX-0.5)>0.1):
        self.jointRequest.position[0]=self.jointPose[0]-(self.targetX-0.5)

      # Adjust the z coordinate of the end effector
      if (abs(self.targetZ-0.5)>0.1):
        self.jointRequest.position[2] =self.jointPose[0]-(self.targetZ-0.5)  # Set a fixed z value for now

      # This command sends the message to the robot
      self.setPose(str(),self.jointRequest,1.0)
      rospy.sleep(1) # Sleep after sending the service request as you can crash the robot firmware if you poll too fast


  # Find the normalised XY co-ordinate of a cube
  def getTarget(self,data):

    # Example = track the biggest red object
    area=[]
    coX=[]
    coY=[]
    coZ=[]

    # Get the red cubes
    for c in range(len(data.cubes)):
      if (data.cubes[c].cube_colour=='red'):
        area.append(data.cubes[c].area)
        coX.append(data.cubes[c].normalisedCoordinateX)
        coY.append(data.cubes[c].normalisedCoordinateY)
        coZ.append(data.cubes[c].normalisedCoordinateZ)

      if (data.cubes[c].cube_colour=='blue'):
        area.append(data.cubes[c].area)
        coX.append(data.cubes[c].normalisedCoordinateX)
        coY.append(data.cubes[c].normalisedCoordinateY)
        coZ.append(data.cubes[c].normalisedCoordinateZ)
      
      if (data.cubes[c].cube_colour=='yellow'):
        area.append(data.cubes[c].area)
        coX.append(data.cubes[c].normalisedCoordinateX)
        coY.append(data.cubes[c].normalisedCoordinateY)
        coZ.append(data.cubes[c].normalisedCoordinateZ)

    # If no red cubes detected, set target to center
    if len(coX)==0:
      self.targetX=0.5
      self.targetY=0.5
      self.targetZ=0.2
    else:
      # Track the biggest red cube
      biggest=np.argmax(area)
      self.targetX=coX[biggest]
      self.targetY=coY[biggest]
      self.targetZ=coZ[biggest]

class Robot:
    def __init__(self, x, y, z, speed):
        self.x = x
        self.y = y
        self.z = z
        self.speed = speed
        
    def move(self, target_x, target_y, target_z):
        distance = math.sqrt((target_x - self.x)**2 + (target_y - self.y)**2 + (target_z - self.z)**2)
        steps = int(distance / self.speed)
        if steps == 0:
            self.x = target_x
            self.y = target_y
            self.z = target_z
        else:
            x_step = (target_x - self.x) / steps
            y_step = (target_y - self.y) / steps
            z_step = (target_z - self.z) / steps
            for _ in range(steps):
                self.x += x_step
                self.y += y_step
                self.z += z_step

# Main 
def main():
    # Initialize the robot and connect to the camera
    robot = Robot(0, 0, 0, 10)
    camera = connect_camera()
    
    while True:
        # Capture an image from the camera
        image = capture_image(camera)
        
        # Detect the cube
        cube = detect_cube(image)
        
        # If no cube is detected, continue searching
        if cube is None:
            print("Cube not found. Searching...")
            continue
        
        # If a cube is detected, move towards it
        print(f"Cube found. Moving towards ({cube[0]}, {cube[1]}, {cube[2]})...")
        robot.move(cube[0], cube[1], cube[2])
        print("Arrived at cube.")
        
if __name__ == '__main__':
    main(sys.argv)
