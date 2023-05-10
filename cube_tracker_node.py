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
from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink
from open_manipulator_msgs.msg import *
from open_manipulator_msgs.srv import *
detected = False
global result

test = True



# Assuming the real width of the block (in meters)
REAL_WIDTH = 0.03 # Adjust this value according to your block

# Assuming the focal length of the camera (in pixels)
FOCAL_LENGTH = 850  # You may need to calibrate your camera to find this value

class cubeTracker:
  def __init__(self):
    self.my_chain = Chain(name='my_chain', links=[
            OriginLink(),
            URDFLink(
              name="joint_1",
              bounds=[-3.14159, 3.14159],
              origin_translation=[0, 0, 1.57],
              origin_orientation=[0, 0, 0],
              rotation=[0, 0, 1],
            ),
            URDFLink(
              name="joint_2",
              bounds=[-3.14159, 3.14159],
              origin_translation=[0, 0, 1.57],
              origin_orientation=[0, 0, 0],
              rotation=[0, 1, 0],
            ),
            # Add more joints here
        ])

    
    # Where the block is in the image (start at the centre)
    self.targetX=0.5
    self.targetY=0.5
    self.targetZ=0.1
    self.targetArea=0
    self.previous_area=0
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
    self.jointRequest.position=[0,-1.352,0.379,1.906]
    #self.jointRequest.position=[-1.9,-1.352,0.379,1.906]
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
  def pick_up(self,result):
    not_at_target = True
    xchange = 0
    ychange = 0
    zchange = 0


    print("Picking Up")
    self.setPose = rospy.ServiceProxy('goal_joint_space_path_to_kinematics_pose', SetKinematicsPose)
    self.poseRequest=KinematicsPose()
    self.poseRequest.pose.position.x = 0.135
    self.poseRequest.pose.position.y = 0
    self.poseRequest.pose.position.z = 0.185
    self.setPose(str(),'gripper',self.poseRequest,3.0)
    rospy.sleep(1)

    xincrement = 5
    yincrement = 5
    zincrement = 5

    uncertainty = 0.05
    

    xdiff= abs((0.135 - result[0])/xincrement)
    ydiff= abs((0 - result[1])/yincrement)
    zdiff= abs((0.185 - result[2])/zincrement)
    while not_at_target==True:
      print("Result:")
      print(result)
      
      if result[0] < 0:
        if self.poseRequest.pose.position.x > result[0] + uncertainty:
          xchange = -xdiff
        elif self.poseRequest.pose.position.x < result[0] + uncertainty:
          xchange = 0  
      if result[0] > 0:
        if self.poseRequest.pose.position.x < result[0] - uncertainty:
          xchange = xdiff
        elif self.poseRequest.pose.position.x > result[0] - uncertainty:
          xchange = 0
      if result[1] < 0:
        if self.poseRequest.pose.position.y > result[1] + uncertainty:
          ychange = -ydiff
        elif self.poseRequest.pose.position.y < result[1] + uncertainty:
          ychange = 0
      if result[1] > 0:
        if self.poseRequest.pose.position.y < result[1] - uncertainty:
          ychange = ydiff
        elif self.poseRequest.pose.position.y > result[1] - uncertainty:
          ychange = 0
      if result[2] < 0:
        if self.poseRequest.pose.position.z > result[2] + uncertainty:
          zchange = -zdiff
        if self.poseRequest.pose.position.z < result[2] + uncertainty:
          zchange = 0
      if result[2] > 0:
        if self.poseRequest.pose.position.z < result[2] - uncertainty:
          zchange = zdiff
        elif self.poseRequest.pose.position.z > result[2] - uncertainty:
          zchange = 0
      



      
      if xchange==0 and ychange==0 and zchange==0:
        not_at_target=False
      print("Current X:")
      print(self.poseRequest.pose.position.x)
      print("Current yY:")
      print(self.poseRequest.pose.position.y)
      print("Current Z:")
      print(self.poseRequest.pose.position.z)
      print("xchange:")
      print(xchange)
      print("ychange:")
      print(ychange)
      print("zchange:")
      print(zchange)
      try:
        self.poseRequest.pose.position.x = self.poseRequest.pose.position.x + xchange
        self.poseRequest.pose.position.y = self.poseRequest.pose.position.y
        self.poseRequest.pose.position.z = self.poseRequest.pose.position.z
        self.setPose(str(),'gripper',self.poseRequest,5)
        rospy.sleep(5)
      except:
        self.poseRequest.pose.position.x = self.poseRequest.pose.position.x -xchange/2
        self.poseRequest.pose.position.y = self.poseRequest.pose.position.y
        self.poseRequest.pose.position.z = self.poseRequest.pose.position.z

      try:
        self.poseRequest.pose.position.x = self.poseRequest.pose.position.x
        self.poseRequest.pose.position.y = self.poseRequest.pose.position.y + ychange
        self.poseRequest.pose.position.z = self.poseRequest.pose.position.z
        self.setPose(str(),'gripper',self.poseRequest,5)
        rospy.sleep(5)
      except:
        self.poseRequest.pose.position.x = self.poseRequest.pose.position.x
        self.poseRequest.pose.position.y = self.poseRequest.pose.position.y - ychange/2
        self.poseRequest.pose.position.z = self.poseRequest.pose.position.z

      try:
        self.poseRequest.pose.position.x = self.poseRequest.pose.position.x
        self.poseRequest.pose.position.y = self.poseRequest.pose.position.y
        self.poseRequest.pose.position.z = self.poseRequest.pose.position.z + zchange
        self.setPose(str(),'gripper',self.poseRequest,5)
        rospy.sleep(5)
      except:
        self.poseRequest.pose.position.x = self.poseRequest.pose.position.x
        self.poseRequest.pose.position.y = self.poseRequest.pose.position.y
        self.poseRequest.pose.position.z = self.poseRequest.pose.position.z - zchange/2

      print("New X:")
      print(self.poseRequest.pose.position.x)
      print("New Y:")
      print(self.poseRequest.pose.position.y)
      print("New Z:")
      print(self.poseRequest.pose.position.z)
    '''

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
    '''

    #self.gripperRequest=JointPosition()
    #self.gripperRequest.joint_name=["gripper"]  
    #self.gripperRequest.position=[0.01]# -0.01 represents closed
    #self.setGripper(str(),self.gripperRequest,1.0)
    #rospy.sleep(5)


    #self.poseRequest=KinematicsPose()
    #self.poseRequest.pose.position.x = 0.239
    #self.poseRequest.pose.position.y = -0.111
    #self.poseRequest.pose.position.z = 0.118
    #self.setPose(str(),'gripper',self.poseRequest,2.0)
    #rospy.sleep(2)


    #self.poseRequest=KinematicsPose()
    #self.poseRequest.pose.position.x = 0.118
    #self.poseRequest.pose.position.y = -0.05
    #self.poseRequest.pose.position.z = 0.18
    #self.setPose(str(),'gripper',self.poseRequest,2.0)
    #rospy.sleep(2)
    #print('1/2')
    #self.poseRequest=KinematicsPose()
    #self.poseRequest.pose.position.x = (0.135+result[0])/2
    #self.poseRequest.pose.position.y = (0+result[1])/2
    #self.poseRequest.pose.position.z = (0.185+result[2])/2
    #self.setPose(str(),'gripper',self.poseRequest,5)
    #rospy.sleep(5)

    #print('1/2 done')
    #print(result)
    #self.poseRequest=KinematicsPose()
    #self.poseRequest.pose.position.x = result[0] 
    #self.poseRequest.pose.position.y = result[1]
    #self.poseRequest.pose.position.z = result[2] 
    #self.setPose(str(),'gripper',self.poseRequest,5)
    #rospy.sleep(5)
    #print('all done')
    #print(result)
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





    if (len(area))>0:
      index_max = max(range(len(area)), key=area.__getitem__)
      self.targetX=coX[index_max]
      self.targetY=coY[index_max]
      self.targetArea=area[index_max]
      self.previous_area = area[index_max-1]
      
      # Estimate depth (Z-coordinate)
      perceived_width = math.sqrt(self.targetArea)  # Assuming the block is square
      self.targetZ = (REAL_WIDTH * FOCAL_LENGTH) / perceived_width
      
      
  def calc_postion(self):
    print("calc_pos")
    def get_position_matrix(x, y, z):
        print("get_pos")
        # Let's say you have the coordinates x, y, z of the block in the camera frame
        # You need to transform these coordinates to the robot frame using a transformation matrix
        # The transformation matrix depends on the relative position and orientation of the camera with respect to the robot
        # For example, let's assume the camera is positioned at [1, 0, 0] with respect to the robot and there is no rotation
        transformation_matrix = np.array([
            [1, 0, 0, 1],
            [0, 1, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])
        T_base_to_gripper = np.array([[0, 0, 1, 0],
                              [1, 0, 0, 0],
                              [0, 1, 0, 0.16],
                              [0, 0, 0, 1]])
        # Convert the block coordinates to homogeneous coordinates
        block_coordinates_camera_frame = np.array([x, y, z, 1])

        # Transform the block coordinates to the robot frame
        block_coordinates_robot_frame = np.dot(transformation_matrix, block_coordinates_camera_frame)

        # Use IKPy to find the position matrix
        return block_coordinates_robot_frame
    block_coordinates_robot_frame = get_position_matrix(self.targetX, self.targetY, self.targetZ)
    def base_grip():
        print("base_grip")
        # Define the transformation matrices
        self.setPose = rospy.ServiceProxy('goal_joint_space_path', SetJointPosition)    
        self.jointRequest=JointPosition()
        self.jointRequest.joint_name=["joint1","joint2","joint3","joint4"] 
        θ1 = 0
        θ2 = -1.352
        θ3 = 0.379
        θ4 = 1.906
        d1 = 0.077
        a1 = 0
        a2 = 0.13
        a3 = 0.135
        a4 = 0.126

        T01 = np.array([[np.cos(θ1), 0, np.sin(θ1), 0],
                        [np.sin(θ1), 0, -np.cos(θ1), 0],
                        [0, 1, 0, d1],
                        [0, 0, 0, 1]])

        T12 = np.array([[np.cos(θ2), -np.sin(θ2), 0, a2*np.cos(θ2)],
                        [np.sin(θ2), np.cos(θ2), 0, a2*np.sin(θ2)],
                        [0, 0, 1, 0],
                        [0, 0, 0, 1]])

        T23 = np.array([[np.cos(θ3), -np.sin(θ3), 0, a3*np.cos(θ3)],
                        [np.sin(θ3), np.cos(θ3), 0, a3*np.sin(θ3)],
                        [0, 0, 1, 0],
                        [0, 0, 0, 1]])

        T34 = np.array([[np.cos(θ4), -np.sin(θ4), 0, a4*np.cos(θ4)],
                        [np.sin(θ4), np.cos(θ4), 0, a4*np.sin(θ4)],
                        [0, 0, 1, 0],
                        [0, 0, 0, 1]])

        # Multiply the matrices
        T04 = T01 @ T12 @ T23 @ T34
        return T04
    T04 = base_grip()
    global result
    result = T04@block_coordinates_robot_frame 
    result = result[:3]
    result[0] = result[0]
    result[1] = result[1]
    result[2] = result[2]
    return result

# Main 
def main(args):
  ic = cubeTracker()
  rospy.init_node('cube_tracker', anonymous=True)
  joint1 = -1.9
  try:
    while not rospy.is_shutdown():
      result = ic.calc_postion()
      if detected == False:
        #ic.search(joint1=joint1)
        joint1 += 0.1
      #if detected == True:
      #  print('Detected')
      print('just before pick up')
      print(result)
      ic.pick_up(result)
      break
      #ic.move_towards()
      #if pick_up == True:
      #  ic.picking_up()
      #if back_home == True:
      #  ic.go_home()
      
  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)
