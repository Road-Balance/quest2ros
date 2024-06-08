#!/usr/bin/env python3
import rospy
import sys

from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster, Buffer, TransformListener, LookupException, ConnectivityException
from tf_conversions import transformations

from quest2ros.msg import OVR2ROSInputs, OVR2ROSHapticFeedback
from geometry_msgs.msg import PoseStamped, Twist, geometry_msgs
from std_msgs.msg import Float32
from std_msgs.msg import Bool

import numpy as np
import time

class ros2quest:

  def __init__(self):

    # TF broadcaster
    self.br = TransformBroadcaster()
    
    # TF listener  
    self.tf_buffer = Buffer()
    self.tf_listener = TransformListener(self.tf_buffer)
      
    # ROS Subscribers

    # right hand
    self.ovr2ros_right_hand_pose_sub = rospy.Subscriber(
      "/q2r_right_hand_pose", 
      PoseStamped, 
      self.right_hand_pose_cb, 
      queue_size=1
    )
    self.ovr2ros_right_hand_inputs_sub = rospy.Subscriber(
      "/q2r_right_hand_inputs", 
      OVR2ROSInputs, 
      self.right_hand_inputs_cb, 
      queue_size=1
    )

    # left hand
    self.ovr2ros_left_hand_pose_sub = rospy.Subscriber(
      "/q2r_left_hand_pose",
      PoseStamped, 
      self.left_hand_pose_cb,
      queue_size=1
    )
    self.ovr2ros_left_hand_inputs_sub = rospy.Subscriber(
      "/q2r_left_hand_inputs",
      OVR2ROSInputs,
      self.left_hand_inputs_cb,
      queue_size=1
    )

    # Publishers for Input values
    # Right hand
    self.right_button_upper_pub = rospy.Publisher("/right_button_upper", Bool, queue_size=1)
    self.right_button_lower_pub = rospy.Publisher("/right_button_lower", Bool, queue_size=1)
    self.right_thumb_stick_horizontal_pub = rospy.Publisher("/right_thumb_stick_horizontal", Float32, queue_size=1)
    self.right_thumb_stick_vertical_pub = rospy.Publisher("/right_thumb_stick_vertical", Float32, queue_size=1)
    self.right_press_index_pub = rospy.Publisher("/right_press_index", Float32, queue_size=1)
    self.right_press_middle_pub = rospy.Publisher("/right_press_middle", Float32, queue_size=1)

    # Left hand
    self.left_button_upper_pub = rospy.Publisher("/left_button_upper", Bool, queue_size=1)
    self.left_button_lower_pub = rospy.Publisher("/left_button_lower", Bool, queue_size=1)
    self.left_thumb_stick_horizontal_pub = rospy.Publisher("/left_thumb_stick_horizontal", Float32, queue_size=1)
    self.left_thumb_stick_vertical_pub = rospy.Publisher("/left_thumb_stick_vertical", Float32, queue_size=1)
    self.left_press_index_pub = rospy.Publisher("/left_press_index", Float32, queue_size=1)
    self.left_press_middle_pub = rospy.Publisher("/left_press_middle", Float32, queue_size=1)

    self.right_hand_pos = np.zeros(3)
    self.right_hand_ori = np.zeros(4)
    self.right_hand_inputs = OVR2ROSInputs()

    self.left_hand_pos = np.zeros(3)
    self.left_hand_ori = np.zeros(4)
    self.left_hand_inputs = OVR2ROSInputs()

  # Right hand topic CB
  def right_hand_pose_cb(self, data):
    self.right_hand_pos = np.array([data.pose.position.x, data.pose.position.y, data.pose.position.z])
    self.right_hand_ori = np.array([data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w])

    # For debugging purposes, we are printing also the pose of the hand wrt world
    right_hand_transform = geometry_msgs.msg.TransformStamped()
    right_hand_transform.header.stamp = rospy.Time.now()
    right_hand_transform.header.frame_id = "base"
    right_hand_transform.child_frame_id = "right_hand"
    right_hand_transform.transform.translation.x = self.right_hand_pos[0]
    right_hand_transform.transform.translation.y = self.right_hand_pos[1]
    right_hand_transform.transform.translation.z = self.right_hand_pos[2]
    right_hand_transform.transform.rotation.x = self.right_hand_ori[0]
    right_hand_transform.transform.rotation.y = self.right_hand_ori[1]
    right_hand_transform.transform.rotation.z = self.right_hand_ori[2]
    right_hand_transform.transform.rotation.w = self.right_hand_ori[3]
    
    self.br.sendTransform(right_hand_transform)

  def right_hand_inputs_cb(self, data):
    self.right_hand_inputs = data
    self.right_button_upper_pub.publish(data.button_upper)
    self.right_button_lower_pub.publish(data.button_lower)
    self.right_thumb_stick_horizontal_pub.publish(data.thumb_stick_horizontal)
    self.right_thumb_stick_vertical_pub.publish(data.thumb_stick_vertical)
    self.right_press_index_pub.publish(data.press_index)
    self.right_press_middle_pub.publish(data.press_middle)

  # Left hand topic CB
  def left_hand_pose_cb(self, data):
    self.left_hand_pos = np.array([data.pose.position.x, data.pose.position.y, data.pose.position.z])
    self.left_hand_ori = np.array([data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w])
    
    left_hand_transform = geometry_msgs.msg.TransformStamped()
    left_hand_transform.header.stamp = rospy.Time.now()
    left_hand_transform.header.frame_id = "base"
    left_hand_transform.child_frame_id = "left_hand"
    left_hand_transform.transform.translation.x = self.left_hand_pos[0]
    left_hand_transform.transform.translation.y = self.left_hand_pos[1]
    left_hand_transform.transform.translation.z = self.left_hand_pos[2]
    left_hand_transform.transform.rotation.x = self.left_hand_ori[0]
    left_hand_transform.transform.rotation.y = self.left_hand_ori[1]
    left_hand_transform.transform.rotation.z = self.left_hand_ori[2]
    left_hand_transform.transform.rotation.w = self.left_hand_ori[3]
    
    self.br.sendTransform(left_hand_transform)

  def left_hand_inputs_cb(self, data):    
    self.left_hand_inputs = data
    self.left_button_upper_pub.publish(data.button_upper)
    self.left_button_lower_pub.publish(data.button_lower)
    self.left_thumb_stick_horizontal_pub.publish(data.thumb_stick_horizontal)
    self.left_thumb_stick_vertical_pub.publish(data.thumb_stick_vertical)
    self.left_press_index_pub.publish(data.press_index)
    self.left_press_middle_pub.publish(data.press_middle)

def main(args):
  rospy.init_node('quest2ros_demo_node', anonymous=True)

  r2q = ros2quest()
  r = rospy.Rate(100) 

  while not rospy.is_shutdown():
    r.sleep()

if __name__ == '__main__':
    main(sys.argv)