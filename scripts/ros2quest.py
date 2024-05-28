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
    
    # Frame to reference joystick motion
    self.p_reference = np.zeros(3)
    self.q_reference = np.zeros(4)
      
    # Subscribers
    self.ovr2ros_right_hand_pose_sub = rospy.Subscriber("/q2r_right_hand_pose", PoseStamped, self.ovr2ros_right_hand_pose_callback, queue_size=1)
    # self.ovr2ros_right_hand_twist_sub = rospy.Subscriber("/q2r_right_hand_twist",Twist,self.ovr2ros_right_hand_twist_callback)
    self.ovr2ros_right_hand_inputs_sub = rospy.Subscriber("/q2r_right_hand_inputs", OVR2ROSInputs, self.ovr2ros_right_hand_inputs_callback, queue_size=1)
    self.right_hand_prev_time = None
    self.right_hand_prev_pos = None

    # self.ovr2ros_left_hand_pose_sub = rospy.Subscriber("/q2r_left_hand_pose",PoseStamped,self.ovr2ros_left_hand_pose_callback)
    # self.ovr2ros_left_hand_twist_sub = rospy.Subscriber("/q2r_left_hand_twist",Twist,self.ovr2ros_left_hand_twist_callback)
    # self.ovr2ros_left_hand_inputs_sub = rospy.Subscriber("/q2r_left_hand_inputs",OVR2ROSInputs,self.ovr2ros_left_hand_inputs_callback)
    self.left_hand_prev_time = None

    # Publishers  
    # self.ros2ovr_dice_twist_pub =rospy.Publisher("/dice_twist", Twist, queue_size=1)
    # self.ros2ovr_q2r_twist_pub =rospy.Publisher("/q2r_twist", Twist, queue_size=1)
    self.pose_based_twist = rospy.Publisher("/pose_based_twist", Twist, queue_size=1)

    # Input values publisher
    # self.left_button_upper_pub = rospy.Publisher("/left_button_upper", Bool, queue_size=1)
    # self.left_button_lower_pub = rospy.Publisher("/left_button_lower", Bool, queue_size=1)
    # self.left_thumb_stick_horizontal_pub = rospy.Publisher("/left_thumb_stick_horizontal", Float32, queue_size=1)
    # self.left_thumb_stick_vertical_pub = rospy.Publisher("/left_thumb_stick_vertical", Float32, queue_size=1)
    # self.left_press_index_pub = rospy.Publisher("/left_press_index", Float32, queue_size=1)
    # self.left_press_middle_pub = rospy.Publisher("/left_press_middle", Float32, queue_size=1)

    self.right_button_upper_pub = rospy.Publisher("/right_button_upper", Bool, queue_size=1)
    self.right_button_lower_pub = rospy.Publisher("/right_button_lower", Bool, queue_size=1)
    self.right_thumb_stick_horizontal_pub = rospy.Publisher("/right_thumb_stick_horizontal", Float32, queue_size=1)
    self.right_thumb_stick_vertical_pub = rospy.Publisher("/right_thumb_stick_vertical", Float32, queue_size=1)
    self.right_press_index_pub = rospy.Publisher("/right_press_index", Float32, queue_size=1)
    self.right_press_middle_pub = rospy.Publisher("/right_press_middle", Float32, queue_size=1)

    self.right_hand_pos = np.zeros(3)
    self.right_hand_ori = np.zeros(4)
    self.right_hand_inputs = OVR2ROSInputs()
    self.prev_right_hand_pos = None

    self.left_hand_pos = np.zeros(3)
    self.left_hand_ori = np.zeros(4)
    self.left_hand_inputs = OVR2ROSInputs()
    self.prev_left_hand_pos = None


  #####################
  ### Ros callbacks ###
  #####################
  # Right hand 
  def ovr2ros_right_hand_pose_callback(self, data):
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

    if self.prev_right_hand_pos is not None:
      pos_delta = self.prev_right_hand_pos - self.right_hand_pos
      # cur_time = 
      time_delta = time.time() - self.prev_right_time
      twist = pos_delta / time_delta
      movement = np.sum(twist)
      if movement > 0.002:
        print(time_delta, twist, movement)
      else:
        twist = np.array([0.0, 0.0, 0.0])
      
      twist_msg = Twist()
      twist_msg.linear.x = twist[0]
      twist_msg.linear.y = twist[1]
      twist_msg.linear.z = twist[2]
      self.pose_based_twist.publish(twist_msg)

    self.prev_right_hand_pos = self.right_hand_pos
    self.prev_right_time = time.time()

  def ovr2ros_right_hand_inputs_callback(self, data):

    self.right_hand_inputs = data
    self.right_button_upper_pub.publish(data.button_upper)
    self.right_button_lower_pub.publish(data.button_lower)
    self.right_thumb_stick_horizontal_pub.publish(data.thumb_stick_horizontal)
    self.right_thumb_stick_vertical_pub.publish(data.thumb_stick_vertical)
    self.right_press_index_pub.publish(data.press_index)
    self.right_press_middle_pub.publish(data.press_middle)

  # # Left hand
  # def ovr2ros_left_hand_pose_callback(self, data):
  #   self.left_hand_pos = np.array([data.pose.position.x, data.pose.position.y, data.pose.position.z])
  #   self.left_hand_ori = np.array([data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w])
    
  #   left_hand_transform = geometry_msgs.msg.TransformStamped()
  #   left_hand_transform.header.stamp = rospy.Time.now()
  #   left_hand_transform.header.frame_id = "base"
  #   left_hand_transform.child_frame_id = "left_hand"
  #   left_hand_transform.transform.translation.x = self.left_hand_pos[0]
  #   left_hand_transform.transform.translation.y = self.left_hand_pos[1]
  #   left_hand_transform.transform.translation.z = self.left_hand_pos[2]
  #   left_hand_transform.transform.rotation.x = self.left_hand_ori[0]
  #   left_hand_transform.transform.rotation.y = self.left_hand_ori[1]
  #   left_hand_transform.transform.rotation.z = self.left_hand_ori[2]
  #   left_hand_transform.transform.rotation.w = self.left_hand_ori[3]
    
  #   self.br.sendTransform(left_hand_transform)

  # def ovr2ros_left_hand_inputs_callback(self, data):    
  #   self.left_hand_inputs = data
  #   self.left_button_upper_pub.publish(data.button_upper)
  #   self.left_button_lower_pub.publish(data.button_lower)
  #   self.left_thumb_stick_horizontal_pub.publish(data.thumb_stick_horizontal)
  #   self.left_thumb_stick_vertical_pub.publish(data.thumb_stick_vertical)
  #   self.left_press_index_pub.publish(data.press_index)
  #   self.left_press_middle_pub.publish(data.press_middle)

  #   #if the lower button is pressed send the twist back to the quest to move the dice ; 0 otherwise
  #   dice_twist=Twist()
  #   if self.left_hand_inputs.button_lower:
  #     dice_twist=self.left_hand_twist
  #   self.ros2ovr_dice_twist_pub.publish(dice_twist)

  #   #send the triggers as frequency and amplitude of vibration back to the quest
  #   left_hand_haptic_feedback = OVR2ROSHapticFeedback()
  #   left_hand_haptic_feedback.frequency = self.left_hand_inputs.press_index
  #   left_hand_haptic_feedback.amplitude = self.left_hand_inputs.press_middle



  def base2flange(self):
    base2flange = geometry_msgs.msg.TransformStamped()
    base2flange = self.tf_buffer.lookup_transform("base", "flange", rospy.Time(), rospy.Duration(10))
    self.base2flangePosition = np.array([base2flange.transform.translation.x, base2flange.transform.translation.y, base2flange.transform.translation.z])
    self.base2flangeOrientation = np.array([base2flange.transform.rotation.x, base2flange.transform.rotation.y, base2flange.transform.rotation.z, base2flange.transform.rotation.w])

def main(args):
  rospy.init_node('quest2rosdemo', anonymous=True)

  r2q = ros2quest()

  r = rospy.Rate(100) 

  while not rospy.is_shutdown():
    r.sleep()


if __name__ == '__main__':
    main(sys.argv)