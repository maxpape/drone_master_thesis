#!/usr/bin/env python

import rospy
import tf2_ros
import numpy as np
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from tf.transformations import *

def pose_callback(pose_msg):

    # Convert the original orientation to a list
    #original_orientation = [
    #    pose_msg.pose.orientation.x,
    #    pose_msg.pose.orientation.y,
    #    pose_msg.pose.orientation.z,
    #    pose_msg.pose.orientation.w
    #]

    # Add a static rotation around the x-axis of 90 degrees
    #static_rotation = quaternion_from_euler(-np.pi/2,0, 0)  # 90 degrees in radians
    #static_rotation = quaternion_from_euler(0,0, 0)  # 90 degrees in radians

    # Multiply the original quaternion by the static rotation quaternion
    #rotated_quaternion = quaternion_multiply(original_orientation, static_rotation)




    # Create a TransformStamped message
    transform = TransformStamped()
    #transform.header.stamp = rospy.Time.now()
    transform.header.stamp = pose_msg.header.stamp
    transform.header.frame_id = "/map"  # Change to your reference frame
    transform.child_frame_id = "/base_link"  # Change to your child frame
    
    transform.transform.translation = pose_msg.pose.pose.position
    transform.transform.rotation = pose_msg.pose.pose.orientation
    #transform.transform.rotation.x = rotated_quaternion[0]
    #transform.transform.rotation.y = rotated_quaternion[1]
    #transform.transform.rotation.z = rotated_quaternion[2]
    #transform.transform.rotation.w = rotated_quaternion[3]
#

    # Publish the TF message
    tf_broadcaster.sendTransform(transform)
    

if __name__ == '__main__':
    rospy.init_node('pose_to_tf_node')

    # Set up TF broadcaster
    tf_broadcaster = tf2_ros.TransformBroadcaster()

    # Set up PoseStamped subscriber
    pose_topic = "/mavros/global_position/local"  # Change to your PoseStamped topic
    rospy.Subscriber(pose_topic, Odometry, pose_callback)

    rospy.spin()
