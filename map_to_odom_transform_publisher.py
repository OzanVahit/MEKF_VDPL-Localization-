#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import tf.transformations as tft
import tf2_ros
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class TransformPublisher:
    def __init__(self):
        rospy.init_node('map_to_odom_transform_publisher', anonymous=True)
        
        self.mekf_pose = None
        self.odom = None
        
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        
        self.mekf_sub = rospy.Subscriber('/MEKF_pose', PoseWithCovarianceStamped, self.mekf_pose_callback)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        
        self.rate = rospy.Rate(29)  # 29 Hz
        
    def mekf_pose_callback(self, data):
        self.mekf_pose = data
        
    def odom_callback(self, data):
        self.odom = data
        
    def publish_transform(self):
        if self.mekf_pose and self.odom:
            # Position differences
            x_diff = self.mekf_pose.pose.pose.position.x - self.odom.pose.pose.position.x
            y_diff = self.mekf_pose.pose.pose.position.y - self.odom.pose.pose.position.y
            z_diff = (self.mekf_pose.pose.pose.position.z - self.odom.pose.pose.position.z)

            
            # Orientation differences
            mekf_orientation_q = [self.mekf_pose.pose.pose.orientation.w, self.mekf_pose.pose.pose.orientation.x, self.mekf_pose.pose.pose.orientation.y,self.mekf_pose.pose.pose.orientation.z]
            
            odom_orientation_q = [self.odom.pose.pose.orientation.w, self.odom.pose.pose.orientation.x, self.odom.pose.pose.orientation.y, self.odom.pose.pose.orientation.z]
            odom_orientation_q_conjugate = [odom_orientation_q[0], -odom_orientation_q[1], -odom_orientation_q[2], -odom_orientation_q[3]]
            
         
            orientation_diff_q = tft.quaternion_multiply(mekf_orientation_q, odom_orientation_q_conjugate)
            
            t = TransformStamped()
            t.header.stamp = rospy.Time.now()
            t.header.frame_id = "map"
            t.child_frame_id = "odom"
            t.transform.translation.x = x_diff
            t.transform.translation.y = y_diff
            t.transform.translation.z = z_diff
            t.transform.rotation.x = orientation_diff_q[0]
            t.transform.rotation.y = orientation_diff_q[1]
            t.transform.rotation.z = orientation_diff_q[2]
            t.transform.rotation.w = orientation_diff_q[3]

            
            self.tf_broadcaster.sendTransform(t)
            
    def run(self):
        while not rospy.is_shutdown():
            self.publish_transform()
            self.rate.sleep()


if __name__ == '__main__':
    try:
        transform_publisher = TransformPublisher()
        transform_publisher.run()
    except rospy.ROSInterruptException as e:
        rospy.logerr(e)
	pass
