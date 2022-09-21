#!/usr/bin/env python

"""
Does Transformation between two coordinate frames
In this particular case, Global UWB to Local Mavros
"""
  
import rospy
import os
from geometry_msgs.msg import PoseStamped,PoseWithCovarianceStamped
import tf
from tf.transformations import *
import numpy as np
from std_msgs.msg import String

class transform():

    def __init__(self, global_pos_topic ="/UAV" + str(os.getenv('DRONE_NUMBER')) + "PoseUWB", local_pos_topic = os.getenv('DRONE_NAME') + "/mavros/local_position/pose"):

        # Variables
        self.uav_mavros_pose = PoseStamped()
        self.uav_uwb_pose = PoseStamped()
        self.prev_pose = PoseStamped()

        self.recieved_new_ap_callback = False
        self.recieved_new_ap_callback_uav2 = False
        self.drone_name = "/uav" + os.getenv('DRONE_NUMBER')
        self.mavros_pose_topic = self.drone_name + "/mavros/local_position/pose"
        self.uwb_pose_topic = "/UAV" + os.getenv('DRONE_NUMBER') + "PoseUWB"
        self.uav_publish_topic = self.drone_name + "/control_manager/mavros_assigned_virtual_position"
        rospy.Subscriber(local_pos_topic,  PoseStamped, self.uav_mavros_callback)
        rospy.Subscriber(global_pos_topic,  PoseWithCovarianceStamped, self.uav_uwb_callback)


    def global_to_local(self, input_pose):
        # print("Looking up Global to Local")
        final_pose_uav  = PoseStamped()
        vector_diff_uav = PoseStamped()

        if self.uav_uwb_pose.pose.orientation.w==0:
            rospy.logwarn_once("UWB Positioning Not Initalised Yet")
            final_pose_uav = input_pose # In the case that UWB is not initalised, just use the current pose
        # Only do global to local lookup if new pose is sent, else just retrieve the prior point
        elif input_pose.pose.position.x != self.prev_pose.pose.position.x or input_pose.pose.position.y != self.prev_pose.pose.position.y or input_pose.pose.position.z != self.prev_pose.pose.position.z or input_pose.pose.orientation.w != self.prev_pose.pose.orientation.w or input_pose.pose.orientation.x != self.prev_pose.pose.orientation.x or input_pose.pose.orientation.y != self.prev_pose.pose.orientation.y or input_pose.pose.orientation.z != self.prev_pose.pose.orientation.z:
            vector_diff_uav = self.pose_diff(self.uav_uwb_pose, input_pose)
            final_pose_uav = self.pose_addition(vector_diff_uav, self.uav_mavros_pose)
            print("-> Corrected pose", "[%.3f, %.3f, %.3f, %.3f deg]" % (final_pose_uav.pose.position.x,final_pose_uav.pose.position.y,final_pose_uav.pose.position.z, math.degrees(euler_from_quaternion([final_pose_uav.pose.orientation.x,final_pose_uav.pose.orientation.y,final_pose_uav.pose.orientation.z,final_pose_uav.pose.orientation.w])[2])))
        else:
            final_pose_uav = self.prev_pose

        final_pose_uav.header.frame_id = '/odom'
        self.prev_pose=input_pose
        return final_pose_uav

    def uav_mavros_callback(self,data):
        self.uav_mavros_pose = data
    
    def uav_uwb_callback(self,data):
        self.uav_uwb_pose.pose = data.pose.pose
        self.uav_uwb_pose.header = data.header

    # rotation
    # Relative rotation q_r from q_1 to q_2
    # q_2 = q_r*q_1
    # therefore q_r = q_2*q_1_inverse
    def pose_diff(self, pose_stamped_previous, pose_staped_now):
        q1_inv = np.zeros(4)
        q2 = np.zeros(4)
        vector_diff = PoseStamped()

        # position vector diff 
        vector_diff.pose.position.x = pose_staped_now.pose.position.x - pose_stamped_previous.pose.position.x
        vector_diff.pose.position.y = pose_staped_now.pose.position.y - pose_stamped_previous.pose.position.y
        vector_diff.pose.position.z = pose_staped_now.pose.position.z - pose_stamped_previous.pose.position.z

        # print("Pose_stamped_now {} {} {}".format(pose_staped_now.pose.position.x, pose_staped_now.pose.position.y, pose_staped_now.pose.position.z))
        # print("Pose_stamped_previous  {} {} {}".format(pose_stamped_previous.pose.position.x, pose_stamped_previous.pose.position.y, pose_stamped_previous.pose.position.z))
        # print("Vector diff {} {} {}".format(vector_diff.pose.position.x, vector_diff.pose.position.y, vector_diff.pose.position.z))

        q1_inv[0] = pose_stamped_previous.pose.orientation.x
        q1_inv[1] = pose_stamped_previous.pose.orientation.y
        q1_inv[2] = pose_stamped_previous.pose.orientation.z
        q1_inv[3] = -pose_stamped_previous.pose.orientation.w # Negate for inverse

        q2[0] = pose_staped_now.pose.orientation.x
        q2[1] = pose_staped_now.pose.orientation.y
        q2[2] = pose_staped_now.pose.orientation.z
        q2[3] = pose_staped_now.pose.orientation.w

        qr = quaternion_multiply(q2, q1_inv)

        vector_diff.pose.orientation.x = qr[0]
        vector_diff.pose.orientation.y = qr[1]
        vector_diff.pose.orientation.z = qr[2]
        vector_diff.pose.orientation.w = qr[3]

        return vector_diff

    def pose_addition(self, vector_pose_stamped_to_add, pose_staped_now):
        q_rot = np.zeros(4)
        q_origin = np.zeros(4)
        new_pose_stamped = PoseStamped()

        # position vector diff 
        new_pose_stamped.pose.position.x = pose_staped_now.pose.position.x + vector_pose_stamped_to_add.pose.position.x
        new_pose_stamped.pose.position.y = pose_staped_now.pose.position.y + vector_pose_stamped_to_add.pose.position.y
        new_pose_stamped.pose.position.z = pose_staped_now.pose.position.z + vector_pose_stamped_to_add.pose.position.z

        q_origin[0] = pose_staped_now.pose.orientation.x
        q_origin[1] = pose_staped_now.pose.orientation.y
        q_origin[2] = pose_staped_now.pose.orientation.z
        q_origin[3] = pose_staped_now.pose.orientation.w

        q_rot[0] = vector_pose_stamped_to_add.pose.orientation.x
        q_rot[1] = vector_pose_stamped_to_add.pose.orientation.y
        q_rot[2] = vector_pose_stamped_to_add.pose.orientation.z
        q_rot[3] = vector_pose_stamped_to_add.pose.orientation.w

        q_new = quaternion_multiply(q_rot, q_origin)

        new_pose_stamped.pose.orientation.x = q_new[0]
        new_pose_stamped.pose.orientation.y = q_new[1]
        new_pose_stamped.pose.orientation.z = q_new[2]
        new_pose_stamped.pose.orientation.w = q_new[3]

        return new_pose_stamped

if __name__ == '__main__':
    rospy.init_node(os.getenv('DRONE_NAME') + '_Control_Manager')

    node = transform()

    rospy.spin()
