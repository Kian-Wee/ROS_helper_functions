#!/usr/bin/env python
  
import rospy
import os
from geometry_msgs.msg import PoseStamped,PoseWithCovarianceStamped
import tf
from tf.transformations import *
import numpy as np
from std_msgs.msg import String

# Latest control manager code.
# Doesn't do anymore global to local lookup.

class transform():

    def __init__(self):

        # Variables
        self.cmd = PoseStamped()
        self.uav_ap_uwb = PoseStamped()
        self.uav_mavros_pose = PoseStamped()
        self.uav_uwb_pose = PoseStamped()
        self.input_pose_stamped = PoseStamped()
        self.mode = String()

        self.recieved_new_ap_callback = False
        self.recieved_new_ap_callback_uav2 = False
        self.drone_name = "/uav" + os.getenv('DRONE_NUMBER')
        self.ap_uwb_pose_topic = self.drone_name + "/control_manager/input"
        self.mavros_pose_topic = self.drone_name + "/mavros/local_position/pose"
        self.uwb_pose_topic = "/UAV" + os.getenv('DRONE_NUMBER') + "PoseUWB"
        self.uav_publish_topic = self.drone_name + "/control_manager/mavros_assigned_virtual_position"
        # self.uav_publish_topic = self.drone_name + "/mavros/setpoint_position/local"
        self.uav_mode_topic = "/uav" + os.getenv('DRONE_NUMBER') + "/hri_mode"
        self.uav_input_pose_topic = "/uav" + os.getenv('DRONE_NUMBER') + "/input_pose_stamped"

        rospy.Subscriber(self.ap_uwb_pose_topic,  PoseStamped, self.uav_ap_uwb_callback)
        rospy.Subscriber(self.mavros_pose_topic,  PoseStamped, self.uav_mavros_callback)
        rospy.Subscriber(self.uwb_pose_topic,  PoseWithCovarianceStamped, self.uav_uwb_callback)
        rospy.Subscriber(self.uav_mode_topic,  String, self.hri_mode_callback)
        rospy.Subscriber(self.uav_input_pose_topic,  PoseStamped, self.uav_input_pose_topic_callback)

        mavros_ap_publisher = rospy.Publisher(self.uav_publish_topic , PoseStamped,queue_size=1)

        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            final_pose_uav  = PoseStamped()
            vector_diff_uav = PoseStamped()

            if self.recieved_new_ap_callback:
                if self.mode == "Go_There": 
                    # vector_diff_uav = self.pose_diff(self.uav_uwb_pose, self.uav_ap_uwb)
                    # final_pose_uav = self.pose_addition(vector_diff_uav, self.uav_mavros_pose)
                    # self.cmd = final_pose_uav
                    # self.cmd.pose.orientation = self.uav_ap_uwb.pose.orientation # Do not transform orientation
                    self.cmd = self.uav_ap_uwb 
                    
                elif self.mode == "Follow_Me":
                    # In Follow me mode, the formation generation algorithm will generate some unstable orientation due to the human orientation
                    # Being quite unstable, thus the drone should always look forward
                    # vector_diff_uav = self.pose_diff(self.uav_uwb_pose, self.uav_ap_uwb)
                    # final_pose_uav = self.pose_addition(vector_diff_uav, self.uav_mavros_pose)
                    # self.cmd = final_pose_uav
                    self.cmd = self.uav_ap_uwb
                    self.cmd.pose.orientation.x = 0
                    self.cmd.pose.orientation.y = 0
                    self.cmd.pose.orientation.z = 0
                    self.cmd.pose.orientation.w = 1
                else:
                    self.cmd = self.uav_ap_uwb # default
                self.recieved_new_ap_callback = False

            self.cmd.pose.position.z = 1.2
            self.cmd.header.frame_id = '/odom'
            mavros_ap_publisher.publish(self.cmd)

            rate.sleep()

    def uav_ap_uwb_callback(self,data):
        if (self.uav_ap_uwb.pose.position.x != data.pose.position.x or self.uav_ap_uwb.pose.position.y != data.pose.position.y or self.uav_ap_uwb.pose.position.z != data.pose.position.z or self.uav_ap_uwb.pose.orientation.x != data.pose.orientation.x or self.uav_ap_uwb.pose.orientation.y != data.pose.orientation.y or self.uav_ap_uwb.pose.orientation.z != data.pose.orientation.z or self.uav_ap_uwb.pose.orientation.w != data.pose.orientation.w):
            self.recieved_new_ap_callback = True
            self.uav_ap_uwb = data
        else:
            self.recieved_new_ap_callback = False

    def uav_mavros_callback(self,data):
        self.uav_mavros_pose = data

    def uav_input_pose_topic_callback(self,data):
        self.input_pose_stamped = data
    
    def uav_uwb_callback(self,data):
        self.uav_uwb_pose.pose = data.pose.pose
        self.uav_uwb_pose.header = data.header
    
    def hri_mode_callback(self, data):
        self.mode = data.data

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
    rospy.init_node('Control_Manager')

    node = transform()

    rospy.spin()
