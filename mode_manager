#!/usr/bin/env python

## Publishes a topic with the the with/after tf transforms
  
import os
import rospy
import math
import os
from geometry_msgs.msg import PoseStamped,PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import tf
from tf2_msgs.msg import TFMessage
from std_msgs.msg import Int32, Float64, String

class transform():

    def __init__(self):

        # Mode
        self.mode = String()
        rospy.Subscriber('/hri_mode', String, self.mode_callback)

        # Follow Me
        self.follow_me_pos = PoseWithCovarianceStamped() ########################
        rospy.Subscriber('/uav_all/follow_me_target_pose',  PoseWithCovarianceStamped, self.position_callback)

        # Go There
        self.uav1_go_there_pos = PoseWithCovarianceStamped()
        rospy.Subscriber('/uav1/command/pose',  PoseWithCovarianceStamped, self.uav1_go_there_callback)
        # self.uav1_go_there_yaw = Float64()
        # rospy.Subscriber('/uav1/command/yaw',  Float64, self.uav1_go_there_yaw_callback)
        self.uav2_go_there_pos = PoseWithCovarianceStamped()
        rospy.Subscriber('/uav2/command/pose',  PoseWithCovarianceStamped, self.uav2_go_there_callback)

        # Publish topic for planner
        uav1_publish_topic = "/uav1/teaming_planner/assigned_virtual_position"
        uav2_publish_topic = "/uav2/teaming_planner/assigned_virtual_position"
        uav1_publisher = rospy.Publisher(uav1_publish_topic , PoseStamped,queue_size=1)
        uav2_publisher = rospy.Publisher(uav2_publish_topic , PoseStamped,queue_size=1)

        # Local Position Callback
        self.uav1_pos = PoseStamped()
        rospy.Subscriber('/uav1/mavros/local_position/pose',  PoseStamped, self.uav1_callback)
        self.uav2_pos = PoseStamped()
        rospy.Subscriber('/uav2/mavros/local_position/pose',  PoseStamped, self.uav2_callback)
        self.uav1_go_there_pos.pose.pose.position.z=0 #initalise to 0 for takeover
        self.uav2_go_there_pos.pose.pose.position.z=0

        rate = rospy.Rate(20.0)

        while not rospy.is_shutdown():

            # print(self.mode.data)

            cmd1 = PoseStamped()
            cmd2 = PoseStamped()

            if self.mode.data=="Follow_Me":
                print("FM")
                cmd1.pose.position.x=self.follow_me_pos.pose.pose.position.x - 3.5
                cmd1.pose.position.y=self.follow_me_pos.pose.pose.position.y + 1.5
                cmd1.pose.position.z=1

                cmd1.pose.orientation.x=self.follow_me_pos.pose.pose.orientation.x
                cmd1.pose.orientation.y=self.follow_me_pos.pose.pose.orientation.y
                cmd1.pose.orientation.z=self.follow_me_pos.pose.pose.orientation.z
                cmd1.pose.orientation.w=self.follow_me_pos.pose.pose.orientation.w

                cmd2.pose.position.x=self.follow_me_pos.pose.pose.position.x - 3.5
                cmd2.pose.position.y=self.follow_me_pos.pose.pose.position.y - 2
                cmd2.pose.position.z=1

                cmd2.pose.orientation.x=self.follow_me_pos.pose.pose.orientation.x
                cmd2.pose.orientation.y=self.follow_me_pos.pose.pose.orientation.y
                cmd2.pose.orientation.z=self.follow_me_pos.pose.pose.orientation.z
                cmd2.pose.orientation.w=self.follow_me_pos.pose.pose.orientation.w

            elif self.mode.data=="Go_There":
                print("GT")
                # If there is go there position sent, go to
                if self.uav1_go_there_pos.pose.pose.position.z != 0:
                    cmd1.pose.position.x=self.uav1_go_there_pos.pose.pose.position.x
                    cmd1.pose.position.y=self.uav1_go_there_pos.pose.pose.position.y
                    cmd1.pose.position.z=1

                    cmd1.pose.orientation.x=self.uav1_go_there_pos.pose.pose.orientation.x
                    cmd1.pose.orientation.y=self.uav1_go_there_pos.pose.pose.orientation.y
                    cmd1.pose.orientation.z=self.uav1_go_there_pos.pose.pose.orientation.z
                    cmd1.pose.orientation.w=self.uav1_go_there_pos.pose.pose.orientation.w
                # If there is no position sent yet, hover at current spot
                else:
                    cmd1.pose.position.x=self.uav1_pos.pose.position.x
                    cmd1.pose.position.y=self.uav1_pos.pose.position.y
                    cmd1.pose.position.z=self.uav1_pos.pose.position.z

                    cmd1.pose.orientation.x=self.uav1_pos.pose.orientation.x
                    cmd1.pose.orientation.y=self.uav1_pos.pose.orientation.y
                    cmd1.pose.orientation.z=self.uav1_pos.pose.orientation.z
                    cmd1.pose.orientation.w=self.uav1_pos.pose.orientation.w

                #-------------------------------------------------------------------------------

                # If there is go there position sent, go to
                if self.uav2_go_there_pos.pose.pose.position.z != 0:
                    cmd2.pose.position.x=self.uav2_go_there_pos.pose.pose.position.x
                    cmd2.pose.position.y=self.uav2_go_there_pos.pose.pose.position.y
                    cmd2.pose.position.z=1

                    cmd2.pose.orientation.x=self.uav2_go_there_pos.pose.pose.orientation.x
                    cmd2.pose.orientation.y=self.uav2_go_there_pos.pose.pose.orientation.y
                    cmd2.pose.orientation.z=self.uav2_go_there_pos.pose.pose.orientation.z
                    cmd2.pose.orientation.w=self.uav2_go_there_pos.pose.pose.orientation.w
                # If there is no position sent yet, hover at current spot
                else:
                    cmd2.pose.position.x=self.uav2_pos.pose.position.x
                    cmd2.pose.position.y=self.uav2_pos.pose.position.y
                    cmd2.pose.position.z=self.uav2_pos.pose.position.z

                    cmd2.pose.orientation.x=self.uav2_pos.pose.orientation.x
                    cmd2.pose.orientation.y=self.uav2_pos.pose.orientation.y
                    cmd2.pose.orientation.z=self.uav2_pos.pose.orientation.z
                    cmd2.pose.orientation.w=self.uav2_pos.pose.orientation.w

            else:
                print("No Mode")

                cmd1.pose.position.x=self.uav1_pos.pose.position.x
                cmd1.pose.position.y=self.uav1_pos.pose.position.y
                cmd1.pose.position.z=self.uav1_pos.pose.position.z

                cmd1.pose.orientation.x=self.uav1_pos.pose.orientation.x
                cmd1.pose.orientation.y=self.uav1_pos.pose.orientation.y
                cmd1.pose.orientation.z=self.uav1_pos.pose.orientation.z
                cmd1.pose.orientation.w=self.uav1_pos.pose.orientation.w

                cmd2.pose.position.x=self.uav2_pos.pose.position.x
                cmd2.pose.position.y=self.uav2_pos.pose.position.y
                cmd2.pose.position.z=self.uav2_pos.pose.position.z

                cmd2.pose.orientation.x=self.uav2_pos.pose.orientation.x
                cmd2.pose.orientation.y=self.uav2_pos.pose.orientation.y
                cmd2.pose.orientation.z=self.uav2_pos.pose.orientation.z
                cmd2.pose.orientation.w=self.uav2_pos.pose.orientation.w
                


            print("1:",[cmd1.pose.position.x,cmd1.pose.position.y,cmd1.pose.position.z])
            print("2:",[cmd2.pose.position.x,cmd2.pose.position.y,cmd2.pose.position.z])
            uav1_publisher.publish(cmd1)
            uav2_publisher.publish(cmd2)

            rate.sleep()


    def position_callback(self,data):
        self.follow_me_pos=data

    def mode_callback(self,data):
        self.mode=data



    def uav1_go_there_callback(self,data):
        self.uav1_go_there_pos=data

    def uav2_go_there_callback(self,data):
        self.uav2_go_there_pos=data


    
    def uav1_callback(self,data):
        self.uav1_pos=data
    
    def uav2_callback(self,data):
        self.uav2_pos=data

if __name__ == '__main__':
    rospy.init_node('Mock_Multi_Agent')

    node = transform()

    rospy.spin()