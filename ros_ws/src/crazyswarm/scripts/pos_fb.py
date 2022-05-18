#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
import numpy as np

class pos_fb_updater:

    def __init__(self):

        num_cf = 2

    #     self.positions = np.full([3,num_cf],0.01)

    #     self.cf1_cb = rospy.Subscriber("/cf1/log1",GenericLogData, self.cf_1_cb, queue_size=1)
    #     self.cf2_cb = rospy.Subscriber("/cf2/log1",GenericLogData, self.cf_2_cb, queue_size=1)
    #     self.cf3_cb = rospy.Subscriber("/cf3/log1",GenericLogData, self.cf_3_cb, queue_size=1)
    #     self.cf4_cb = rospy.Subscriber("/cf4/log1",GenericLogData, self.cf_4_cb, queue_size=1)
    #     self.cf5_cb = rospy.Subscriber("/cf5/log1",GenericLogData, self.cf_5_cb, queue_size=1)


    # def cf_1_cb(self, pos_log_msg):

    #     self.positions[0][0] = pos_log_msg.values[0]
    #     self.positions[1][0] = pos_log_msg.values[1]
    #     self.positions[2][0] = pos_log_msg.values[2]

    # def cf_2_cb(self, pos_log_msg):

    #     self.positions[0][1] = pos_log_msg.values[0]
    #     self.positions[1][1] = pos_log_msg.values[1]
    #     self.positions[2][1] = pos_log_msg.values[2]

    # def cf_3_cb(self, pos_log_msg):

    #     self.positions[0][2] = pos_log_msg.values[0]
    #     self.positions[1][2] = pos_log_msg.values[1]
    #     self.positions[2][2] = pos_log_msg.values[2]

    # def cf_4_cb(self, pos_log_msg):

    #     self.positions[0][3] = pos_log_msg.values[0]
    #     self.positions[1][3] = pos_log_msg.values[1]
    #     self.positions[2][3] = pos_log_msg.values[2]

    # def cf_5_cb(self, pos_log_msg):

    #     self.positions[0][4] = pos_log_msg.values[0]
    #     self.positions[1][4] = pos_log_msg.values[1]
    #     self.positions[2][4] = pos_log_msg.values[2]        


        self.positions = np.full([3,num_cf],0.01)

        self.cf1_cb = rospy.Subscriber("/cf1/pose",PoseStamped, self.cf_1_cb, queue_size=1)
        self.cf2_cb = rospy.Subscriber("/cf2/pose",PoseStamped, self.cf_2_cb, queue_size=1)
        self.cf3_cb = rospy.Subscriber("/cf3/pose",PoseStamped, self.cf_3_cb, queue_size=1)
        self.cf4_cb = rospy.Subscriber("/cf4/pose",PoseStamped, self.cf_4_cb, queue_size=1)
        self.cf5_cb = rospy.Subscriber("/cf5/pose",PoseStamped, self.cf_5_cb, queue_size=1)


    def cf_1_cb(self, pos_log_msg):

        self.positions[0][0] = pos_log_msg.pose.position.x
        self.positions[1][0] = pos_log_msg.pose.position.y
        self.positions[2][0] = pos_log_msg.pose.position.z

    def cf_2_cb(self, pos_log_msg):

        self.positions[0][1] = pos_log_msg.pose.position.x
        self.positions[1][1] = pos_log_msg.pose.position.y
        self.positions[2][1] = pos_log_msg.pose.position.z

    def cf_3_cb(self, pos_log_msg):

        self.positions[0][2] = pos_log_msg.pose.position.x
        self.positions[1][2] = pos_log_msg.pose.position.y
        self.positions[2][2] = pos_log_msg.pose.position.z

    def cf_4_cb(self, pos_log_msg):

        self.positions[0][3] = pos_log_msg.pose.position.x
        self.positions[1][3] = pos_log_msg.pose.position.y
        self.positions[2][3] = pos_log_msg.pose.position.z

    def cf_5_cb(self, pos_log_msg):

        self.positions[0][4] = pos_log_msg.pose.position.x
        self.positions[1][4] = pos_log_msg.pose.position.y
        self.positions[2][4] = pos_log_msg.pose.position.z

    def get_positions(self):
        return self.positions


if __name__ == '__main__':
    
    rospy.init_node('pos_fb_listener', anonymous=True)


    fb_updater = pos_fb_updater()

    rospy.spin()

