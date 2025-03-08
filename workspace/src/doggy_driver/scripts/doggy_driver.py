#!/usr/bin/env python

from __future__ import print_function
#import roslib
import roslib; roslib.load_manifest('doggy_driver')
import rospy

import numpy as np

from trajectory_msgs.msg import JointTrajectory

#from doggy.HardwareInterface import HardwareInterface

class ServoInterface:
    def __init__(self,topic_name='joint_group_position_controller/command'):
        self.subscriber = rospy.Subscriber(
            topic_name, JointTrajectory,
            self.cmd_callback, queue_size = 1)
        #self.hardware_interface = HardwareInterface()
        # rospy.loginfo(f"Subscribed to {topic_name}")

    def cmd_callback(self, msg):
        joint_positions = msg.points[0].positions
        lf1_position = joint_positions[0]
        lf2_position = joint_positions[1]
        lf3_position = joint_positions[2]
        rf1_position = joint_positions[3]
        rf2_position = joint_positions[4]
        rf3_position = joint_positions[5]
        lb1_position = joint_positions[6]
        lb2_position = joint_positions[7]
        lb3_position = joint_positions[8]
        rb1_position = joint_positions[9]
        rb2_position = joint_positions[10]
        rb3_position = joint_positions[11]

        joint_angles = np.array([
            [rf1_position, lf1_position, rb1_position, lb1_position],
            [rf2_position, lf2_position, rb2_position, lb2_position],
            [rf2_position + rf3_position, lf2_position + lf3_position,
             rb2_position + rb3_position, lb2_position + lb3_position]
        ])
        #print(joint_angles)
        
        # Radiant to degree conversion
        joint_angles = np.degrees(joint_angles)
        print(joint_angles)
        
        #self.hardware_interface.set_actuator_positions(joint_angles)


if __name__ == "__main__":
    rospy.init_node('doggy_driver')
    servo_interface_node = ServoInterface()
    rospy.spin()
