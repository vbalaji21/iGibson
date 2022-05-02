#!/usr/bin/env python3

# Brief: This function publishes /joint_states for tucked_arm_pose of PR2, necessary for displaying robot robot_description in Rviz
# Author: Phani Teja Singamaneni

import rospy
from sensor_msgs.msg import JointState

def main():
    rospy.init_node('stage_joints', anonymous=True)
    joint_states = JointState()
    joint_states.header.frame_id = "/joint_states"
    joint_states.name = ["torso_lift_joint","shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint", "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint","wrist_roll_joint"]


    joint_states.position = [0.02, 1.1707963267948966, 1.4707963267948965, -0.4, 1.6707963267948966, 0.0, 1.5707963267948966, 0.0]
    #tucking joint data got from igibson/robots/fethc.py, data got from joint mapping to ids, joint ids (3, 5,6 .... 11) 
       
    r = rospy.Rate(50)
    joint_states_pub = rospy.Publisher("/stage_joint_states", JointState, queue_size=1)
    while not rospy.is_shutdown():
        joint_states.header.stamp = rospy.Time.now()
        joint_states_pub.publish(joint_states)
        r.sleep()

if __name__ == "__main__":
    main()
