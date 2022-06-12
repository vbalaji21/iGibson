#!/usr/bin/env python3

# Brief: This function publishes /joint_states for tucked_arm_pose of PR2, necessary for displaying robot robot_description in Rviz


import rospy
from sensor_msgs.msg import Joy
from igibson.objects.pedestrian import Pedestrian
import pybullet as p

x_linear_vel_increment = 0.1
y_linear_vel_increment = 0.1
y_angular_vel_increment = 0.1

def callback(data):

    print("\033[2;31;43m callback for subscriber \n", data) 
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.header)
    #TODO: Update the received data to pedestrians command velocity / twist 

    global x_linear_vel_increment
    global y_linear_vel_increment
    global y_angular_vel_increment

    ped = Pedestrian(style=(0))



    linear_x = x_linear_vel_increment * data.buttons[2] # blue X button
    linear_y = y_linear_vel_increment * data.buttons[3] # blue Y button
    angular_y = y_angular_vel_increment * data.buttons[1] #red green button

    linear_velocity = [linear_x , linear_y , 0] # linear velocity along X and Y
    angular_velocity = [0, angular_y , 0]    # angular velocity along Y


    ped.human_velocity(linear_velocity, angular_velocity)
    p.connect(p.SHARED_MEMORY, key=0)
    p.resetBaseVelocity(objectUniqueId=3 , linearVelocity=linear_velocity, angularVelocity=angular_velocity, physicsClientId=0)

def main():
    rospy.init_node('human_teleop', anonymous=True)
    print("\033[2;31;43m loop running \n")
    rospy.Subscriber("/joy", Joy, callback)   #subscribe to joy publisher  #check the publisher name and data type

    rospy.spin()

if __name__ == "__main__":
    main()
