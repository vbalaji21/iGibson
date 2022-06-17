#!/usr/bin/env python3

# Brief: This clears the costmap every 1 second
# Author: Vignesh Balaji

import rospy
from std_srvs.srv import Empty

def main():
    rospy.init_node('clear_costmap', anonymous=True)
    rospy.wait_for_service('/move_base/clear_costmaps')
    s = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)

       
    r = rospy.Rate(1)
    while not rospy.is_shutdown():
        s()
        r.sleep()

if __name__ == "__main__":
    main()
