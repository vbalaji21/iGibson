#!/usr/bin/env python3

# Brief: This publishes tike taken to reach goal and minimum human robot distance
# Author: Vignesh Balaji

import rospy
from std_msgs.msg import String
from datetime import datetime
from actionlib_msgs.msg import GoalStatusArray

global my_data
global flag1
global flag2
global flag3
flag1 = True
flag2 = True
flag3 = False

#### Internal utility functions
def calculate_time_in_sec(hour, min, sec, microsec):

    time = ((((hour*60)+min)*60)+sec) + (microsec/1000000)
    return time

def unix_timestamp_to_sec(unix_time_stamp):
    dt = datetime.fromtimestamp(unix_time_stamp)
    return calculate_time_in_sec(dt.hour, dt.minute, dt.second, dt.microsecond)

#########################

def diff_in_sec(unix_time_stamp_old, unix_time_stamp_new):
    return unix_timestamp_to_sec(unix_time_stamp_new) - unix_timestamp_to_sec(unix_time_stamp_old)










def callback(data):
    global my_data
    my_data = data
    global flag1
    global flag2
    global flag3
    global start
    global end

    #my_data.status_list[0].status
    #rospy.loginfo(my_data.goal_id.secs)
    #my_data.status_list[0].goal_id.stamp.nsecs
    #rospy.loginfo(my_data.status_list[0].goal_id.stamp.secs)
    # rospy.loginfo("flag1 %d",flag1)
    # # rospy.loginfo("flag2 %d",flag2)
    # rospy.loginfo("flag3 %d",flag3)
    # rospy.loginfo(data.status_list[0].status)
    if(data.status_list[-1].status == 1 and flag1==True and flag3 == False):
        rospy.loginfo("start")
        start = data.status_list[-1].goal_id.stamp.secs
        flag1 = False
        flag3 = True
        rospy.loginfo("start %f",start)

        
    if(data.status_list[-1].status == 3 and flag2==True and flag3== True):
        end = data.status_list[-1].goal_id.stamp.secs
        flag2 = False
        flag3 = False
        rospy.loginfo("end %f",end)

    if(flag1==False and flag2==False):
        rospy.loginfo("result")
        a = diff_in_sec(start, end)
        rospy.loginfo("answer %f",a)
        flag1 = True
        flag2 = True
        flag3 = False

    return 1
#    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

def main():
    pub = rospy.Publisher("exp_goal_data", String, queue_size=10)       
    rospy.init_node('goal_data_publisher', anonymous=True)
#    rospy.init_node('listener_goal_data_extraction', anonymous=True)

    r = rospy.Rate(50)

    a = rospy.get_time()
    b = rospy.get_time() 

    start = 0
    end = 0

    goal_data = rospy.Subscriber("/move_base/status", GoalStatusArray, callback)

#    while not rospy.is_shutdown():
#        hello_str = "hello world %s" % rospy.get_time()
#        rospy.loginfo(hello_str)
    rospy.spin()










#        rospy.loginfo(goal_data)



#        rospy.loginfo(diff_in_sec)
#        pub.publish(hello_str)
#        r.sleep()

if __name__ == "__main__":
    main()