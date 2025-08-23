#!/usr/bin/env python
import getch
import rospy
from std_msgs.msg import String #String message
from std_msgs.msg import Int8

################################
# created by yuvaram
# yuvaramsingh94@gmail.com
################################

def keys():
    pub = rospy.Publisher('key', Int8, queue_size=10) # "key" is the publisher name
    rospy.init_node('keypress', anonymous=True)
    rate = rospy.Rate(10) # try removing this line and see what happens

    while not rospy.is_shutdown():
        # Obtain the ASCII value of the keypress event from the keyboard,
        # joypad, or joystick.
        s = input('')

        if s:
            k = ord(s[0])
        else:
            k = 10

        # Print the obtained ASCII value in the terminal and publish it on the
        # given topic.
        rospy.loginfo(str(k))
        pub.publish(k)

if __name__=='__main__':
    try:
        keys()
    except rospy.ROSInterruptException:
        pass

