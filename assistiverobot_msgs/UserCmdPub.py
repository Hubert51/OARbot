#!/usr/bin/env python2

import rospy
from std_msgs.msg import String, Int8
import threading

class UserCmdPub(object):
    """docstring for talker"""
    def __init__(self):
        self.val = 0

        self.start = True
    #     self._t_effector = threading.Thread(target=self.run)
    #     self._t_effector.daemon = True
    #     self._t_effector.start()

    # def run(self):
    #     while True:
    #         input1 = raw_input("Please input cmd => ")
    #         if input1 == '1':
    #             self.start = False
    #             break
    #         elif input1.isdigit():
    #             self.val = int(input1)
    #         else:
    #             print "Unknown Command"

        

    def talker(self):
        pub = rospy.Publisher('chatter', Int8, queue_size=1)
        rospy.init_node('talker', anonymous=True)
        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():            
            rospy.loginfo(self.val)
            pub.publish(self.val)
            rate.sleep()

if __name__ == '__main__':
    # try:
    UserCmdPub()
    # except rospy.ROSInterruptException:
    #     pass