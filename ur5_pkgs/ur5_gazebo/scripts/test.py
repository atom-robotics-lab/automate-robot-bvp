#! /usr/bin/env python

import rospy
from std_msgs.msg import String

def test_cb(msg):
    print(msg)
    rospy.sleep(2)
    print("SO CHUKA HUN BHAI MAIN")


def main():
    rospy.init_node("test_node")

    rospy.Subscriber("test_topic", String, test_cb)

    rate = rospy.Rate(10)
    counter = 0
    
    while not rospy.is_shutdown():
        print("Sahi hai bhai {}".format(counter))
        counter = counter + 1
        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
