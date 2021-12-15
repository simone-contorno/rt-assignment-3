#! /usr/bin/env python

# import ros stuff
import rospy
from std_srvs.srv import *

# service callback

def set_new_pos(req):
    print("Target reached! Please insert a new position")
    x = float(raw_input('x :'))
    y = float(raw_input('y :'))
    rospy.set_param("des_pos_x", x)
    rospy.set_param("des_pos_y", y)
    print("Thanks! Let's reach the next position")
    return []

def main():
    rospy.init_node('user_interface')

    x = rospy.get_param("des_pos_x")
    y = rospy.get_param("des_pos_y")
    print("Hi! We are reaching the first position: x = " +
          str(x) + ", y = " + str(y))
    srv = rospy.Service('user_interface', Empty, set_new_pos)
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    main()