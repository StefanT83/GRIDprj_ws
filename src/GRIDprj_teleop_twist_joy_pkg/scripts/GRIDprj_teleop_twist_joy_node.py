#!/usr/bin/env python3

# Code description: Basic teleop control of the AgileX Scout 2.0 robot
# Inputs: /joy topic
# Outputs: /cmd_vel topic

import rospy
from std_msgs.msg import String

from sensor_msgs.msg import Joy 
from geometry_msgs.msg import Twist

AgileX_absvmax     = 0.5 #[m/s]
AgileX_absomegamax = 3.14/2.0 #[rad/s]

# global variables
rospy.init_node('GRIDprj_teleop_twist_joy_node', anonymous=True)
cmd_vel_pub_ = rospy.Publisher('/cmd_vel',Twist,queue_size=1)

def joy_callback(data):
    for i in data.axes:
        print(str(i) + " ")

    vd_nz = (data.axes)[4]
    omegad_nz = (data.axes)[3]

    ## SC algo
    # WIP, future work, for now this functionality is disabled

    vr_nz     = vd_nz; 
    omegar_nz = omegad_nz;
        
    #ROS
    cmd_vel_msg = Twist()
    cmd_vel_msg.linear.x  =  vr_nz * AgileX_absvmax
    cmd_vel_msg.angular.z =  omegar_nz * AgileX_absomegamax

    cmd_vel_pub_.publish(cmd_vel_msg)


def talker():
    rospy.Subscriber('/joy',Joy,joy_callback)
    rospy.spin()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
