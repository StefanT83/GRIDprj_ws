#!/usr/bin/env python3

# Code description: Control 4 relays on the Pi Relay 6 add-on board (a hat) sitting on the RPi4B by hitting the 4 buttons on the right of a Sony DS4 joypad. The relays are then used to turn on/off 4 different light sources on the instrumented AgileX robot
# Inputs: /joy topic
# Outputs: relays on/off

import rospy
from std_msgs.msg import String

import RPi.GPIO as GPIO
import time

from sensor_msgs.msg import Joy 

pins         = [13,5,19,6,26,21] # cf https://learn.sb-components.co.uk/PiRelay-6 ;; pins[idx] corresp to: idx=0 is fwd light; idx=1 is bwd light; idx=2 is left light; idx=3 is right light; 
state_relays = [-1,-1,-1,-1,-1,-1]  #-1=undefined, 0=off, 1=on

def joy_callback(data):
    for i in data.axes:
        print(str(i) + " ")

    #def & ini
    idx = -1; #-1 is undefined

    if (data.axes)[7] == 1.0:
        print("fwd light")

        #choose what idx within state_relays corresponds to this light
        idx = 0;  

    elif (data.axes)[7] == -1.0:
        print("bwd light on")

        #choose what idx within state_relays corresponds to this light
        idx = 1;  


    if (data.axes)[6] == 1.0:
        print("left light on")

        #choose what idx within state_relays corresponds to this light
        idx = 2;  

    elif (data.axes)[6] == -1.0:
        print("right light on")

        #choose what idx within state_relays corresponds to this light
        idx = 3;  

    print("========")

    if idx>=0: #ensure one of the buttons of interest was pressed (not released, situation which would also correspond to a joy_callback giving the associated data.axes[.] a value 0 ) 
        if state_relays[idx] == 0:
            GPIO.output(pins[idx], GPIO.HIGH)
            state_relays[idx] = 1; #update
        elif state_relays[idx] == 1:
            GPIO.output(pins[idx], GPIO.LOW)
            state_relays[idx] = 0; #update

        time.sleep(0.075) # workaround to avoid quick on/off switching when holding one joystick button down; inspired by https://learn.sparkfun.com/tutorials/raspberry-gpio/python-rpigpio-example  

def talker():
#    pub = rospy.Publisher('chatter', String, queue_size=10)
#    rospy.init_node('talker', anonymous=True)
#    rate = rospy.Rate(10) # 10hz
#    while not rospy.is_shutdown():
#        hello_str = "hello world %s" % rospy.get_time()
#        rospy.loginfo(hello_str)
#        pub.publish(hello_str)
#        rate.sleep()


    GPIO.setmode(GPIO.BCM)

    #def & ini
    idx = 0; 

    for pin in pins: 
        GPIO.setup(pin,GPIO.OUT)

        GPIO.output(pin,GPIO.LOW)
##        time.sleep(1)
##        GPIO.output(pin, GPIO.HIGH)
##        time.sleep(1)
##        GPIO.output(pin, GPIO.LOW)
##        time.sleep(1)

        #conseq
        state_relays[idx] = 0;

        #update
        idx = idx + 1

    rospy.init_node('GRIDprj_joy_pirelay6_node', anonymous=True)
    rospy.Subscriber('/joy',Joy,joy_callback)
    rospy.spin()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
