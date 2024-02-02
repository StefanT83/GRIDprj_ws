#!/usr/bin/env python3

# Code description: Use joystick to actuate the dynamixel motor (connected to the whinch system)
# Inputs: /joy topic
# Outputs: none, just the motor spinning

# Code inspired from DynamixelSDK/ros/dynamixel_sdk_examples/src/read_write_node.py

import os
import rospy
from dynamixel_sdk import *

from sensor_msgs.msg import Joy 

import time

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

# Control table address: //all values here and below cf https://emanual.robotis.com/docs/en/dxl/x/xh540-v270/#operating-mode
ADDR_TORQUE_ENABLE      = 64               # Control table address is different in Dynamixel model
ADDR_OPERATING_MODE     = 11  
ADDR_MAX_POSITION_LIMIT = 48
ADDR_MIN_POSITION_LIMIT = 52
ADDR_GOAL_VELOCITY      = 104

# Protocol version
PROTOCOL_VERSION            = 2.0               # See which protocol version is used in the Dynamixel

# Default setting
DXL_ID                      = 1                 # Dynamixel ID : 1
BAUDRATE                    = 57600             # Dynamixel default baudrate : 57600
DEVICENAME                  = '/dev/ttyUSB0'    # Check which port is being used on your controller
                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

TORQUE_ENABLE               = 1                 # Value for enabling the torque

#global variables
portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

#def
GoalVelocity_setpoint = 20; # units are s.t. a value of 32 corresponds to approx 1 rot every 8 seconds

#ini joy buttons positions
data_buttons_2_prev = 0.0
data_buttons_0_prev = 0.0

def setGoalVelocity(value):
    #Motor: set Goal Velocity to specific value; e.g. value = 32 corresponds to approx 1 rot every 8 seconds
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_VELOCITY, value)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        print("Press any key to terminate...")
        getch()
        quit()
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
        print("Press any key to terminate...")
        getch()
        quit()
    else:
        print("DYNAMIXEL motor: set Goal Velocity to specific value successfully")


def setTorqueEnable(value):
    # Enable/Disable Dynamixel Torque; value can be 0 or 1 (0 = Disable, 1 = Enable)
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, value)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        print("Press any key to terminate...")
        getch()
        quit()
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
        print("Press any key to terminate...")
        getch()
        quit()
    else:
        print("DYNAMIXEL motor: Enable Torque successful")

def joy_callback(data):
    print("Axes:")
    for i in data.axes:
        print(str(i) + " ")

    print("Buttons:")
    for i in data.buttons:
        print(str(i) + "")

    print("=====")

    global data_buttons_0_prev, data_buttons_2_prev

    if (data.buttons)[2] == 1.0 and (data_buttons_2_prev != 1.0):
        print("Rotate motor: lower the capsule")
        setGoalVelocity(GoalVelocity_setpoint)
    elif (data.buttons)[2] == 0.0 and (data_buttons_2_prev != 0.0):
        print("Motor asked to stand still")
        setGoalVelocity(0)
    elif (data.buttons)[0] == 1.0 and data_buttons_0_prev != 1.0:
        print("Rotate motor: higher the capsule")
        setGoalVelocity(-GoalVelocity_setpoint)
    elif (data.buttons)[0] == 0.0 and data_buttons_0_prev != 0.0:
        print("Motor asked to stand still")
        setGoalVelocity(0)
    
    #store & update
    data_buttons_0_prev = (data.buttons)[0]
    data_buttons_2_prev = (data.buttons)[2]
        

def main():
    # Open port
    try:
       portHandler.openPort()
       print("Succeeded to open the port")
    except:
        print("Failed to open the port")
        print("Press any key to terminate...")
        getch()
        quit()

    # Set port baudrate
    try:
        portHandler.setBaudRate(BAUDRATE)
        print("Succeeded to change the baudrate")
    except:
        print("Failed to change the baudrate")
        print("Press any key to terminate...")
        getch()
        quit()

    # Prerequisite for Velocity Control Mode: Disable Dynamixel Torque
    setTorqueEnable(0)

    #Motor: set Operating Mode to Velocity Control Mode i.e. Wheel Mode(endless)
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_OPERATING_MODE, 1)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        print("Press any key to terminate...")
        getch()
        quit()
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
        print("Press any key to terminate...")
        getch()
        quit()
    else:
        print("DYNAMIXEL motor: Operating mode successfully switched to Velocity Control Mode i.e. Wheel Mode(endless)")

     #Motor: set Max Position Limit to 0  
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID, ADDR_MAX_POSITION_LIMIT, 0)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        print("Press any key to terminate...")
        getch()
        quit()
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
        print("Press any key to terminate...")
        getch()
        quit()
    else:
        print("DYNAMIXEL motor: set Max Position Limit to 0 successfully")

    #Motor: set Min Position Limit to 0
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID, ADDR_MIN_POSITION_LIMIT, 0)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        print("Press any key to terminate...")
        getch()
        quit()
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
        print("Press any key to terminate...")
        getch()
        quit()
    else:
        print("DYNAMIXEL motor: set Min Position Limit to 0 successfully")


    setTorqueEnable(1)

    setGoalVelocity(32)
    time.sleep(2)
    setGoalVelocity(-32)
    time.sleep(2.0)
    setGoalVelocity(0)

    #ROS
    rospy.init_node('GRIDprj_dynamixel_joy_node')
    rospy.Subscriber('/joy',Joy,joy_callback)
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
