#!/usr/bin/env python
import rospy

# the message that we get from the arduino
from std_msgs.msg import Int32

# the output message controlling the speed and direction of the robot
from geometry_msgs.msg import Twist 

import time

def ir_callback(data):


    # Twist is a message type in ros, here we use an Twist message to control kobuki's speed
    # twist. linear.x is the forward velocity, if it is zero, robot will be static, 
    # if it is grater than 0, robot will move forward, otherwise, robot will move backward
    # twist.angular.axis is the rotatin velocity around the axis 
    # 
    # Around which axis do we have to turn? In wich direction will it turn with a positive value? 
    # Right hand coordinate system: x forward, y left, z up

    twist = Twist()
    twist.linear.x = 0.
    twist.angular.z = 0. 

    # write your code here
    twist.linear.x = 0.4
    mixed_data = bin(data.data)
    big_sensor = int(mixed_data[-16:], 2)
    small_sensor = int(mixed_data[:-16], 2)
    if small_sensor >= 600:
        userTime = time.time()
        if (userTime - rawTime[-1]) >= averageTime/2:
            if (userTime - rawTime[-1]) <= averageTime*3/4
                if big_sonser >= 400:
                    twist.liner.x = 0.2
                    twist.angular.z = big_sonser/720
            else:
                if big_sonser >= 400:
                    twist.liner.x = 0.2
                    twist.angular.z = -big_sonser/720
    else:
        one_time = time.time()
        sub = one_time - rawTime[-1]
        if sub > 0.2:
            rawTime.append(one_time())
            oneLoopTime.append(sub)
            averageTime = sum(oneLoopTime)/len(oneLoopTime)



    # actually publish the twist message
    kobuki_velocity_pub.publish(twist)  
    
    
def range_controller():
    
    # define the publisher globally
    global kobuki_velocity_pub
    
    # initialize the node
    rospy.init_node('range_controller', anonymous=True)

    # initialize the publisher - to publish Twist message on the topic below...
    kobuki_velocity_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)
    
    #rospy.init_node('laser_scan_publisher')
    scan_pub = rospy.Publisher('scan', LaserScan, queue_size = 50)
    # subscribe to the topic '/ir_data' of message type Int32. The function 'ir_callback' will be called
    # every time a new message is received - the parameter passed to the function is the message
    rospy.Subscriber("/ir_data", Int32, ir_callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

# start the line follow
if __name__ == '__main__':
    rawTime = [ ]
    oneLoopTime = [ ]
    averageTime = sum(oneLoopTime)/len(OneLoopTime)
    rawTime.append(time.time())
    range_controller()
    
