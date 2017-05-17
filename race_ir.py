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
    global averageTime
    global rawTime
    global oneLoopTime
    global distanceList
    global rightDistance
    global leftDistance
    global fivePoint


    twist.linear.x = 0.4
    mixed_data = bin(data.data)
    big_sensor = int(mixed_data[-16:], 2)
    small_sensor = int(mixed_data[:-16], 2)




    if small_sensor >= 600:
        distanceList.append(big_sensor)
        userTime = time.time()
        if (userTime - rawTime[-1]) >= averageTime/2:
            fivePoint.append(big_sensor)
            if len(fivePoint) == 6:
                fivePoint=fivePoint[1:]
            elif len(fivePoint) >= 6:
                exit("fivePoint error")
            else:
                pass
            #right side at the front
            if (userTime - rawTime[-1]) <= averageTime*3/4:
                #print fivePoint,"1"
                rightDistance.append(big_sensor)
                averageDis = sum(rightDistance)/len(rightDistance)
                twist.linear.x = 0.2
                twist.angular.z = 0.02*(big_sensor - sum(fivePoint)/5)
                #print twist.angular.z, " re "
                if len(distanceList) == 0:
                    exit("Error scan right,distanceList used before assignment")
                elif big_sensor >= averageDis:# this value is charged by sensor.
                    twist.linear.x = 0.2
                    twist.angular.z = averageDis/470 + 0.02*(big_sensor - averageDis) + 0.02*(big_sensor - sum(fivePoint)/5)
                    #print twist.angular.z, " tw"
            #left side of the front
            else:
                #print fivePoint, "2"
                leftDistance.append(big_sensor)
                averageDis = sum(leftDistance)/len(leftDistance)
                twist.linear.x = 0.2
                twist.angular.z = -0.002*(big_sensor - sum(fivePoint)/5)
                if len(distanceList) == 0:
                    exit("Error scan right,distanceList used before assignment")
                elif big_sensor >= averageDis:
                    twist.linear.x = 0.2
                    twist.angular.z = -averageDis/470 + 0.02*(big_sensor - averageDis) + 0.02*(big_sensor - sum(fivePoint)/5)
                print twist.angular.z
    else:
        sub = time.time() - rawTime[-1]
        if sub > 0.1:
            rawTime.append(sub + rawTime[-1])
            oneLoopTime.append(sub)
            averageTime = sum(oneLoopTime)/len(oneLoopTime)
        fivePoint = [ ]

    # actually publish the twist message
    kobuki_velocity_pub.publish(twist)


def range_controller():

    # define the publisher globally
    global kobuki_velocity_pub
    #global scan_pub
    # initialize the node
    rospy.init_node('range_controller', anonymous=True)

    # initialize the publisher - to publish Twist message on the topic below...
    kobuki_velocity_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)

    #rospy.init_node('laser_scan_publisher')
    #scan_pub = rospy.Publisher('scan', LaserScan, queue_size = 50)
    # subscribe to the topic '/ir_data' of message type Int32. The function 'ir_callback' will be called
    # every time a new message is received - the parameter passed to the function is the message
    rospy.Subscriber("/ir_data", Int32, ir_callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

# start the line follow
if __name__ == '__main__':
    distanceList = [ ]
    rightDistance = [ ]
    leftDistance = [ ]
    rawTime = []
    fivePoint = [300,300,300,300,300]
    oneLoopTime = [0.9]
    averageTime = sum(oneLoopTime)/len(oneLoopTime)
    rawTime.append(time.time())
    range_controller()
