#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
# the message that we get from the arduino
from std_msgs.msg import Int32
import roslib; roslib.load_manifest('kobuki_testsuite')
from kobuki_msgs.msg import ButtonEvent
from kobuki_msgs.msg import BumperEvent
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

    # write your code here
    global averageTime
    global scan_pub
    global rawTime
    global oneLoopTime
    global distanceList
    global rightDistance
    global leftDistance
    global fivePoint
    global state
    global twist

    twist = Twist()

    global scan
    global ranges
    global counter
    current_time = rospy.Time.now()
    scan = LaserScan()
    scan.header.stamp = current_time
    scan.header.frame_id = 'laser_frame'
    scan.angle_min = -3.14
    scan.angle_max = 3.14
    scan.angle_increment = 0.13
    scan.time_increment = 0.8
    scan.range_min = 0.0
    scan.range_max = 1024.0
    scan.ranges = []
    scan.intensities = [999]

    if state == 0:
        twist.linear.x = 0.18
        twist.angular.z = 0.0
        mixed_data = bin(data.data)
        small_sensor = int(mixed_data[-16:], 2)
        big_sensor = int(mixed_data[:-16], 2)
        ranges.append(big_sensor)
        scan.ranges = ranges
        counter+=1
        #print small_sensor
        #print big_sensor
        if small_sensor >= 820:
            #print small_sensor
            distanceList.append(big_sensor)
            userTime = time.time()
            if (userTime - rawTime[-1]) >= averageTime/2:
                fivePoint.append(big_sensor)
                if len(fivePoint) == 5:
                    fivePoint=fivePoint[1:]
                elif len(fivePoint) > 5:
                    exit("fivePoint error")
                else:
                    pass
                #right side at the front
                if (userTime - rawTime[-1]) <= averageTime*3/4:
                    rightDistance.append(big_sensor)
                    averageDis = sum(rightDistance)/len(rightDistance)
                    twist.linear.x = 0.18
                    twist.angular.z = 0
                    #twist.angular.z = 0.01*(big_sensor - sum(fivePoint)/5)
                    #print twist.angular.z, " re "
                    if len(distanceList) == 0:
                        exit("Error scan right,distanceList used before assignment")
                    #elif twist.angular.z >=  0.1:
                    elif big_sensor >= 300:# this value is charged by sensor.
                        twist.linear.x = 0.1
                        if big_sensor >= 320:
                            twist.angular.z = 0.7 + 0.008*(big_sensor - averageDis) + 0.002*abs(big_sensor-sum(fivePoint)/len(fivePoint))
                        #print twist.angular.z, " tw"
                        elif big_sensor >= 340:
                            twist.angular.z = 0.76 + 0.008*(big_sensor - averageDis) + 0.002*abs(big_sensor-sum(fivePoint)/len(fivePoint))
                        #print twist.angular.z, " tw"
                        elif big_sensor >= 360:
                            twist.angular.z = 0.8 + 0.008*(big_sensor - averageDis) + 0.002*abs(big_sensor-sum(fivePoint)/len(fivePoint))
                        #print twist.angular.z, " tw"
                        elif big_sensor >= 400:
                            twist.angular.z = 0.9 + 0.012*(big_sensor - averageDis) + 0.004*abs(big_sensor-sum(fivePoint)/len(fivePoint))
                        #print twist.angular.z, " tw"

                    #print twist.angular.z

                    else:
                        pass
                #left side of the front
                else:
                    leftDistance.append(big_sensor)
                    averageDis = sum(leftDistance)/len(leftDistance)
                    twist.linear.x = 0.18
                    twist.angular.z =0
                    #twist.angular.z = -0.01*(big_sensor - sum(fivePoint)/5)
                    if len(distanceList) == 0:
                        exit("Error scan right,distanceList used before assignment")
                    #elif twist.angular.z <= -0.1:
                    elif big_sensor >= 300:
                        twist.linear.x = 0.1
                        if big_sensor >= 320:
                            twist.angular.z = -0.7 - 0.008*(big_sensor - averageDis)  - 0.002*abs(big_sensor-sum(fivePoint)/len(fivePoint))
                        #print twist.angular.z, " tw"
                        elif big_sensor >= 340:
                            twist.angular.z = -0.76 - 0.008*(big_sensor - averageDis) - 0.002*abs(big_sensor-sum(fivePoint)/len(fivePoint))
                        #print twist.angular.z, " tw"
                        elif big_sensor >= 360:
                            twist.angular.z = -0.8 - 0.008*(big_sensor - averageDis) - 0.002*abs(big_sensor-sum(fivePoint)/len(fivePoint))
                        #print twist.angular.z, " tw"
                        elif big_sensor >= 400:
                            twist.angular.z = -0.9 - 0.012*(big_sensor - averageDis) - 0.004*abs(big_sensor-sum(fivePoint)/len(fivePoint))
                        #print twist.angular.z, " tw"
                    else:
                        pass
                allSub = sum(rightDistance) -sum(leftDistance)
                if len(rawTime) > 2:
                    if allSub > 10000:
                        twist.linear.x = 0.05
                        twist.angular.z = 0.8*allSub/10000
                    elif -allSub > 10000:
                        twist.linear.x = 0.05
                        twist.angular.z = 0.8*allSub/10000
                    else:
                        pass
        else:
            sub = time.time() - rawTime[-1]
            if sub > 0.05:
                print sum(rightDistance), sum(leftDistance)
                rawTime.append(sub + rawTime[-1])
                oneLoopTime.append(sub)
                averageTime = sum(oneLoopTime)/len(oneLoopTime)
                rightDistance = []
                leftDistance = []
                scan_pub.publish(scan)
                #print ranges
                ranges = []
                counter = 0
                #print "one loop", sub, small_sensor
                fivePoint = [ ]

        #print twist.angular.z, big_sensor
        #print ranges
        kobuki_velocity_pub.publish(twist)
    else:
        pass

def BumperEventCallback(data):
    global state
    global twist
    if ( data.state == BumperEvent.RELEASED ) :
        pass
    else:
        twist.angular.z = 0
        twist.linear.x = 0
        state = 1
        kobuki_velocity_pub.publish(twist)

def ButtonEventCallback(data):
    global state
    if ( data.button == ButtonEvent.Button0 ) :
        if ( data.state == ButtonEvent.RELEASED ) :
            pass
        else:
             state = 0

def range_controller():

    # define the publisher globally
    global kobuki_velocity_pub
    global scan_pub
    # initialize the node
    rospy.init_node('range_controller', anonymous=True)

    # initialize the publisher - to publish Twist message on the topic below...
    kobuki_velocity_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)

    #rospy.init_node('laser_scan_publisher')
    scan_pub = rospy.Publisher('scan', LaserScan, queue_size = 50)
    # subscribe to the topic '/ir_data' of message type Int32. The function 'ir_callback' will be called
    # every time a new message is received - the parameter passed to the function is the message
    rospy.Subscriber("/ir_data", Int32, ir_callback)
    rospy.Subscriber("/mobile_base/events/button",ButtonEvent,ButtonEventCallback)
    rospy.Subscriber("/mobile_base/events/bumper",BumperEvent,BumperEventCallback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

# start the line follow
if __name__ == '__main__':
    counter = 0
    ranges = []
    state = 0
    distanceList = [ ]
    rightDistance = [ ]
    leftDistance = [ ]
    rawTime = []
    fivePoint = [280,280,280,280]
    oneLoopTime = [0.81]
    averageTime = sum(oneLoopTime)/len(oneLoopTime)
    rawTime.append(time.time())
    range_controller()
