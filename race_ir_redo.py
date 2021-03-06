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
    global pose

    twist = Twist()

    global scan
    global ranges
    global counter
    global sumTopDis
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
        twist.linear.x = 0.2
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
            if averageTime*25.0/51.0 <=(userTime - rawTime[-1]) < averageTime/2.0:
                twist.linear = 0.2
                twist.angular.z = 0
            if (userTime - rawTime[-1]) >= averageTime/2.0:
                fivePoint.append(big_sensor)
                if len(fivePoint) == 5:
                    fivePoint=fivePoint[1:]
                elif len(fivePoint) > 5:
                    exit("fivePoint error")
                else:
                    pass
                #right side at the front
                if (userTime - rawTime[-1]) <= averageTime*3.00/4.00:
                    rightDistance.append(big_sensor)
                    averageDis = sum(rightDistance)/len(rightDistance)
                    twist.linear.x = 0.2
                    twist.angular.z = 0             
                    """if big_sensor >= 310:# this value is charged by sensor.
                        twist.linear.x = 0.0
                        twist.angular.z = big_sensor/310 + 0.006*(big_sensor - averageDis) + 0.01*abs(big_sensor-sum(fivePoint)/len(fivePoint))
                    """
                #left side of the front
                elif averageTime*3.00/4.00 <(userTime - rawTime[-1])< averageTime * 50.0/51.0:
                    leftDistance.append(big_sensor)
                    averageDis = sum(leftDistance)/len(leftDistance)
                    twist.linear.x = 0.2
                    twist.angular.z =0
                    """if big_sensor >= 290:
                        twist.linear.x = 0.
                        twist.angular.z = -big_sensor/280 - 0.002*(big_sensor - averageDis)  - 0.01*abs(big_sensor-sum(fivePoint)/len(fivePoint))
                allSub = sum(rightDistance) -sum(leftDistance)
                if sum(sumTopDis) > 30000:
                    if len(rawTime) > 4:
                        if allSub > 26500:
                            print "zuozuozuo"
                            twist.linear.x = 0.05
                            twist.angular.z = 2.4*allSub/15000
                        elif -allSub > 26500:
                            print "youyouyou"
                            twist.linear.x = 0.05
                            twist.angular.z = 2.4*allSub/15000
                        else:
                            pass"""
                else:
                    if len(rawTime) > 2:
                        leftDistance.reverse()
                        if len(leftDistance) > len(rightDistance):
                            leftDistance = leftDistance[:len(rightDistance)]
                        else:
                            rightDistance = rightDistance[:len(leftDistance)]
                        for i,j in leftDistance,rightDistance:
                            pose.append(float(j)/float(i))
                        allPose = sum(pose)
                        averagePose = allPose/len(allPose)

                        if averagePose > 1:
                            if allPose[:len(Pose)//2] > allPose[len(allPose//2)]:
                                twist.linear.x = 0.1
                                twist.angular.z =  averagePose* 1.57/6.0
                            else:
                                twist.linear.x = 0.1
                                twist.angular.z = averagePose* 1.57/1.5
                        else:
                            if allPose[:len(Pose)//2] > allPose[len(allPose//2)]:
                                twist.linear.x = 0.1
                                twist.angular.z =  -1/averagePose* 1.57/6.0
                            else:
                                twist.linear.x = 0.1
                                twist.angular.z = -1/averagePose* 1.57/1.5
                        
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
                twist.angular.z = 0.

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
    sumTopDis = [ ]
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
