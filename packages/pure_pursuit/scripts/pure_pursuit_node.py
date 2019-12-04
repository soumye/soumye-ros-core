#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from duckietown_msgs.msg import SegmentList
from duckietown_msgs.msg import Twist2DStamped 
import numpy as np

def callback_orig(data):
    """
    The algorithm to follow should be roughly the following:

    Filter the line detections to find the white ones and yellow ones that are "close" to your lookahead distance L
    Use these line detections to calculate an estimate of \alpha (I suggest to average between the white and yellow ones)
    Encapsulate this in a node, wire things up, and try it on the robot.

    """
    # rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)
    print('Call back Pure Pursuit')
    # print('len', len(data.segments), data.segments[0].points)
    n = len(data.segments)
    white = []
    yellow = []
    for seg in data.segments:
        if seg.color == 0:
            white.append(seg.points)
        elif seg.color == 1:
            yellow.append(seg.points)
        
    if len(white) == 0 and len(yellow) == 0:
        x = .1
        y = 0
    elif len(white)>=len(yellow):
        print("follow white")
        x = 0
        y = 0
        for b in white:
            x += (b[0].x + b[1].x)/2
            y += (b[0].y + b[1].y)/2
        x = x/len(white)
        y = y/len(white)
        y = y + .15
    elif len(yellow) > len(white):
        print("follow yellow")
        x = 0
        y = 0
        for b in yellow:
            x += (b[0].x + b[1].x)/2
            y += (b[0].y + b[1].y)/2
        x = x/len(yellow)
        y = y/len(yellow)
        y = y - .15
    else:
        x = .1
        y = 0
    print("x, y",x, y)
    L = np.sqrt(x**2 + y**2)
    alpha = np.arctan2(y,x)
    v = 0.2
    og = 2

    omega = 2*v*(np.sin(alpha))/L
    omega = omega*og

    msg = Twist2DStamped()
    msg.v = v
    msg.omega = omega
    print('sending: v, omega', v, omega)
    pub.publish(msg)

def callback(data):
    """
    The algorithm to follow should be roughly the following:

    Filter the line detections to find the white ones and yellow ones that are "close" to your lookahead distance L
    Use these line detections to calculate an estimate of \alpha (I suggest to average between the white and yellow ones)
    Encapsulate this in a node, wire things up, and try it on the robot.

    """
    # rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)
    print('Call back Pure Pursuit')
    # print('len', len(data.segments), data.segments[0].points)
    n = len(data.segments)
    white = []
    yellow = []

    v = 0.14
    og = 2

    for seg in data.segments:
        if seg.color == 0:
            white.append(seg.points)
        elif seg.color == 1:
            yellow.append(seg.points)
    
    if len(white) == 0 and len(yellow) == 0:
        x = .1
        y = 0
    elif len(white)>=1.1*len(yellow):
        print("follow white")
        x = 0
        y = 0
        for b in white:
            x += (b[0].x + b[1].x)/2
            y += (b[0].y + b[1].y)/2
        x = x/len(white)
        y = y/len(white)
        y = y + .10
    elif 1.1*len(yellow) > len(white):
        print("follow yellow")
        x = 0
        y = 0
        for b in yellow:
            x += (b[0].x + b[1].x)/2
            y += (b[0].y + b[1].y)/2
        x = x/len(yellow)
        y = y/len(yellow)
        y = y - .12
    else:
        x = .1
        y = 0
    print("x, y",x, y)
    L = np.sqrt(x**2 + y**2)
    alpha = np.arctan2(y,x)

    if np.abs(x) > 0.22:
        print("gofast")
        v = 0.6
        og = 1

    # v = 0.2
    # og = 3.5  
    # if np.abs(x) > 0.2:
    #     v = 8.0
    #     og = 1/2
    omega = 2*v*(np.sin(alpha))/L
    omega = omega*og


    msg = Twist2DStamped()
    msg.v = v
    msg.omega = omega
    print('sending: v, omega', v, omega)
    pub.publish(msg)


def listener():
    # From ROS website
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    # rospy.init_node('pure_pursuit_node', anonymous=True)

    # rospy.Subscriber('/default/lane_filter_node/seglist_filtered', SegmentList, callback)
    rospy.Subscriber('~seglist_filtered', SegmentList, callback)


    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('pure_pursuit_node', anonymous=True)
    # pub = rospy.Publisher('/default/joy_mapper_node/car_cmd', Twist2DStamped, queue_size=10)
    pub = rospy.Publisher('~car_cmd', Twist2DStamped, queue_size=10)

    listener()