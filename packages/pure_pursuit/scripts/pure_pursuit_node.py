#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from duckietown_msgs.msg import SegmentList
from duckietown_msgs.msg import Twist2DStamped 
import numpy as np

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
    for seg in data.segments:
        if seg.color == 0:
            white.append(seg.points)
        elif seg.color == 1:
            yellow.append(seg.points)
    
    if len(white)>len(yellow):
        x = 0
        y = 0
        for b in white:
            x += (b[0].x + b[1].x)/2
            y += (b[0].y + b[1].y)/2
        x = x/len(white)
        y = y/len(white)
        y = y + .2
    elif len(yellow) > len(white):
        x = 0
        y = 0
        for b in yellow:
            x += (b[0].x + b[1].x)/2
            y += (b[0].y + b[1].y)/2
        x = x/len(yellow)
        y = y/len(yellow)
        y = y - .2
    else:
        x = .1
        y = 0
    L = np.sqrt(x**2 + y**2)
    alpha = np.arctan2(y,x)
    v = 0.2
    omega = 2*v*(np.sin(alpha))/L

    msg = Twist2DStamped()
    print('sending: v, omega', v, omega)
    msg.v = v
    msg.omega = omega*4
    pub.publish(msg)


def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('pure_pursuit_node', anonymous=True)

    rospy.Subscriber('/walle/lane_filter_node/seglist_filtered', SegmentList, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    pub = rospy.Publisher('/walle/joy_mapper_node/car_cmd', Twist2DStamped, queue_size=10)
    listener()