#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from duckietown_msgs.msg import SegmentList
from duckietown_msgs.msg import Twist2DStamped 
from sensor_msgs.msg import CompressedImage, Image
import numpy as np
from duckietown_utils.jpg import bgr_from_jpg
import cv2
import cv2 as cv
import time
from cv_bridge import CvBridge, CvBridgeError
# from  ground_projection.ground_projection_interface import GroundProjection, get_ground_projection_geometry_for_robot
from  ground_projection.GroundProjection import GroundProjection

import duckietown_utils as dtu

stop = False

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
    global stop
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
        # print("follow white")
        x = 0
        y = 0
        for b in white:
            x += (b[0].x + b[1].x)/2
            y += (b[0].y + b[1].y)/2
        x = x/len(white)
        y = y/len(white)
        y = y + .10
    elif len(yellow) > len(white):
        # print("follow yellow")
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
    # print("x, y",x, y)
    L = np.sqrt(x**2 + y**2)
    alpha = np.arctan2(y,x)
    v = 0.3
    # v = 0
    og = 2

    omega = 2*v*(np.sin(alpha))/L
    omega = omega*og

    msg = Twist2DStamped()
    if stop is True:
        msg.v = 0
        msg.omega = 0
        print("NOT SENDING SHITTT.")
    else:
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

    v = 0.25
    og = 2

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
        y = y + .10
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

    if np.abs(x) > 0.24:
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

def processImage(image_msg):
    global stop
    image_size = [120,160]
    # top_cutoff = 40

    start_time = time.time()
    try:
        image_cv = bgr_from_jpg(image_msg.data)
    except ValueError as e:
        print("image decode error", e)
        return
    
    # Resize and crop image
    hei_original, wid_original = image_cv.shape[0:2]

    if image_size[0] != hei_original or image_size[1] != wid_original:
        image_cv = cv2.resize(image_cv, (image_size[1], image_size[0]),
                                interpolation=cv2.INTER_NEAREST)
    # image_cv = image_cv[top_cutoff:,:,:]
    # image_cv = cv.medianBlur(image_cv,5)
    hsv = cv.cvtColor(image_cv, cv.COLOR_BGR2HSV)

    # hsv_obs_yellow1 = np.array([15,210,85])
    # hsv_obs_yellow2 = np.array([35,250,255])

    hsv_obs_red1 = np.array([0,140, 100])
    hsv_obs_red2 = np.array([15,255,255])
    hsv_obs_red3 = np.array([165,140, 100]) 
    hsv_obs_red4 = np.array([180,255,255])

    # hsv_obs_red1 = np.array([0,90, 15])
    # hsv_obs_red2 = np.array([10,255,255])
    # hsv_obs_red3 = np.array([150,70, 0]) 
    # hsv_obs_red4 = np.array([255,255,255])
    # Just for dots on bot      
    # hsv_obs_black1 = np.array([0,0,0])
    # hsv_obs_black2 = np.array([255,255,15])
    # #Just for board containing dots
    # hsv_obs_white1 = np.array([25,0,90])
    # hsv_obs_white2 = np.array([255,12,115])
    # dw = cv.inRange(hsv, hsv_obs_yellow1, hsv_obs_yellow2)

    bw1 = cv.inRange(hsv, hsv_obs_red1, hsv_obs_red2)
    bw2 = cv.inRange(hsv, hsv_obs_red3, hsv_obs_red4)
    # bw3 = cv.inRange(hsv, hsv_obs_black1, hsv_obs_black2)
    # bw4 = cv.inRange(hsv, hsv_obs_white1, hsv_obs_white2)
    bw = cv.bitwise_or(bw1, bw2)
    # bw = cv.bitwise_or(bw, bw3)
    # bw = cv.bitwise_or(bw, bw4)
    cnts = cv2.findContours(bw.copy(),
                              cv2.RETR_EXTERNAL,
                              cv2.CHAIN_APPROX_SIMPLE)[-2]

    if len(cnts)>0:
        print('object detected')
        red_area = max(cnts, key=cv2.contourArea)
        (xg,yg,wg,hg) = cv2.boundingRect(red_area)
        box_img = cv2.rectangle(image_cv,(xg,yg),(xg+wg, yg+hg),(0,255,0),2)
        print('BEFORE X', [xg, xg+wg], " BEFORE Y", [yg+hg, yg+hg])

        # x_arr, y_arr = gpg.point2ground([xg, xg+wg], [yg + hg, yg + hg], image_size[0], image_size[1])
        # print("BOTTOM OF ROBOT : X ", x_arr, ' Y :', y_arr)
        # if x_arr[0] < 0.3:
        if yg+hg > 60:
            print("STOP THE FUCKIN Bot")
            stop = True
        else 
            stop = False
        image_msg_out = bridge.cv2_to_imgmsg(box_img, "bgr8")
        # image_msg_out = bridge.cv2_to_imgmsg(bw, "mono8")
        image_msg_out.header.stamp = image_msg.header.stamp
        pub_image.publish(image_msg_out)
    else:
        stop = False
        image_msg_out = bridge.cv2_to_imgmsg(image_cv, "bgr8")
        # image_msg_out = bridge.cv2_to_imgmsg(bw, "mono8")
        image_msg_out.header.stamp = image_msg.header.stamp
        pub_image.publish(image_msg_out)
    
    print('Time to process', time.time() - start_time)

def callback_charan(seglist_msg):

    global stop
        
    #initialize variables
    yellow_offset, white_offset, omega_gain = -0.12, 0.15, 2
    white_seg_count, yellow_seg_count = 0, 0
    white_x_accumulator, white_y_accumulator, yellow_x_accumulator, yellow_y_accumulator = 0.0, 0.0, 0.0, 0.0
    white_centroid_x, white_centroid_y, yellow_centroid_x, yellow_centroid_y = 0.0, 0.0, 0.0, 0.0

    for segment in seglist_msg.segments:
        #the point is behind us
        if segment.points[0].x < 0 or segment.points[1].x < 0: 
            continue

        #calculate white segments sum, count values
        if segment.color == segment.WHITE:
            white_x_accumulator += (segment.points[0].x + segment.points[1].x) / 2
            white_y_accumulator += (segment.points[0].y + segment.points[1].y) / 2 
            white_seg_count += 1.0
        #calculate yellow segments sum, count values
        elif segment.color == segment.YELLOW:
            yellow_x_accumulator += (segment.points[0].x + segment.points[1].x) / 2
            yellow_y_accumulator += (segment.points[0].y + segment.points[1].y) / 2 
            yellow_seg_count += 1.0
        #skip red segments
        else:
            continue

    #calculate centroid for white segments
    if white_seg_count > 0:
        white_centroid_x, white_centroid_y = white_x_accumulator/white_seg_count, white_y_accumulator/white_seg_count

    #calculate centroid for yellow segments
    if yellow_seg_count > 0:
        yellow_centroid_x, yellow_centroid_y = yellow_x_accumulator/yellow_seg_count, yellow_y_accumulator/yellow_seg_count

    #if white seg count is greater, trust white line segments
    if  white_seg_count >  yellow_seg_count:   
        follow_point_x = white_centroid_x
        follow_point_y = white_centroid_y + white_offset

    #if yellow seg count is greater, trust yellow line segments
    elif  yellow_seg_count > white_seg_count:  
        follow_point_x = yellow_centroid_x
        follow_point_y = yellow_centroid_y + yellow_offset
    
    #if both are equal, take average
    else:
        follow_point_x = 0.5 * (white_centroid_x + yellow_centroid_x)
        follow_point_y = 0.5 * (white_centroid_y + yellow_centroid_y)
        #check if they are zero, because they might become zero if no white/yellow segments are encountered
        if follow_point_x == 0 and follow_point_y == 0:
            follow_point_x, follow_point_y = 0.1, 0

    #tan_alpha = y/x => alpha = tan-1(y/x)
    alpha = np.arctan2(follow_point_y, follow_point_x)
    lookahead_dist = np.sqrt(follow_point_x * follow_point_x + follow_point_y * follow_point_y)
    #calculating v, omega

    
    if np.abs(follow_point_y) > 0.2:
        v, omega_gain = 0.25, 3
    else:
        if np.abs(follow_point_x) >= 0.55:
            v, omega_gain = 0.7, 1.5
        elif np.abs(follow_point_x) > 0.48 and np.abs(follow_point_x) < 0.55:
            v, omega_gain = 0.4, 1.5
        else:
            v, omega_gain = 0.25, 2

    omega  =  2 * v * np.sin(alpha) / lookahead_dist        

    #publishing to car_cmd topic
    car_control_msg = Twist2DStamped()
    if stop is True:
        car_control_msg.v = 0
        car_control_msg.omega = 0
        print("NOT SENDING SHITTT. V= 0, omega = 0")
    else:
        car_control_msg.v = v
        car_control_msg.omega = omega * omega_gain
        print('sending: v, omega', v, omega)
    pub.publish(car_control_msg)

if __name__ == '__main__':
    rospy.init_node('pure_pursuit_node', anonymous=True)
    pub = rospy.Publisher('~car_cmd', Twist2DStamped, queue_size=10)
    pub_image = rospy.Publisher("~image_with_object", Image, queue_size=1)
    bridge = CvBridge()
    global stop
    stop = False
    robot_name = rospy.get_param("~config_file_name", None)
    if robot_name is None:
        robot_name = dtu.get_current_robot_name()
    gpg = GroundProjection(robot_name)
    rospy.Subscriber('~seglist_filtered', SegmentList, callback_charan)
    rospy.Subscriber("~corrected_image/compressed", CompressedImage, processImage, queue_size=1)
rospy.spin()