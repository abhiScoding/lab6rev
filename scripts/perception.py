#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import  LaserScan
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
import math
import random


# RANSAC: retuns points of best fit line
def get_bestfit_line(points):
    global first_pt, second_pt
    max_inliers = 0
    iterations = 20

    if len(points) > 0:
        for i in range(iterations):
            p1 = random.choice(points)
            p2 = random.choice(points)
            
            if p1 != p2:
                # get line from p1 and p2 : Ax + By + C = 0
                x1, y1 = p1
                x2, y2 = p2
                A = y2 - y1
                B = x1 - x2
                C = (x2*y1) - (x1*y2)

                # calculate number of inliers
                inliers = 0
                min_dist = 15

                for point in points:
                    x3, y3 = point
                    dist = abs((A*x3 + B*y3 + C)/(math.sqrt(A**2 + B**2)))
                    if dist < min_dist:
                        inliers += 1

                # update max_inliers
                if inliers > max_inliers:
                    max_inliers = inliers
                    first_pt = p1
                    second_pt = p2

    # print("two poins:", first_point, second_point)       
    return first_pt, second_pt

def callback(data):
    global laser_ranges, cart_coords
    laser_ranges = data.ranges
    theta_inc = data.angle_increment

    theta = 0
    cart_coords = []

    for r in laser_ranges:
        if r < 3:
            x = r*math.cos(theta)
            y = r*math.sin(theta)
            cart_coords.append((x, y))
        theta += theta_inc

    get_bestfit_line(cart_coords)
    return 0

def perception():
    rospy.init_node('perception')
    rospy.Subscriber('/base_scan', LaserScan, callback)
    pub_marker = rospy.Publisher("/marker", Marker, queue_size = 10)
    # pub_laser = rospy.Publisher("/laser", LaserScan, queue_size = 10)
    rospy.sleep(1)

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        
        X1, Y1 = first_pt
        X2, Y2 = second_pt
        # print("X1, Y1, X2, Y2: ", X1, Y1, X2, Y2)

        # marker message
        # defining marker obj.
        marker = Marker()
        
        # marker message arguments
        marker.header.frame_id = "base_link"
        marker.header.stamp = rospy.Time.now()
        marker.type = marker.LINE_STRIP
        marker.action = marker.ADD
        marker.lifetime = rospy.Duration(0)

        # line start and end pots
        marker.points = []
    
        # first point
        first_point = Point()
        first_point.x = X1
        first_point.y = Y1
        first_point.z = 0.0
        marker.points.append(first_point)
    
        # second point
        second_point = Point()
        second_point.x = X2
        second_point.y = Y2
        second_point.z = 0.0
        marker.points.append(second_point)

        # Set Marker scale
        marker.scale.x = 0.03
        marker.scale.y = 0.03
        marker.scale.z = 0.03

        # Set Marker color
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 1.0

        # pub_laser.publish(laser_ranges)
        pub_marker.publish(marker)

        rate.sleep()

    rospy.spin()


if __name__ == '__main__':
    try:
        perception()
    except rospy.ROSInterruptException:
        pass