#!/usr/bin/env python3
import rospy
import math
import random
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

vel = Twist()

def callback1(msg):
    global x, y, theta
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    theta = msg.pose.pose.orientation.z
    # print(x, y, theta)
    return 0

def callback2(data):

    global front_ranges, laser_ranges
    front_ranges = data.ranges[90:270]
    laser_ranges = data.ranges
    theta_inc = data.angle_increment
    theta = data.angle_min

    cart_coords = []
    for r in laser_ranges:
        if r < 3:
            x = r*math.cos(theta)
            y = r*math.sin(theta)
            cart_coords.append((x, y))
        theta += theta_inc

    get_bestfit_line(cart_coords)

    return 0

# RANSAC: retuns points of best fit line
def get_bestfit_line(points):
    global distFromWall
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

    # get robot distance from line
    x1, y1 = first_pt
    x2, y2 = second_pt
    A = y2 - y1
    B = x1 - x2
    C = (x2*y1) - (x1*y2)

    distFromWall = abs((A*x + B*y + C)/(math.sqrt(A**2 + B**2)))    
    return 0

def obstacleInWay():

    minRange = min(front_ranges)

    return minRange < 0.5

def goalSeekVel():
    # rotate towards goal and head towards it
    if abs(angleDiff) > 0.01:
        angular_vel = 1.5*angleDiff
        linear_vel = 0 
    else:
        angular_vel = 0
        if abs(linearDist) > 0.05:
            linear_vel = 0.5*linearDist
    
    return linear_vel, angular_vel


def wallFollowVel():
    
    print(distFromWall)
    # turn parralel to wall
    minRange = min(front_ranges)

    if minRange < 0.5:
        linear_vel = 0
        angular_vel = -0.5
    else:
        linear_vel = 2
        angular_vel = 0.3*minRange

    return linear_vel, angular_vel



def bug2():
    rospy.init_node("bug2")
    
    rospy.Subscriber("/odom", Odometry, callback1)
    rospy.Subscriber("/base_scan", LaserScan, callback2)

    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

    # goal coordinates
    # goalx = rospy.get_param('~goalx')
    # goaly = rospy.get_param('~goaly')
    global goalx, goaly, linearDist, angleDiff
    goalx, goaly = 7.22, 7.36

    atGoal = False
    goalSeek = True
    wallFollow = False

    rate = rospy.Rate(10)
    rospy.sleep(1)
    while not atGoal:

        # get angular and linear distance from goal
        robotAngle = math.asin(theta)
        dy,dx = (goaly - y), (goalx- x)
        slop = dy/dx

        if dx<0 and dy>0:
            goalAngle = math.pi + math.atan(slop)
        elif dx<0 and dy<0:
            goalAngle = math.atan(slop) - math.pi
        else:
            goalAngle = math.atan(slop)

        angleDiff = goalAngle - 2*robotAngle
        linearDist = math.sqrt((goalx - x)**2 + (goaly - y)**2)

        if linearDist < 0.5:
            print("at Goal!")
            linear_vel = 0
            angular_vel = 0
        else:
            if goalSeek:
                linear_vel, angular_vel = goalSeekVel()
                if obstacleInWay():
                    wallFollow = True
                    goalSeek = False
            
            if wallFollow:
                linear_vel, angular_vel = wallFollowVel()

        vel.linear.x = linear_vel
        vel.angular.z = angular_vel
        
        pub.publish(vel)

        rate.sleep()
    

if __name__ == '__main__':
    try:
        bug2()
    except rospy.ROSInterruptException:
        pass
