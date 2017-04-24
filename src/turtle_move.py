#!/usr/bin/env python
import rospy, math
from geometry_msgs.msg import Twist

# NOTE: move_goal won't work because it needs to know where are the goal coordinates
# and where is the turtlebot currently to calculate the linear and angular velocity.
# In Gazebo there is no absolute coordinate system or at least there are no absolute coordinates published
# by the running nodes so I can't use it.

pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=10)


def degrees2radians(angle):
    return angle * (math.pi / 180.0)


def move(distance, isForward):
    speed = 0.2
    outData = Twist()
    t0 = rospy.get_rostime().secs
    current_distance = 0
    rate = rospy.Rate(10)

    while not rospy.is_shutdown() and t0 == 0:
        t0 = rospy.get_rostime().secs

    outData.linear.x = speed if isForward else -speed

    while not rospy.is_shutdown() and current_distance < distance:
        pub.publish(outData)

        t1 = rospy.get_rostime().secs
        current_distance = speed * (t1 - t0)

        rate.sleep()

    outData.linear.x = 0.0
    pub.publish(outData)
    rate.sleep()


def rotate(relative_angle, isClockwise):
    speed = degrees2radians(30.0)
    outData = Twist()

    t0 = rospy.get_rostime().secs
    current_angle = 0
    rate = rospy.Rate(10)

    while not rospy.is_shutdown() and t0 == 0:
        t0 = rospy.get_rostime().secs

    outData.angular.z = -speed if isClockwise else speed

    while not rospy.is_shutdown() and current_angle < relative_angle:
        pub.publish(outData)

        t1 = rospy.get_rostime().secs
        current_angle = speed * (t1 - t0)

        rate.sleep()

    outData.angular.z = 0.0
    pub.publish(outData)
    rate.sleep()


def drawSquare(square_size):
    for x in range(0, 4):
        if x > 0:
            rotate(degrees2radians(90), True)

        move(square_size, True)


def init():
    rospy.init_node('turtle_mover', anonymous=True)


if __name__ == '__main__':
    init()
    drawSquare(2.0)  # for real turtlebot I changed this to 1.0
