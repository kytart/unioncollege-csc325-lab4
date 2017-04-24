#!/usr/bin/env python
import rospy, math
from std_msgs.msg import String
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)


class Robot:
    def __init__(self):
        self.init = False
        self.turtlesim_pose = Pose()

    def poseCallback(self, data):
        self.turtlesim_pose.x = data.x
        self.turtlesim_pose.y = data.y
        self.turtlesim_pose.theta = data.theta

    def setDesiredOrientation(self, desired_angle_radians):
        tolerance = 5  # degrees

        desired_angle_degrees = math.degrees(desired_angle_radians) % 360
        current_angle_degrees = math.degrees(self.turtlesim_pose.theta) % 360

        relative_angle = abs(desired_angle_degrees - current_angle_degrees)

        # recursively repeat the process until the difference is less than tolerance
        if relative_angle > tolerance:
            isClockwise = (
                (current_angle_degrees > desired_angle_degrees and relative_angle <= 180) or
                (current_angle_degrees < desired_angle_degrees and relative_angle >= 180)
            )

            if relative_angle > 180:
                relative_angle = 360 - relative_angle

            rotate(degrees2radians(relative_angle), isClockwise)
            self.setDesiredOrientation(desired_angle_radians)

    def moveGoal(self, goal_pose, distance_tolerance):

        rate = rospy.Rate(10)
        vel_msg = Twist()

        linear_multiplier = 1.5
        angular_multiplier = 4.0
        angular_tolerance = 0.2

        while not rospy.is_shutdown():

            distance = findDistance(goal_pose.x, self.turtlesim_pose.x, goal_pose.y, self.turtlesim_pose.y)

            if distance < distance_tolerance:
                break

            steering_angle = math.atan2(goal_pose.y - self.turtlesim_pose.y, goal_pose.x - self.turtlesim_pose.x)
            steering_angle = degrees2radians(math.degrees(steering_angle) % 360)
            current_angle = degrees2radians(math.degrees(self.turtlesim_pose.theta) % 360)
            angular_velocity = angular_multiplier * (steering_angle - current_angle)

            if abs(angular_velocity) < angular_tolerance:
                angular_velocity = 0

            vel_msg.linear.x = distance * linear_multiplier
            vel_msg.angular.z = angular_velocity

            pub.publish(vel_msg)
            rate.sleep()

        vel_msg.linear.x = 0
        vel_msg.angular.z = 0

        pub.publish(vel_msg)


def findDistance(x1, x0, y1, y0):
    return math.sqrt((x1 - x0) ** 2 + (y1 - y0) ** 2)


def degrees2radians(angle):
    return angle * (math.pi / 180.0)


def move(distance, isForward):
    speed = 1.0
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
    speed = degrees2radians(4.0)
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


def moveRobot(robot, x, y):
    rate = rospy.Rate(100)
    distance_tolerance = 0.1

    goal_pose = Pose()
    goal_pose.x = x
    goal_pose.y = y
    goal_pose.theta = 0.0
    robot.moveGoal(goal_pose, distance_tolerance)
    rate.sleep()


def drawSquare(robot, square_size):
    # move to the upper left corner
    robot.setDesiredOrientation(degrees2radians(90))
    move(square_size, True)

    # move to the upper right corner
    robot.setDesiredOrientation(degrees2radians(0))
    move(square_size, True)

    # move to the lower right corner
    robot.setDesiredOrientation(degrees2radians(270))
    move(square_size, True)

    # move to the lower left corner
    robot.setDesiredOrientation(degrees2radians(180))
    move(square_size, True)


def init(robot):
    rospy.init_node('robotMover', anonymous=True)
    rospy.Subscriber("/turtle1/pose", Pose, robot.poseCallback)


if __name__ == '__main__':
    robot = Robot()
    init(robot)
    drawSquare(robot, 3.0)
