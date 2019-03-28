#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, PoseStamped
import tf
import rostopic
import tf
from math import pi, pow, sqrt, cos, sin, acos, atan2
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from tf.transformations import euler_from_quaternion
import time



class Robot:

    def __init__(self):
        """"
        Set up the node here

        """
        rospy.init_node("robot", anonymous=True)

        # gets odometry
        sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)

        # allows for publishing of vel commands
        self.vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

        # get location of click
        new_loc = rospy.Subscriber("/new_goal", PoseStamped, self.nav_to_pose)

        # init robot variables
        self.vel_msg = Twist()
        self.px = 0.0
        self.py = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

        self.tf_listener = tf.TransformListener()
        #rospy.Timer(rospy.Duration(0.1), self.timerCallback)




    def find_travel_param(self, x1, y1, yaw, x2, y2):

        """
        function to determine angle of rotation needed in rad to
        "face" a point, and distance needed to get there
        :param x1: location of start x pos
        :param y1: location of start y pos
        :param x2: location of end x pos
        :param y2: location of end y pos
        :return: angle between robot and point, distance between them
        """
		# calculate magnitude of travel and reference point
        mag_point = sqrt(pow(x2-x1, 2) + pow(y2-y1, 2))
        rad_x1 = (cos(yaw) * mag_point)
        rad_y1 = (sin(yaw) * mag_point)
	
		# find angle of rotation
		new_angle = atan2(y2-y1, x2-x1) - yaw


        dot_prod = rad_x1*(x2 - x1) + rad_y1*(y2-y1)
	
	return new_angle, mag_point
	




    def nav_to_pose(self, goal):
        # type: (PoseStamped) -> None
        """
        This is a callback function. It should extract data from goal, drive in a straight line to reach the goal and
        then spin to match the goal orientation.
        :param goal: PoseStamped
        :return:
        """
		# get desired end location
        goal_x = goal.pose.position.x
        goal_y = goal.pose.position.y
        quat = goal.pose.orientation
        q = [quat.x, quat.y, quat.z, quat.w]
        goal_roll, goal_pitch, goal_yaw = euler_from_quaternion(q)

        # calc angle between current pose and click
        print "click!"
        x = self.px
        y = self.py
        yaw = self.yaw
		
		# perform required trig
        angle, distance = self.find_travel_param(x, y, yaw, goal_x, goal_y)

        print "goal_y"
        print goal_y
        print "goal x"
        print goal_x
        print "real x"
        print x
        print "real y"
        print y


		# make smallest rotation
        if abs(angle) > pi:
            if angle < 0:
                angle = angle + 2*pi
            else:
                angle = angle - 2*pi
        print angle
		
		# move to position
        self.rotate(angle)
        # time.sleep(2)
        self.drive_straight(0.1, distance)
        # time.sleep(2)
        self.rotate(goal_yaw - self.yaw)
        # set linear velocity
        self.vel_msg.linear.x = 0.0
        self.vel_msg.linear.y = 0.0
        self.vel_msg.linear.z = 0.0

        # ensure angular vel is zero
        self.vel_msg.angular.x = 0.0
        self.vel_msg.angular.y = 0.0
        self.vel_msg.angular.z = 0.0
		
		# make sure we stop
        for x in range(20):

            self.vel_pub.publish(self.vel_msg)




    def drive_straight(self, speed, distance):
        """
        Make the turtlebot drive straight
        :type speed: float
        :type distance: float
        :param speed: speed to drive
        :param distance: distance to drive
        :return:
        """

        # define start position
        start_X = self.px
        start_Y = self.py

        # set linear velocity
        self.vel_msg.linear.x = speed
        self.vel_msg.linear.y = 0.0
        self.vel_msg.linear.z = 0.0

        # ensure angular vel is zero
        self.vel_msg.angular.x = 0.0
        self.vel_msg.angular.y = 0.0
        self.vel_msg.angular.z = 0.0
        self.X_Setpoint = 0.0
        self.Y_Setpoint = 0.0

        # publish new vel message
        self.vel_pub.publish(self.vel_msg)
        traveling = True
        # travel until specified distance is reached
        while traveling:
            self.vel_pub.publish(self.vel_msg)
            #print('Loop')
            print distance - sqrt(pow((start_X-self.px), 2) + pow((start_Y - self.py), 2))
            if sqrt(pow((start_X-self.px), 2) + pow((start_Y - self.py), 2)) >= distance:
                # stop travel
            #    print('Stop')
                self.vel_msg.linear.x = 0.0
                self.vel_msg.linear.y = 0.0
                self.vel_pub.publish(self.vel_msg)
                traveling = False


    def rotate(self, angle):
        """
        Rotate in place
        :param angle: angle to rotate
        :return:
        angle specified is no larger than pi
        """
        # counter clockwise, increase, drop, incease, drop
        # clockwise, decrease, drop, decrease, drop

        # init var used in state machine
        prev_angle = self.yaw

        # determine if we are rotating clock wise or counter clockwise


        if angle < 0:
            end_angle = angle * -1
            self.vel_msg.angular.z = -0.1
            clockWise = True
        else:
            end_angle = angle
            self.vel_msg.angular.z = 0.1
            clockWise = False
        # ensure we wont move forward
        self.vel_msg.linear.x = 0.0
        self.vel_msg.linear.y = 0.0

        # publish new vel message
        self.vel_pub.publish(self.vel_msg)

        rotating = True
        traveled = 0
       # print end_angle
        # clockwise rotation logic
        if clockWise:
            while (rotating):
                self.vel_pub.publish(self.vel_msg)
                position = self.yaw
                # if robot has passed over rotation thresh, adjust previous angle
                if(position > 3) & (prev_angle < 0):
                    prev_angle = 2*pi + prev_angle

                # if a large rotation is detected, it is likely due to noise near
                # rotation threshold, ignore this measurment
                if(prev_angle - position) < 3:
                    traveled += (prev_angle - position)
                    prev_angle = position

                # if desired distance has been traveled, stop
                if traveled >= end_angle:
                    self.vel_msg.angular.z = 0.0
                    self.vel_pub.publish(self.vel_msg)
                    rotating = False

        # counter clockwise logid
        else:
            while (rotating):
                self.vel_pub.publish(self.vel_msg)
                position = self.yaw

                # check if we have entered rotation threshold, if so adjust prev angle
                if(position < -3) & (prev_angle > 0):
                    prev_angle = -2*pi + prev_angle

                # if a large distance is read, attribute to noise and ignore
                if(position - prev_angle) < 3:
                    traveled += (position - prev_angle)
                    prev_angle = position

                # if desired travel reached, stop
                if traveled >= end_angle:
                    self.vel_msg.angular.z = 0.0
                    self.vel_pub.publish(self.vel_msg)
                    rotating = False



    def odom_callback(self, msg):
        """
        update the state of the robot
        :type msg: Odom
        :return:
        """
        self.px = msg.pose.pose.position.x
        self.py = msg.pose.pose.position.y
        quat = msg.pose.pose.orientation
        q = [quat.x, quat.y, quat.z, quat.w]
        self.roll, self.pitch, self.yaw = euler_from_quaternion(q)

    def new_goal_callback(self, msg):
        """
        grab new goal location info
        :param msg:
        :return:
        """



if __name__ == '__main__':
    try:
        robot = Robot()
        # robot.drive_straight(0.1, 1)
        # robot.rotate(pi/4)
        while 1:
            pass

    except rospy.ROSInterruptException:
        pass
